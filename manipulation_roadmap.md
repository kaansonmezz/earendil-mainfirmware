# Manipulation UART8 DMA Roadmap

## 1. Amaç

Bu roadmap, mevcut STM32H723ZG ana kontrolcü projesine robot kol / manipulation F411 modülü için yeni bir UART katmanı eklemek içindir.

Yeni katman şu davranışı sağlamalıdır:

- H7 terminalinden gelen `arm` prefix'li satırları alır.
- Prefix'ten sonraki payload kısmını bozmadan UART8 üzerinden manipulation F411 modülüne gönderir.
- UART8 RX tarafını DMA + ReceiveToIdle ile dinler.
- UART8 TX tarafını DMA ile gönderir.
- UART8 hata durumlarını motor UART hata sistemine benzer formatta raporlar.
- GUI tarafında UART8 / ARM hata durumları motorlardan ayrı gösterilir.

Bu roadmap bir komut protokolü tasarımı değildir. H7 yalnızca `arm` prefix'inden sonraki payload'u manipulation F411'e forward eder.

---

## 2. Sabit komut formatı

Terminal formatı:

```text
arm <payload>
```

Payload UART8'e şu şekilde gönderilir:

```text
<payload>\r\n
```

Kabul edilen örnek kullanım biçimleri yalnızca şunlardır:

```text
arm forward 1 200
arm set 1 stopmode brake
```

Bu roadmap içinde başka manipulation komutu tanımlanmayacak.

---

## 3. Mevcut proje durumu

Yüklenen son proje incelendiğinde UART8 donanım tarafı büyük ölçüde hazır durumdadır.

### 3.1 UART8 pinleri

| Sinyal | Pin |
|---|---|
| UART8_RX | PE0 |
| UART8_TX | PE1 |

### 3.2 UART8 haberleşme ayarı

| Ayar | Değer |
|---|---|
| Peripheral | UART8 |
| Mode | TX/RX asynchronous |
| Baudrate | 115200 |
| Word length | 8 bit |
| Stop bit | 1 |
| Parity | None |
| Hardware flow control | None |
| FIFO mode | Disabled |

### 3.3 UART8 DMA ayarı

| Akış | DMA |
|---|---|
| UART8_RX | DMA2 Stream0 |
| UART8_TX | DMA2 Stream1 |

### 3.4 NVIC durumu

| IRQ | Priority |
|---|---:|
| UART8_IRQn | 6 |
| DMA2_Stream0_IRQn | 6 |
| DMA2_Stream1_IRQn | 6 |

### 3.5 Mevcut eksik

Donanım ve HAL init tarafı hazır olsa da uygulama katmanı eksiktir:

- UART8 için ayrı RX line parser yok.
- UART8 için ayrı TX DMA queue yok.
- UART8 için motor UART tarzı hata raporlama yok.
- Mevcut `HAL_UARTEx_RxEventCallback()` sadece motor UART slotlarını işliyor.
- Mevcut `HAL_UART_ErrorCallback()` sadece motor UART slotlarını işliyor.
- Mevcut `HAL_UART_TxCpltCallback()` motor TX DMA katmanına bağlı.
- Terminal parser içinde `arm` prefix'i yok.
- GUI error parser sadece motor UART hatalarını tanıyor.

---

## 4. Tasarım kararı

UART8, motor sistemine eklenmeyecek.

Yapılmayacaklar:

- `MotorId_t` içine yeni motor eklenmeyecek.
- `MOTOR_COUNT` değiştirilmeyecek.
- `motor_dispatcher` içine UART8 eklenmeyecek.
- `motor_tuning_config` içine UART8 eklenmeyecek.
- Motor telemetry parser UART8 için kullanılmayacak.
- Motor tablosuna sahte beşinci motor satırı eklenmeyecek.

Yapılacaklar:

- UART8 için bağımsız bir manipulation UART DMA modülü açılacak.
- Terminal tarafında `arm` prefix'i parse edilecek.
- Payload korunarak UART8'e DMA TX ile basılacak.
- UART8 RX satırları ayrı log formatıyla terminale basılacak.
- UART8 error bilgisi motorlardan ayrı tutulacak.
- GUI'de ARM UART durumu ayrı gösterilecek.

---

## 5. Yeni dosya yapısı

Eklenecek dosyalar:

```text
Core/Inc/manipulation_uart_dma.h
Core/Src/manipulation_uart_dma.c
```

Önerilen isimlendirme:

- Modül adı: `ManipulationUartDma`
- Terminal prefix'i: `arm`
- Fiziksel kanal: `UART8`
- Log etiketi: `ARM`

---

## 6. Public API planı

`manipulation_uart_dma.h` içinde şu API bulunmalı:

```c
#ifndef MANIPULATION_UART_DMA_H
#define MANIPULATION_UART_DMA_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

void ManipulationUartDma_Init(void);
void ManipulationUartDma_StartRx(void);
void ManipulationUartDma_Update(void);

bool ManipulationUartDma_SendRaw(const char *payload);

bool ManipulationUartDma_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size);
bool ManipulationUartDma_HandleError(UART_HandleTypeDef *huart);
void ManipulationUartDma_OnTxComplete(UART_HandleTypeDef *huart);

#endif
```

Bu API, HAL callback'lerini doğrudan define etmeyecek. Mevcut callback'ler router gibi kullanılacak.

---

## 7. DMA buffer planı

UART8 RX/TX buffer'ları DMA-safe memory section içinde tutulmalı.

Öneri:

```c
#define MANIP_DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
```

Buffer önerileri:

| Buffer | Boyut | Amaç |
|---|---:|---|
| RX DMA buffer | 128 byte | ReceiveToIdle DMA ham alım |
| RX line buffer | 160 byte | Satır birleştirme |
| TX active buffer | 160 byte | Aktif DMA TX paketi |
| TX queue slot | 160 byte | Bekleyen TX payload |
| TX queue depth | 4 veya 8 | Arka arkaya gelen terminal komutları |

Notlar:

- DMA buffer'ları 32-byte aligned olmalı.
- D-cache kullanılan H7 mimarisinde RX/TX DMA buffer cache coherency dikkatli ele alınmalı.
- Mevcut projedeki `.dma_buffer` yaklaşımı korunmalı.
- RX line overflow olursa sistem kilitlenmemeli; dropped counter artırılmalı.

---

## 8. RX DMA çalışma mantığı

### 8.1 Başlatma

`ManipulationUartDma_StartRx()` şu işi yapmalı:

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart8, rx_dma_buffer, sizeof(rx_dma_buffer));
__HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
```

Half-transfer interrupt kapatılmalı. Line bazlı ReceiveToIdle yeterlidir.

### 8.2 RX event akışı

1. UART8 RX event gelir.
2. Callback router `ManipulationUartDma_HandleRxEvent()` çağırır.
3. Gelen byte'lar line buffer'a eklenir.
4. `\n` görülünce line tamamlanır.
5. Tamamlanan line terminale loglanır.
6. RX DMA tekrar başlatılır.

Önerilen RX log formatı:

```text
[ARM_RX] <line>
```

### 8.3 RX parser sınırları

Manipulation F411'den gelen veri, H7 tarafında yorumlanmayacak.

H7 sadece:

- Satırı alır.
- Loglar.
- GUI'nin görebileceği formatta terminale basar.

İçerik anlamlandırma sonraki aşamaya bırakılır.

---

## 9. TX DMA çalışma mantığı

### 9.1 Terminalden gelen payload

Terminal satırı:

```text
arm <payload>
```

UART8'e gidecek veri:

```text
<payload>\r\n
```

### 9.2 TX queue

Motor TX DMA mantığına benzer şekilde küçük bir queue kullanılmalı.

Gerekçe:

- GUI veya terminal arka arkaya satır gönderebilir.
- UART8 TX DMA busy iken ikinci satır kaybolmamalı.
- `HAL_BUSY` durumunda doğrudan fail etmek yerine sıraya almak daha güvenlidir.

TX queue davranışı:

| Durum | Davranış |
|---|---|
| TX idle | Payload active buffer'a alınır ve DMA başlatılır |
| TX busy | Payload queue içine alınır |
| Queue dolu | Dropped counter artırılır, kontrollü warning basılır |
| TX complete | Busy clear edilir, queue'daki sonraki payload gönderilir |

Önerilen TX log formatı:

```text
[ARM_TX] <payload>
```

---

## 10. UART8 error handling planı

UART8 hata sistemi motor UART hata sistemine benzer olmalı ama motor state'e bağlanmamalı.

### 10.1 Hata kaynakları

İzlenecek HAL hata bayrakları:

| HAL flag | Anlam |
|---|---|
| `HAL_UART_ERROR_PE` | Parity error |
| `HAL_UART_ERROR_NE` | Noise error |
| `HAL_UART_ERROR_FE` | Framing error |
| `HAL_UART_ERROR_ORE` | Overrun error |
| `HAL_UART_ERROR_DMA` | DMA error |
| `HAL_UART_ERROR_RTO` | Receiver timeout |

### 10.2 Error log formatı

Motor UART hata loglarına benzer format kullanılmalı:

```text
[ERROR] UART8 UART error code: 0x00000004
[ERROR] UART8 error: FE - Framing error
[INFO] UART8 RX recovered after UART error
```

### 10.3 Spam engelleme

Hata hâlâ çözülmediyse her loop'ta basılmamalı.

Öneri:

| Durum | Loglama |
|---|---|
| İlk hata | Hemen bas |
| Hata devam ediyor | 5000 ms'de bir tekrar bas |
| RX recovery | Bir kere recovery logu bas |
| Hata temizlendi | Error state clear |

### 10.4 ISR içinde ağır iş yapılmamalı

`HAL_UART_ErrorCallback()` içinde yapılacaklar minimum olmalı:

- UART8 mi kontrol et.
- Error code kaydet.
- Recovery needed flag set et.
- Gerekirse `HAL_UART_AbortReceive()` çağrısı dikkatli kullanılmalı.
- Asıl loglama ve restart `ManipulationUartDma_Update()` içinde yapılmalı.

---

## 11. Callback routing planı

Mevcut projede HAL callback'leri zaten motor UART dosyalarında tanımlı olduğu için yeni manipulation modülü içinde tekrar callback define edilmemeli.

### 11.1 RX callback

Mevcut `HAL_UARTEx_RxEventCallback()` içine UART8 routing eklenmeli.

Akış:

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (ManipulationUartDma_HandleRxEvent(huart, Size)) {
        return;
    }

    /* existing motor UART RX behavior */
}
```

### 11.2 Error callback

Mevcut `HAL_UART_ErrorCallback()` içine UART8 routing eklenmeli.

Akış:

```c
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (ManipulationUartDma_HandleError(huart)) {
        return;
    }

    /* existing motor UART error behavior */
}
```

### 11.3 TX complete callback

Mevcut `HAL_UART_TxCpltCallback()` içinde motor TX davranışı korunmalı, UART8 için ek çağrı yapılmalı.

Akış:

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    MotorTxDma_OnTxComplete(huart);
    ManipulationUartDma_OnTxComplete(huart);
}
```

UART8 TX complete motor busy state'lerini etkilememeli.

---

## 12. Terminal parser entegrasyonu

### 12.1 Yeni command type

`terminal_parser.h` içine yeni command type eklenmeli:

```c
TCMD_ARM_RAW
```

`TerminalCommand_t` içine payload alanı eklenmeli:

```c
char armPayload[RAW_PAYLOAD_MAX];
```

Ayrı bir define de kullanılabilir:

```c
#define ARM_PAYLOAD_MAX RAW_PAYLOAD_MAX
```

### 12.2 Prefix parsing

Parser şu kuralı uygulamalı:

- Satır `arm` prefix'i ile başlıyorsa `TCMD_ARM_RAW` üret.
- Prefix sonrası boşluklardan sonra gelen kısım payload kabul edilir.
- Payload boşsa parser geçersiz veya usage error'a yönlendirilecek command üretir.
- Prefix case-insensitive olabilir.
- Payload mümkün olduğunca değiştirilmeden korunmalı.

Önemli not:

Mevcut parser satırı lowercase'e çeviriyorsa, `arm` parsing'i lowercase işleminden önce yapılmalı veya payload orijinal input'tan kopyalanmalı.

Bunun sebebi, manipulation F411 tarafında payload'ın ileride case-sensitive olabilme ihtimalidir.

---

## 13. Command handler entegrasyonu

`command_handler.c` içinde `TCMD_ARM_RAW` case'i eklenmeli.

Davranış:

1. Payload boş mu kontrol et.
2. Rover operating mode kontrolüne göre izin politikası uygula.
3. Uygunsa `ManipulationUartDma_SendRaw(cmd->armPayload)` çağır.
4. Queue dolu veya gönderim başlatılamadıysa kontrollü error log bas.

### 13.1 DISARM politikası

Bu roadmap'te manipulation komutlarının anlamı H7 tarafından bilinmeyecek. Bu yüzden iki seçenek var:

| Seçenek | Açıklama |
|---|---|
| A | DISARM içinde bütün `arm` forwarding bloklanır |
| B | DISARM içinde de `arm` forwarding serbest bırakılır |

Önerilen ilk güvenli yaklaşım:

- Başlangıçta DISARM içinde `arm` forwarding bloklansın.
- Manual / autonomous modda forwarding serbest olsun.
- Daha sonra manipulation F411 protokolü netleşirse payload-level safety policy eklenir.

Bu karar, H7'nin payload anlamını bilmeden robot kolu hareket ettirebilecek satırları forward etmesini engeller.

---

## 14. `app_config.h` güncellemesi

`app_config.h` içinde UART8 handle'ları extern olarak bulunmalı.

Eklenecekler:

```c
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_uart8_tx;
```

Böylece `manipulation_uart_dma.c` içinde doğrudan `huart8` kullanılabilir.

---

## 15. `app_main.c` entegrasyonu

`app_main.c` içine header eklenmeli:

```c
#include "manipulation_uart_dma.h"
```

`App_Init()` içinde çağrılacaklar:

```c
ManipulationUartDma_Init();
ManipulationUartDma_StartRx();
```

`App_Update()` içinde çağrılacak:

```c
ManipulationUartDma_Update();
```

Bu sayede:

- Error recovery ana döngüde yapılır.
- Error repeat timer ana döngüde yönetilir.
- TX queue warning logları ISR dışına taşınabilir.

---

## 16. GUI entegrasyon roadmap'i

GUI tarafında UART8, motor tablosuna eklenmemeli. Bunun yerine ayrı bir ARM UART status alanı eklenmeli.

### 16.1 Regex genişletme

Mevcut UART error regex'leri motor UART'larına göre yazılmış durumda. UART8 de tanınmalı.

Mantık:

- Motor UART hatası gelirse motor tablosundaki ilgili satır güncellensin.
- UART8 hatası gelirse ARM UART status alanı güncellensin.

### 16.2 ARM UART status kutusu

Önerilen alanlar:

| Alan | İçerik |
|---|---|
| Link | OK / Error / Unknown |
| Error | Aktif UART8 hata özeti |
| Last RX | Son ARM RX satırı |
| Last TX | Son ARM TX satırı |
| Dropped | Queue veya RX overflow sayısı |

### 16.3 GUI parse edilecek firmware satırları

GUI şu formatları yakalamalı:

```text
[ARM_RX] <line>
[ARM_TX] <payload>
[ERROR] UART8 UART error code: 0x00000004
[ERROR] UART8 error: FE - Framing error
[INFO] UART8 RX recovered after UART error
```

### 16.4 Motor tablosu korunmalı

Motor table mapping değişmeyecek:

| UART | Motor |
|---|---|
| USART2 | FL |
| UART4 | FR |
| UART7 | RL |
| UART5 | RR |

UART8 bu tabloya dahil edilmeyecek.

---

## 17. Stage planı

## Stage 0 — Baseline doğrulama

Amaç: UART8'in donanım seviyesinde projede gerçekten hazır olduğunu teyit etmek.

Kontrol listesi:

- `huart8` var mı?
- `MX_UART8_Init()` çağrılıyor mu?
- PE0 / PE1 UART8 alternate function olarak ayarlı mı?
- DMA2 Stream0 UART8 RX'e bağlı mı?
- DMA2 Stream1 UART8 TX'e bağlı mı?
- `DMA2_Stream0_IRQHandler()` var mı?
- `DMA2_Stream1_IRQHandler()` var mı?
- `UART8_IRQHandler()` var mı?

Beklenen sonuç:

- CubeMX/HAL init tarafına minimum dokunuş.
- Asıl iş uygulama katmanında yapılacak.

---

## Stage 1 — Manipulation UART module iskeleti

Amaç: UART8 için motorlardan bağımsız yeni modül oluşturmak.

Dosyalar:

- `Core/Inc/manipulation_uart_dma.h`
- `Core/Src/manipulation_uart_dma.c`

Yapılacaklar:

- Public API oluştur.
- Statik state struct oluştur.
- RX/TX buffer tanımla.
- Error state değişkenlerini tanımla.
- Dropped counter değişkenlerini tanımla.

Beklenen sonuç:

- Projede manipulation UART için tek sorumluluğa sahip bağımsız kaynak dosya oluşur.

---

## Stage 2 — RX DMA ReceiveToIdle entegrasyonu

Amaç: UART8 RX DMA'yı başlatmak ve gelen byte'ları satıra çevirmek.

Yapılacaklar:

- `ManipulationUartDma_StartRx()` implement edilir.
- `HAL_UARTEx_ReceiveToIdle_DMA()` kullanılır.
- Half-transfer interrupt disable edilir.
- `ManipulationUartDma_HandleRxEvent()` implement edilir.
- Gelen byte'lar line buffer'a eklenir.
- `\n` ile satır tamamlanınca log basılır.
- RX event sonrası DMA yeniden başlatılır.

Beklenen sonuç:

- Manipulation F411'in UART8'e bastığı satırlar H7 terminalinde görülebilir.

---

## Stage 3 — TX DMA queue entegrasyonu

Amaç: H7 terminalinden gelen payload'ları UART8'e güvenli şekilde basmak.

Yapılacaklar:

- `ManipulationUartDma_SendRaw()` implement edilir.
- Payload sonuna CRLF eklenir.
- TX idle ise DMA hemen başlatılır.
- TX busy ise payload queue'ya alınır.
- Queue doluysa dropped counter artırılır.
- `ManipulationUartDma_OnTxComplete()` implement edilir.
- TX complete sonrası sıradaki payload gönderilir.

Beklenen sonuç:

- Arka arkaya gelen `arm` satırları UART8 tarafında kaybolmadan sırayla gider.

---

## Stage 4 — Callback router entegrasyonu

Amaç: Mevcut HAL callback'lerini bozmadan UART8'i sisteme bağlamak.

Dosyalar:

- `motor_uart_dma.c`
- `motor_tx_dma.c`

Yapılacaklar:

- RX callback başında UART8 manipulation handler çağrılır.
- UART8 event işlendiyse motor RX logic'e geçilmez.
- Error callback başında UART8 error handler çağrılır.
- UART8 error işlendiyse motor error logic'e geçilmez.
- TX complete callback içinde manipulation TX complete çağrılır.
- Motor TX complete davranışı korunur.

Beklenen sonuç:

- UART8 callback'leri çalışır.
- Mevcut motor RX/TX/error davranışı değişmez.

---

## Stage 5 — UART8 error reporting

Amaç: UART8 hata durumlarını motor UART hata mantığına benzer şekilde raporlamak.

Yapılacaklar:

- Error code kaydet.
- Decoded error string üret.
- İlk hatada log bas.
- Hata devam ediyorsa belirli aralıkla log bas.
- RX recovery olunca recovery logu bas.
- RX DMA yeniden başlat.
- Error state clear şartlarını belirle.

Beklenen sonuç:

- UART8 kablo, baudrate, framing, noise, overrun gibi problemleri terminal ve GUI tarafından görülebilir.

---

## Stage 6 — Terminal parser entegrasyonu

Amaç: `arm` prefix'li satırları parser seviyesinde ayrı command type'a çevirmek.

Dosyalar:

- `terminal_parser.h`
- `terminal_parser.c`

Yapılacaklar:

- `TCMD_ARM_RAW` ekle.
- `TerminalCommand_t` içine `armPayload` ekle.
- Prefix parsing'i global lowercase işleminden önce yap.
- Prefix sonrasındaki payload'u orijinal haliyle koru.
- Payload boşsa handler'ın usage error vermesine izin ver.

Beklenen sonuç:

- Terminalden gelen `arm` satırları motor raw forwarding ile karışmaz.
- Payload UART8'e gönderilmeye hazır şekilde command handler'a ulaşır.

---

## Stage 7 — Command handler entegrasyonu

Amaç: Parser'dan gelen ARM command'i UART8 TX DMA modülüne yönlendirmek.

Dosya:

- `command_handler.c`

Yapılacaklar:

- Help text'e manipulation UART açıklaması ekle.
- `TCMD_ARM_RAW` case'i ekle.
- Payload boşluk kontrolü yap.
- Operating mode güvenlik kararını uygula.
- `ManipulationUartDma_SendRaw()` çağır.
- Başarısızlık durumunda kontrollü error log bas.

Beklenen sonuç:

- H7 terminalinden girilen `arm` prefix'li satırlar UART8'e forward edilir.

---

## Stage 8 — App lifecycle entegrasyonu

Amaç: Manipulation UART modülünü sistem başlangıcına ve ana döngüye bağlamak.

Dosya:

- `app_main.c`

Yapılacaklar:

- Header include edilir.
- `App_Init()` içinde init/start çağrıları eklenir.
- `App_Update()` içinde update çağrısı eklenir.

Beklenen sonuç:

- UART8 RX boot sonrası otomatik başlar.
- Error handling ve queue maintenance düzenli çalışır.

---

## Stage 9 — GUI ARM UART status entegrasyonu

Amaç: UART8 durumunu GUI'de motorlardan ayrı göstermek.

Dosya:

- `earendil.py`

Yapılacaklar:

- UART error regex'leri UART8'i tanıyacak şekilde genişletilir.
- UART8 için motor mapping yapılmaz.
- ARM UART status kutusu eklenir.
- `ARM_RX`, `ARM_TX`, UART8 error ve recovery satırları parse edilir.
- Error label güncellenir.
- Recovery gelince error temizlenir.

Beklenen sonuç:

- Kullanıcı UART8 manipulation linkinin hata durumunu GUI'de görebilir.
- Motor tablosu mevcut 4 motorla sınırlı kalır.

---

## Stage 10 — Donanım test sırası

Bu stage kullanıcı tarafından kart üzerinde doğrulanacak şekilde planlanmıştır.

### 10.1 Boot testi

Beklenen:

- Sistem açılır.
- UART8 RX DMA start edilir.
- Mevcut motor UART'ları etkilenmez.

### 10.2 UART8 TX testi

Uygulanacak örnek:

```text
arm forward 1 200
```

Beklenen:

- H7, prefix'ten sonraki payload'u UART8'e gönderir.
- Manipulation F411 tarafında payload görünür.

### 10.3 UART8 TX ikinci örnek testi

Uygulanacak örnek:

```text
arm set 1 stopmode brake
```

Beklenen:

- H7, prefix'ten sonraki payload'u UART8'e gönderir.
- Payload içeriği değiştirilmez.

### 10.4 UART8 RX testi

Beklenen:

- Manipulation F411'in UART8'e gönderdiği satır H7 terminalinde `[ARM_RX]` etiketiyle görünür.

### 10.5 UART8 error testi

Beklenen:

- UART8 hattında fiziksel veya format kaynaklı problem oluşursa H7 terminalinde UART8 error logları görünür.
- GUI'deki ARM UART status alanı error durumuna geçer.

### 10.6 UART8 recovery testi

Beklenen:

- Hata sebebi giderildiğinde RX tekrar başlar.
- Recovery logu görünür.
- GUI error alanı temizlenir.

### 10.7 Regression testi

Beklenen:

- FL / FR / RL / RR motor UART telemetry bozulmaz.
- Motor UART error table davranışı bozulmaz.
- Terminal komutları bozulmaz.
- IMU / MAG komutları bozulmaz.
- Operating mode göstergesi bozulmaz.

---

## 18. Kabul kriterleri

Bu entegrasyon tamamlanmış sayılacaksa aşağıdaki maddeler sağlanmalı:

- UART8 RX DMA boot sonrası başlıyor.
- UART8 TX DMA queue ile çalışıyor.
- `arm` prefix'i parser tarafından ayrı command olarak algılanıyor.
- Payload korunarak UART8'e gönderiliyor.
- UART8 RX satırları terminalde ayrı etiketle görünüyor.
- UART8 hata kodları terminalde görünüyor.
- UART8 recovery logu çalışıyor.
- GUI UART8 hata durumunu motorlardan ayrı gösteriyor.
- Motor UART callback'leri bozulmuyor.
- Motor tablosuna UART8 eklenmiyor.
- Manipulation UART kodu motor tuning / motor dispatcher katmanlarına karışmıyor.

---

## 19. Riskler ve dikkat noktaları

| Risk | Açıklama | Önlem |
|---|---|---|
| Duplicate HAL callback | Yeni modülde callback define edilirse çakışma olur | Mevcut callback'leri router olarak kullan |
| Payload lowercase bozulması | Parser payload'u küçük harfe çevirebilir | Prefix'i ayrı parse et, payload'u orijinalden kopyala |
| DMA cache problemi | H7 D-cache ile DMA buffer tutarsızlığı olabilir | `.dma_buffer`, 32-byte alignment ve mevcut proje yaklaşımını koru |
| TX busy kaybı | Arka arkaya satırlar HAL_BUSY ile düşebilir | TX queue kullan |
| UART8 motor sanılması | GUI veya firmware motor tablosuna yanlış bağlanabilir | UART8'i ayrı ARM status olarak tut |
| Error spam | Sürekli log terminali kilitleyebilir | Throttle uygula |
| DISARM güvenliği | H7 payload anlamını bilmiyor | İlk aşamada mode policy net ve basit tutulmalı |

---

## 20. Agent uygulama raporu formatı

Agent işi bitirdiğinde şu formatta rapor vermeli:

```text
Manipulation UART8 DMA Implementation Report

1. Files changed
- ...

2. UART8 hardware status
- RX pin: PE0
- TX pin: PE1
- RX DMA: DMA2 Stream0
- TX DMA: DMA2 Stream1

3. New module
- manipulation_uart_dma.h/.c added: yes/no

4. Terminal parser
- TCMD_ARM_RAW added: yes/no
- arm payload preserved: yes/no

5. Command handler
- arm prefix forwarding implemented: yes/no

6. Callback routing
- RX callback routed to UART8: yes/no
- Error callback routed to UART8: yes/no
- TX complete callback routed to UART8: yes/no

7. Error reporting
- UART8 error code logging: yes/no
- UART8 decoded error logging: yes/no
- UART8 recovery logging: yes/no

8. GUI
- UART8 error parsing added: yes/no
- ARM UART status separate from motor table: yes/no

9. Existing behavior preserved
- Motor UART behavior changed: yes/no
- Motor telemetry parser changed: yes/no
- IMU/MAG behavior changed: yes/no

10. Notes / limitations
- ...
```

---

## 21. Kısa özet

Bu roadmap'in ana fikri şudur:

- UART8 zaten donanım seviyesinde hazır.
- Eksik olan şey bağımsız manipulation UART uygulama katmanı.
- `arm` prefix'i H7 terminalinde yakalanacak.
- Prefix sonrası payload UART8'e DMA TX ile gönderilecek.
- UART8 RX DMA ile gelen satırlar loglanacak.
- UART8 hataları motor UART error mantığına benzer ama ayrı tutulacak.
- GUI'de ARM UART status motor tablosundan bağımsız gösterilecek.

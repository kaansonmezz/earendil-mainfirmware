# Earendil TCP Haberleşme Dönüşüm Roadmap'i

## 1. Dokümanın Amacı

Bu doküman, mevcut PySide6 tabanlı Earendil GUI uygulamasının STM32H723 ile doğrudan seri port üzerinden haberleşen yapısından çıkarılıp aşağıdaki mimariye taşınması için ayrıntılı uygulama planını tanımlar:

```text
Kontrol Bilgisayarı
PySide6 GUI — TCP Client
        │
        │ Private outdoor Wi-Fi ağı
        │ TCP
        ▼
Raspberry Pi 5
TCP Server ↔ Serial Bridge
        │
        │ USB Serial / UART
        ▼
STM32H723
```

Bu dönüşümün temel hedefleri:

- GUI yalnızca kontrol bilgisayarında çalışacak.
- Raspberry Pi yalnızca TCP–Serial bridge görevi görecek.
- STM32H723, Raspberry Pi'ye USB Serial veya UART üzerinden bağlı kalacak.
- GUI içerisinde serial port seçimi bulunmayacak.
- GUI içerisinde baud rate seçimi bulunmayacak.
- GUI içerisinde serial monitor veya H7 terminali bulunmayacak.
- H7 firmware komut formatı mümkün olduğunca değişmeden korunacak.
- Motor, IMU, manipülasyon kolu ve sondaj modüllerinin mevcut parser ve komut mantığı korunacak.
- TCP bağlantısı koptuğunda rover güvenli şekilde duracak.
- Raspberry Pi açıldığında bridge uygulaması otomatik başlayacak.
- Aynı anda yalnızca bir kontrol istemcisi bağlanabilecek.

---

# 2. Mevcut Sistem Özeti

Mevcut GUI doğrudan `pyserial` kullanarak H7'nin seri portunu açmaktadır.

Genel veri akışı:

```text
GUI
 ├── serial port seçimi
 ├── baud rate seçimi
 ├── pyserial bağlantısı
 ├── SerialReaderThread
 ├── _send_cmd()
 └── _on_rx_line()
        ├── motor telemetry parser
        ├── UART error parser
        ├── operating mode parser
        ├── IMU parser
        ├── motor tuning parser
        ├── manipulation parser
        └── drill parser
```

Mevcut sistemde önemli avantaj, komut gönderme ve veri alma yollarının büyük ölçüde merkezî olmasıdır:

- Tüm komutlar büyük ölçüde `_send_cmd()` üzerinden gönderilmektedir.
- H7'den gelen satırlar `_on_rx_line()` üzerinden parser'lara dağıtılmaktadır.

Bu nedenle TCP dönüşümünde motor, IMU, manipulation ve drill mantığını baştan yazmak gerekmemelidir. Temel olarak taşıma katmanı değiştirilecektir.

---

# 3. Nihai Sistem Mimarisi

## 3.1 Kontrol Bilgisayarı

Kontrol bilgisayarında yalnızca GUI çalışacaktır.

Görevleri:

- Raspberry Pi IP adresine TCP bağlantısı kurmak.
- Kullanıcı komutlarını TCP üzerinden göndermek.
- Raspberry Pi üzerinden gelen H7 telemetrisini almak.
- Gelen satırları mevcut parser zincirine iletmek.
- Bağlantı durumunu kullanıcıya göstermek.
- Bağlantı kesildiğinde tüm kontrol state'lerini temizlemek.
- Eski hareket komutlarını yeniden bağlantıda tekrar uygulamamak.

Kontrol bilgisayarında bulunmayacak bileşenler:

- `pyserial`
- serial port listesi
- baud rate seçimi
- `/dev/ttyACM*`, `/dev/ttyUSB*` veya `COM*` seçimi
- H7 serial monitor
- manuel H7 terminal input'u

## 3.2 Raspberry Pi 5

Raspberry Pi bridge görevi görecektir.

Görevleri:

- H7'nin serial portunu açmak.
- Belirli bir TCP portunu dinlemek.
- PC'den gelen TCP byte'larını H7 serial hattına aktarmak.
- H7'den gelen serial byte'larını PC'ye göndermek.
- Aynı anda yalnızca bir kontrol istemcisi kabul etmek.
- TCP bağlantısı kopunca H7'ye `stop\r\n` göndermek.
- Serial bağlantı koparsa tekrar bağlanmayı denemek.
- Bridge çökerse `systemd` tarafından yeniden başlatılmak.
- Bağlantı ve hata loglarını `journalctl` üzerinden sunmak.

## 3.3 STM32H723

STM32H723 mümkün olduğunca değiştirilmeden kalacaktır.

Korunacak özellikler:

- Mevcut terminal komut formatı
- `\r\n` satır sonlandırması
- Motor komutları
- IMU komutları
- Manipulation komutları
- Drill komutları
- Telemetri satır formatları
- H7 logger formatı

H7 tarafında önerilen ek güvenlik:

- Heartbeat veya command timeout
- Belirli süre geçerli komut alınmazsa `stop`
- Bridge veya Raspberry Pi tamamen kaybolsa bile bağımsız fail-safe

---

# 4. Temel Tasarım Kararları

| Konu | Karar | Gerekçe |
|---|---|---|
| Ana veri taşıma protokolü | TCP | Çift yönlü byte stream, kolay reconnect, düşük uygulama karmaşıklığı |
| Raspberry Pi yönetimi | SSH | Servis yönetimi, log okuma, güncelleme ve bakım için |
| GUI haberleşme rolü | TCP client | Kontrol bilgisayarı Raspberry Pi'ye bağlanacak |
| Raspberry Pi haberleşme rolü | TCP server + Serial bridge | H7 seri portunun tek sahibi Raspberry Pi olacak |
| H7 protokolü | Korunacak | Firmware değişikliğini ve entegrasyon riskini azaltmak |
| TCP framing | `\r\n` ile satır bazlı | Mevcut H7 terminal protokolüyle uyumlu |
| Aktif istemci | 1 | Birden fazla bilgisayarın aynı rover'ı kontrol etmesini önlemek |
| Serial cihaz yolu | `/dev/serial/by-id/...` | `/dev/ttyACM0` değişimlerine karşı kararlı |
| TCP port | Örnek: `5000` | Private rover ağı içinde uygulamaya özel port |
| GUI serial monitor | Kaldırılacak | Operasyon GUI'sini sadeleştirmek ve performansı korumak |
| GUI debug log | Sadece bağlantı ve GUI olayları | Ham telemetriyi kullanıcı arayüzüne doldurmamak |
| Bridge debug log | `journalctl` veya rotating log | Saha hata ayıklaması için |
| Disconnect davranışı | Bridge H7'ye `stop` yollar | Hızlı güvenli duruş |
| Asıl fail-safe | H7 watchdog | Pi veya ağ tamamen kaybolduğunda dahi güvenlik |
| Auto reconnect | Kontrollü | Eski hareket komutları tekrar uygulanmamalı |

---

# 5. Ayrıntılı Roadmap

| Stage | Amaç | PC GUI Tarafında Yapılacaklar | Raspberry Pi Tarafında Yapılacaklar | H7 Tarafında Yapılacaklar | Riskler | Kabul Kriteri |
|---:|---|---|---|---|---|---|
| 0 | Çalışan sürümü korumak | Mevcut serial çalışan GUI ayrı Git branch veya tag olarak saklanacak. | Değişiklik yok. | Mevcut çalışan firmware commit'i sabitlenecek. | Geri dönüş noktası olmadan büyük refactor yapılması | Eski serial sürüm istenildiğinde tekrar çalıştırılabiliyor. |
| 1 | Mevcut komut ve telemetri envanterini çıkarmak | Motor, IMU, arm, drill ve tuning komut yolları listelenecek. | Bridge'in taşıması gereken veri türleri belirlenecek. | H7 çıktıları ve komut sonlandırmaları doğrulanacak. | Bazı komutların `_send_cmd()` dışında gönderiliyor olması | Tüm TX ve RX yolları belgelenmiş. |
| 2 | TCP–Serial protokol sözleşmesini sabitlemek | GUI `komut\r\n` göndermeye devam edecek. | Bridge veriyi yorumlamadan taşıyacak. | Mevcut parser korunacak. | Bridge'in komut parse etmeye başlamasıyla çift mantık oluşması | H7 protokolünde değişiklik gerekmiyor. |
| 3 | Bridge prototipini geliştirmek | Basit test client ile bağlantı kurulacak. | TCP server, serial open, TCP→Serial ve Serial→TCP worker'ları yazılacak. | H7 Raspberry Pi'ye bağlı olacak. | Blocking I/O ve deadlock | Test komutu H7'ye ulaşıyor ve cevap geri geliyor. |
| 4 | TCP stream buffer davranışını doğrulamak | TCP verileri `\n` üzerinden ayrıştırılacak. | Byte stream olduğu için paket sınırına güvenilmeyecek. | Satır sonlandırmaları korunacak. | Bir komutun birkaç `recv()` çağrısına bölünmesi | Parçalı ve birleşik paketler doğru işleniyor. |
| 5 | Kararlı serial cihaz yolu kullanmak | Değişiklik yok. | `/dev/serial/by-id/...` yolu bulunacak ve config'e yazılacak. | Değişiklik yok. | `/dev/ttyACM0` numarasının değişmesi | Pi reboot sonrası doğru cihaz tekrar açılıyor. |
| 6 | Serial reconnect mekanizması | Değişiklik yok. | Serial cihaz yoksa belirli aralıklarla yeniden açma denenecek. | USB yeniden bağlandığında haberleşmeye dönülecek. | Bridge'in serial hata sonrası tamamen kapanması | H7 kablosu çıkarılıp takıldığında bridge toparlanıyor. |
| 7 | Tek istemci politikası | GUI normal client davranacak. | İlk client kabul edilecek, ikinci client reddedilecek. | Değişiklik yok. | İki GUI'nin aynı anda komut göndermesi | Aynı anda sadece bir kontrol bağlantısı aktif. |
| 8 | Disconnect-stop güvenliği | GUI disconnect durumunda tüm state'leri temizleyecek. | Client kopunca `stop\r\n` serial hatta yazılacak. | Stop komutu uygulanacak. | Socket kopmasının geç algılanması | GUI kapanınca rover stop alıyor. |
| 9 | H7 watchdog güvenliği | GUI heartbeat/keepalive tasarımına uygun çalışacak. | Bridge heartbeat üretmeyecek veya güvenliği maskelemeyecek. | Command/heartbeat timeout uygulanacak. | Bridge'in sahte heartbeat ile H7'yi kandırması | Pi tamamen kapanınca H7 rover'ı durduruyor. |
| 10 | Bridge loglama | GUI bridge loglarını göstermeyecek. | Client connect, disconnect, serial error, reconnect ve stop olayları loglanacak. | Değişiklik yok. | Aşırı log üretimi | `journalctl` ile anlamlı loglar görülebiliyor. |
| 11 | Bridge systemd servisi | Değişiklik yok. | `earendil-bridge.service` oluşturulacak. `Restart=always` kullanılacak. | Değişiklik yok. | Pi açılışında ağ veya serial hazır olmayabilir | Pi açıldığında bridge otomatik başlıyor. |
| 12 | GUI TCP katmanını eklemek | `QTcpSocket` veya eşdeğer Qt network yapısı eklenecek. | TCP bağlantısını kabul edecek. | Değişiklik yok. | Thread ve Qt event loop uyumsuzluğu | GUI bağlanıp veri alabiliyor. |
| 13 | GUI TCP RX buffer eklemek | `readyRead()` ile byte'lar buffer'a eklenecek, satırlar ayrılacak. | Ham veri gönderecek. | Telemetri aynı kalacak. | Eksik satırların parser'a verilmesi | `_on_rx_line()` yalnızca tamamlanmış satır alıyor. |
| 14 | `_send_cmd()` fonksiyonunu TCP'ye taşımak | İç arayüz korunacak, `self.ser.write()` yerine socket write kullanılacak. | Gelen byte'ları serial'a yazacak. | Komutları aynı şekilde alacak. | Komut sonlandırmasının değişmesi | Tüm mevcut GUI komutları TCP üzerinden çalışıyor. |
| 15 | Serial bağımlılıklarını kaldırmak | `pyserial`, `SerialReaderThread`, `self.ser`, port tarama ve serial exception kodları kaldırılacak. | `pyserial` yalnızca bridge'de kalacak. | Değişiklik yok. | Gizli serial referanslarının kalması | PC'de `pyserial` olmadan GUI açılıyor. |
| 16 | Network Connection paneli | IP, TCP port, Connect/Disconnect ve durum etiketi eklenecek. | Statik IP ve port kullanılacak. | Değişiklik yok. | Kullanıcıya fazla düşük seviye ayar göstermek | GUI'de serial port veya baud rate alanı kalmıyor. |
| 17 | Serial monitorü kaldırmak | H7 Console, TX/RX terminal logları, manuel input, Send ve Clear kaldırılacak. | Debug logları bridge tarafında tutulacak. | Değişiklik yok. | Debug kabiliyetinin tamamen kaybolması | GUI'de ham H7 terminali bulunmuyor. |
| 18 | GUI Console'u sadeleştirmek | Sadece connect, disconnect, error ve GUI olayları gösterilecek. | Bridge kendi logunu tutacak. | Değişiklik yok. | Ham telemetri nedeniyle GUI'nin yavaşlaması | GUI Console telemetriyle dolmuyor. |
| 19 | Bağlantı state machine | `DISCONNECTED`, `CONNECTING`, `CONNECTED`, `ERROR` tanımlanacak. | Server client state yönetecek. | Değişiklik yok. | Farklı modüllerin farklı bağlantı durumu kullanması | Tek merkezî bağlantı durumu var. |
| 20 | Serial kontrolleri TCP'ye çevirmek | `self.ser.is_open` kontrolleri `_tcp_is_connected()` ile değiştirilecek. | Değişiklik yok. | Değişiklik yok. | Tuning veya polling içinde eski kontrol kalması | Kod içinde `self.ser` referansı kalmıyor. |
| 21 | Disconnect cleanup | Tuş state'leri, repeat timer, arc timer, tuning queue ve polling timer'ları temizlenecek. | Disconnect-stop uygulanacak. | Watchdog destekleyecek. | Reconnect sonrası eski hareketin devam etmesi | Reconnect sonrası araç kendiliğinden hareket etmiyor. |
| 22 | TCP socket ayarları | Bağlantı hata yönetimi uygulanacak. | `TCP_NODELAY`, `SO_KEEPALIVE` ve uygun timeout ayarları uygulanacak. | Değişiklik yok. | Keepalive'ın güvenlik timeout'u sanılması | Düşük gecikme ve doğru disconnect algılama sağlanıyor. |
| 23 | Ağ erişimini sınırlamak | Varsayılan Pi IP ve port ayarı saklanabilir. | Bridge rover ağı IP'sinde dinleyecek, firewall yalnızca kontrol PC'sine izin verecek. | Değişiklik yok. | Portun tüm ağ arayüzlerinde açık olması | Yetkisiz cihazlar bridge portuna erişemiyor. |
| 24 | Fonksiyonel entegrasyon | Motor, IMU, arm, drill ve tuning tek tek test edilecek. | Bridge trafik logları izlenecek. | Komut cevapları doğrulanacak. | Bir modülün farklı komut yolu kullanması | Serial sürümde çalışan fonksiyonlar TCP'de de çalışıyor. |
| 25 | Hata senaryosu testleri | GUI crash, Wi-Fi loss, reconnect testleri yapılacak. | Bridge crash, Pi reboot, serial disconnect testleri yapılacak. | Watchdog test edilecek. | Sadece normal durumda test yapılması | Tüm hata durumlarında güvenli duruş sağlanıyor. |
| 26 | Uzun süreli test | GUI CPU, RAM ve parser davranışı izlenecek. | Bridge 30–60 dakika yoğun telemetriyle çalıştırılacak. | Sürekli telemetri üretilecek. | Buffer veya log nedeniyle memory growth | Sistem uzun süre kararlı çalışıyor. |
| 27 | Saha kabul testi | Outdoor access point üzerinden gerçek mesafe testi yapılacak. | Ağ bağlantısı ve reconnect izlenecek. | Motor ve güvenlik davranışı doğrulanacak. | Laboratuvar sonucu ile saha sonucu farkı | Gerçek rover ağı üzerinde kabul tamamlanıyor. |

---

# 6. Raspberry Pi Bridge Tasarımı

## 6.1 Önerilen Dosya Yapısı

```text
/home/pi/earendil-bridge/
├── tcp_uart_bridge.py
├── bridge_config.json
├── requirements.txt
├── logs/
└── README.md
```

Systemd servis dosyası:

```text
/etc/systemd/system/earendil-bridge.service
```

## 6.2 Önerilen Konfigürasyon

```json
{
  "listen_host": "192.168.50.20",
  "listen_port": 5000,
  "serial_device": "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_...",
  "baud_rate": 115200,
  "serial_timeout": 0.05,
  "serial_reconnect_interval_ms": 1000,
  "tcp_client_limit": 1,
  "send_stop_on_disconnect": true,
  "stop_command": "stop\r\n",
  "tcp_nodelay": true,
  "tcp_keepalive": true
}
```

## 6.3 Bridge Modülleri

| Modül | Görevi |
|---|---|
| Configuration Loader | JSON veya sabit config değerlerini okumak |
| Serial Manager | H7 portunu açmak, hata sonrası kapatmak ve yeniden açmak |
| TCP Listener | Belirlenen IP ve portta client beklemek |
| Client Ownership Manager | Tek istemci politikasını uygulamak |
| TCP RX Worker | PC'den gelen byte'ları serial'a yazmak |
| Serial RX Worker | H7'den gelen byte'ları TCP client'a yazmak |
| Disconnect Handler | Client kopunca stop göndermek ve buffer temizlemek |
| Shutdown Handler | Servis kapanırken socket ve serial portu güvenli kapatmak |
| Logger | Connect, disconnect, error, reconnect ve stop olaylarını kaydetmek |
| Health State | `TCP_CONNECTED`, `SERIAL_CONNECTED`, `BRIDGE_READY` durumlarını takip etmek |

## 6.4 Veri Akışı

### PC'den H7'ye

```text
GUI kontrolü
    ↓
_send_cmd("f100")
    ↓
TCP socket write: b"f100\r\n"
    ↓
Raspberry Pi TCP recv
    ↓
serial.write(b"f100\r\n")
    ↓
STM32H723 command parser
```

### H7'den PC'ye

```text
STM32H723 telemetry
    ↓
Serial RX
    ↓
Raspberry Pi serial read
    ↓
TCP sendall
    ↓
GUI readyRead
    ↓
TCP RX buffer
    ↓
_on_rx_line()
    ↓
Motor / IMU / Arm / Drill parser
```

## 6.5 Bridge'in Yapmaması Gerekenler

Bridge aşağıdaki işleri yapmamalıdır:

- `f100`, `stop`, `FL cfg` gibi komutları parse etmek
- Motor komutlarının anlamını bilmek
- Telemetri değerlerini tabloya çevirmek
- IMU veya motor parser çalıştırmak
- H7 protokolüne yeni prefix eklemek
- Satırların içeriğini değiştirmek
- GUI yerine otomatik sürüş komutu üretmek
- Eski TCP komutlarını reconnect sonrası tekrar göndermek
- PC bağlantısı yokken sahte heartbeat üretmek

Bridge'in görevi yalnızca güvenli ve çift yönlü veri taşımaktır.

---

# 7. TCP Stream Yönetimi

TCP paket tabanlı değil, byte-stream tabanlıdır.

Aşağıdaki durumlar normaldir:

```text
Gönderilen:
f100\r\n

Alınan recv parçaları:
"f1"
"00\r"
"\n"
```

veya:

```text
Gönderilen:
f100\r\nstop\r\n

Tek recv sonucu:
"f100\r\nstop\r\n"
```

Bu nedenle GUI:

1. Gelen byte'ları buffer'a eklemeli.
2. Buffer içinde `\n` aramalı.
3. Tamamlanan satırı ayırmalı.
4. `\r` karakterini temizlemeli.
5. UTF-8 decode uygulamalı.
6. Tam satırı `_on_rx_line()` fonksiyonuna göndermeli.
7. Eksik kalan kısmı buffer'da tutmalı.

Önerilen mantık:

```python
self._tcp_rx_buffer.extend(bytes_received)

while b"\n" in self._tcp_rx_buffer:
    raw_line, _, remaining = self._tcp_rx_buffer.partition(b"\n")
    self._tcp_rx_buffer = bytearray(remaining)

    line = raw_line.decode("utf-8", errors="replace").rstrip("\r")
    if line:
        self._on_rx_line(line)
```

Buffer için üst sınır eklenmelidir:

```text
Önerilen maksimum: 64 KiB
```

Eğer `\n` gelmeden buffer bu sınırı aşarsa:

- buffer temizlenmeli
- GUI error log yazmalı
- bağlantı gerekirse yeniden kurulmalı

---

# 8. PC GUI Kod Değişiklik Planı

## 8.1 Kaldırılacak Yapılar

| Mevcut Yapı | İşlem |
|---|---|
| `import serial` | Tamamen kaldırılacak |
| `import serial.tools.list_ports` | Tamamen kaldırılacak |
| `SerialReaderThread` | Tamamen kaldırılacak |
| `self.ser` | Tamamen kaldırılacak |
| `_refresh_ports()` | Tamamen kaldırılacak |
| `_close_serial()` | TCP karşılığıyla değiştirilecek |
| Serial port ComboBox | Arayüzden kaldırılacak |
| Baud rate ComboBox | Arayüzden kaldırılacak |
| Refresh Ports butonu | Arayüzden kaldırılacak |
| H7 Console | Arayüzden kaldırılacak |
| H7 manual input | Arayüzden kaldırılacak |
| TX-H7/RX-H7 logları | Arayüzden kaldırılacak |
| `SerialException` handling | TCP socket error handling ile değiştirilecek |
| `self.ser.is_open` kontrolleri | `_tcp_is_connected()` ile değiştirilecek |

## 8.2 Eklenecek Yapılar

| Yeni Yapı | Görevi |
|---|---|
| `QTcpSocket` | Raspberry Pi'ye bağlanmak |
| IP alanı | Raspberry Pi statik IP adresi |
| TCP port alanı | Bridge portu |
| Connect/Disconnect butonu | TCP bağlantı kontrolü |
| Connection status badge | Bağlantı durumunu göstermek |
| `_tcp_rx_buffer` | Gelen byte'ları satır bazında biriktirmek |
| `_tcp_connect()` | TCP bağlantısını başlatmak |
| `_tcp_disconnect()` | Socket'i kapatmak |
| `_tcp_ready_read()` | Gelen byte'ları okumak |
| `_tcp_is_connected()` | Merkezî bağlantı durumu kontrolü |
| `_tcp_error()` | Socket hata yönetimi |
| `_handle_unexpected_disconnect()` | State temizliği ve kullanıcı bilgilendirmesi |

## 8.3 Korunacak Fonksiyonlar

Aşağıdaki işlevler mümkün olduğunca korunmalıdır:

- `_send_cmd()`
- `_on_rx_line()`
- motor telemetry parser
- UART error parser
- operating mode parser
- IMU parser
- motor tuning parser
- manipulation parser
- drill parser
- klavye kontrol mantığı
- stop/brake/identify komut yapısı
- polling mantığı

`_send_cmd()` fonksiyonunun dış arayüzü korunmalıdır.

Eski:

```python
def _send_cmd(self, cmd: str):
    self.ser.write((cmd + "\r\n").encode("utf-8"))
```

Yeni:

```python
def _send_cmd(self, cmd: str):
    if not self._tcp_is_connected():
        self._log_warn("Not sent — TCP disconnected")
        return False

    payload = (cmd + "\r\n").encode("utf-8")
    written = self.tcp_socket.write(payload)
    return written == len(payload)
```

## 8.4 Network Connection Paneli

Önerilen arayüz:

```text
Network Connection
┌─────────────────────────────────────────────┐
│ Raspberry Pi IP : [192.168.50.20        ]  │
│ TCP Port        : [5000                 ]  │
│ [Connect / Disconnect]   Status: CONNECTED │
└─────────────────────────────────────────────┘
```

Bağlantı durumları:

| Durum | Gösterim | Davranış |
|---|---|---|
| `DISCONNECTED` | Gri veya kırmızı | IP ve port düzenlenebilir |
| `CONNECTING` | Sarı | Connect butonu geçici kilitli |
| `CONNECTED` | Yeşil | IP ve port kilitli |
| `ERROR` | Kırmızı | Hata mesajı gösterilir |

Bağlantı kurulunca:

- IP alanı disabled
- port alanı disabled
- buton metni `Disconnect`
- durum `CONNECTED`

Bağlantı kesilince:

- IP alanı enabled
- port alanı enabled
- buton metni `Connect`
- durum `DISCONNECTED`

---

# 9. Serial Monitorün Kaldırılması

GUI'den tamamen kaldırılacak bileşenler:

- H7 Console
- H7 terminal text box
- manuel komut input'u
- Send butonu
- Clear H7 Console butonu
- TX-H7 log satırları
- RX-H7 log satırları
- serial monitor başlığı
- serial port bağlantı ayrıntıları

Korunabilecek GUI Console:

```text
[GUI] Connecting to 192.168.50.20:5000
[GUI] Connected
[GUI-WARN] TCP connection lost
[GUI-ERROR] Connection refused
[GUI] Reconnected
```

GUI Console'a yazılmaması gerekenler:

```text
[RX-H7] [INFO] [TEL][FL] RPM:...
[RX-H7] MPU_IMU,...
[RX-H7] ARM_MOTORS,...
```

Ham H7 verileri görünmeden doğrudan `_on_rx_line()` parser zincirine gitmelidir.

Debug ihtiyacı için öneri:

- PC tarafında opsiyonel rotating file log
- Raspberry Pi tarafında `journalctl`
- Debug mode açıkken raw TCP log
- Normal operasyon modunda ham telemetri loglamama

---

# 10. Bağlantı State Machine

Önerilen durum makinesi:

```text
DISCONNECTED
      │ Connect
      ▼
CONNECTING
      │ connected()
      ▼
CONNECTED
      │ disconnect / socket error
      ▼
DISCONNECTED veya ERROR
```

## 10.1 DISCONNECTED

- Komut gönderilmez.
- Polling timer'ları çalışmaz.
- Hareket state'leri temizdir.
- IP ve port düzenlenebilir.

## 10.2 CONNECTING

- Yeni connect isteği engellenir.
- UI sarı bağlantı durumu gösterir.
- Timeout uygulanabilir.
- Başarısız olursa `ERROR` veya `DISCONNECTED`.

## 10.3 CONNECTED

- `_send_cmd()` aktif.
- Motor/IMU/arm/drill komutları gönderilebilir.
- Polling başlatılabilir.
- IP ve port alanları kilitlidir.

## 10.4 ERROR

- Socket hata açıklaması GUI Console'a yazılır.
- Tüm hareket state'leri temizlenir.
- Polling durdurulur.
- Eski komut kuyruğu temizlenir.
- Kullanıcı tekrar bağlanabilir.

---

# 11. Disconnect ve Reconnect Güvenliği

TCP bağlantısı kesildiğinde GUI aşağıdaki işlemleri yapmalıdır:

1. Basılı klavye tuşlarını temizle.
2. W/A/S/D repeat timer'larını durdur.
3. Arc-turn timer'larını durdur.
4. Tuning send queue'yu iptal et.
5. Config read bekleme durumlarını temizle.
6. Arm polling timer'larını durdur.
7. Drill polling timer'larını durdur.
8. IMU polling/stream UI state'ini bağlantısız duruma getir.
9. Bağlantı status'unu `DISCONNECTED` yap.
10. IP ve port alanlarını tekrar aç.
11. Eski hareket komutunu saklama.
12. Reconnect sonrası otomatik hareket başlatma.
13. H7 operating mode için varsayım yapma.
14. H7'den yeni durum telemetrisi bekle.

Raspberry Pi bridge disconnect olduğunda:

1. Client socket kapanışını algıla.
2. H7'ye `stop\r\n` gönder.
3. Gerekirse serial output buffer flush et.
4. TCP client'a ait kalan buffer'ı temizle.
5. Client ownership'i bırak.
6. Yeni client beklemeye dön.
7. Eski komutu yeni client'a taşımama.

---

# 12. Fail-Safe Katmanları

| Katman | Tetikleyici | Eylem | Amaç |
|---|---|---|---|
| GUI | TCP disconnect/error | Timer ve input state'lerini temizler | Eski komutların devamını önlemek |
| Raspberry Pi Bridge | Client socket kapanması | `stop\r\n` gönderir | Hızlı güvenli duruş |
| STM32H723 | Heartbeat/command timeout | Tüm motorlara stop/brake | Pi veya ağ tamamen kaybolduğunda güvenlik |
| Motor sürücüleri | H7 heartbeat veya command timeout | Güvenli motor state'i | Son güvenlik katmanı |
| Güç sistemi | Acil stop veya donanımsal hata | Gücü veya enable hattını keser | Yazılımdan bağımsız fiziksel güvenlik |

Önemli:

Raspberry Pi'nin `stop` göndermesi tek başına yeterli değildir.

Aşağıdaki durumda bridge stop gönderemez:

- Raspberry Pi tamamen kapanırsa
- Raspberry Pi güç kaybederse
- Serial kablo koparsa
- Bridge process'i kernel seviyesinde donarsa
- H7 RX hattı çalışmazsa

Bu nedenle H7 watchdog zorunlu güvenlik katmanı olarak ele alınmalıdır.

---

# 13. TCP Socket Ayarları

## 13.1 TCP_NODELAY

Küçük komutlarda Nagle algoritmasının oluşturabileceği beklemeyi azaltır.

Özellikle şu tip komutlarda faydalıdır:

```text
f100\r\n
stop\r\n
x\r\n
mode manual\r\n
```

## 13.2 SO_KEEPALIVE

Ölü bağlantının işletim sistemi seviyesinde tespitine yardımcı olur.

Ancak:

- Keepalive süreleri genellikle rover güvenliği için uzundur.
- Güvenlik timeout'u olarak kullanılmamalıdır.
- Asıl güvenlik H7 heartbeat timeout ile sağlanmalıdır.

## 13.3 Socket Timeout

Bridge tarafında blocking işlemler sonsuza kadar beklememelidir.

Öneri:

- kısa socket timeout
- kontrollü loop
- shutdown flag
- serial reconnect interval

## 13.4 Write Queue

GUI tarafında normal kontrol komutları için büyük bir yazma kuyruğu tutulmamalıdır.

Özellikle hareket komutları:

- eski komut olarak kuyrukta beklememeli
- reconnect sonrası gönderilmemeli
- bağlantı kopunca iptal edilmeli

---

# 14. Ağ Planı

Örnek statik ağ yapısı:

| Cihaz | IP |
|---|---|
| Outdoor Access Point | `192.168.50.1` |
| Kontrol PC | `192.168.50.10` |
| Raspberry Pi | `192.168.50.20` |
| TCP Bridge | `192.168.50.20:5000` |

Subnet:

```text
255.255.255.0
```

## 14.1 Raspberry Pi Firewall

Örnek politika:

```text
192.168.50.10 → TCP 5000 izin ver
Diğer cihazlar → TCP 5000 reddet
SSH → yalnızca bakım cihazlarına izin ver
```

## 14.2 Access Point Ayarları

- WPA2-AES veya WPA3
- WPS kapalı
- varsayılan yönetici şifresi değiştirilmiş
- internet port forwarding kapalı
- mümkünse ayrı rover SSID
- mümkünse ayrı VLAN
- kontrol ağı üzerinde gereksiz cihaz bulunmamalı
- client isolation kullanılacaksa PC ve Pi arasındaki iletişimi engellemediği doğrulanmalı

## 14.3 Bridge Listen Address

Tercih:

```text
192.168.50.20:5000
```

Gereksizse kullanılmamalı:

```text
0.0.0.0:5000
```

---

# 15. systemd Servis Planı

Önerilen servis:

```ini
[Unit]
Description=Earendil TCP UART Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/earendil-bridge
ExecStart=/usr/bin/python3 /home/pi/earendil-bridge/tcp_uart_bridge.py
Restart=always
RestartSec=1
TimeoutStopSec=3

[Install]
WantedBy=multi-user.target
```

Yönetim komutları:

```bash
sudo systemctl daemon-reload
sudo systemctl enable earendil-bridge
sudo systemctl start earendil-bridge
sudo systemctl restart earendil-bridge
sudo systemctl status earendil-bridge
journalctl -u earendil-bridge
journalctl -u earendil-bridge -f
```

Servis davranışı:

- Pi açılınca otomatik başla
- ağ henüz hazır değilse bekle veya tekrar dene
- H7 serial cihazı hazır değilse process tamamen kapanmasın
- serial reconnect loop çalışsın
- process çökerse systemd tekrar başlatsın
- servis kapanırken socket ve serial güvenli kapatılsın

---

# 16. Uygulama Sırası

| Sıra | İş | Çıktı |
|---:|---|---|
| 1 | Mevcut serial GUI sürümünü Git branch/tag ile yedekle | Güvenli geri dönüş noktası |
| 2 | Mevcut TX/RX komut ve parser envanterini çıkar | Dönüşüm kapsamı |
| 3 | Raspberry Pi bridge prototipini yaz | TCP↔Serial byte aktarımı |
| 4 | Bridge'i terminal veya küçük test client ile doğrula | H7 uçtan uca haberleşme |
| 5 | `/dev/serial/by-id/...` cihaz yolunu sabitle | Kararlı serial bağlantı |
| 6 | Tek client politikası ekle | Kontrol sahipliği |
| 7 | Disconnect-stop ekle | Temel güvenlik |
| 8 | Serial reconnect ekle | H7 kablo kopma toleransı |
| 9 | Bridge loglama ekle | Hata ayıklama |
| 10 | systemd servisi oluştur | Otomatik başlatma |
| 11 | GUI'ye `QTcpSocket` katmanı ekle | TCP client |
| 12 | TCP RX buffer'ı `_on_rx_line()` fonksiyonuna bağla | Parser entegrasyonu |
| 13 | `_send_cmd()` fonksiyonunu TCP'ye taşı | Tüm GUI komutlarının TCP kullanması |
| 14 | Serial port ve baud rate arayüzünü kaldır | Network-only GUI |
| 15 | H7 Console ve manuel terminali kaldır | Serial monitorün iptali |
| 16 | `pyserial`, `SerialReaderThread`, `self.ser` temizliği yap | PC serial bağımlılığının kaldırılması |
| 17 | Bağlantı state machine'i tamamla | Kararlı UI davranışı |
| 18 | Disconnect cleanup işlemlerini tamamla | Eski komut güvenliği |
| 19 | Motor entegrasyonunu test et | Sürüş doğrulaması |
| 20 | IMU entegrasyonunu test et | Sensör doğrulaması |
| 21 | Manipulation entegrasyonunu test et | Kol kontrol doğrulaması |
| 22 | Drill entegrasyonunu test et | Sondaj doğrulaması |
| 23 | Wi-Fi kopma testlerini yap | Ağ güvenliği |
| 24 | Pi restart ve bridge crash testlerini yap | Servis dayanıklılığı |
| 25 | H7 serial disconnect testini yap | Serial reconnect |
| 26 | Uzun süreli yoğun telemetri testi yap | Kararlılık |
| 27 | Outdoor saha kabul testini tamamla | Proje kapanışı |

---

# 17. Test Planı

## 17.1 Temel Bağlantı Testleri

| Test | Adımlar | Beklenen Sonuç |
|---:|---|---|
| 1 | Pi bridge başlat, GUI'den IP ve port ile bağlan | GUI `CONNECTED` gösterir |
| 2 | Yanlış IP ile bağlan | GUI kontrollü hata gösterir |
| 3 | Yanlış port ile bağlan | Connection refused düzgün işlenir |
| 4 | Bridge kapalıyken bağlan | GUI donmadan hata verir |
| 5 | Bağlantıyı normal Disconnect butonuyla kapat | State'ler temizlenir |

## 17.2 Komut Testleri

| Test | Komut/İşlem | Beklenen Sonuç |
|---:|---|---|
| 1 | `identify` | H7 komutu alır ve cevap verir |
| 2 | `stop` | H7 ve motorlar stop olur |
| 3 | `brake` | H7 brake davranışı uygular |
| 4 | `mode manual` | Operating mode telemetrisi GUI'yi günceller |
| 5 | Motor forward komutu | Doğru motor komutu uygulanır |
| 6 | IMU read/init komutu | IMU parser GUI tablosunu günceller |
| 7 | Arm command | Manipulation telemetrisi güncellenir |
| 8 | Drill command | Drill telemetrisi güncellenir |
| 9 | Motor tuning sequence | Kuyruk TCP üzerinden doğru sırayla gider |
| 10 | cfgread | Response parser mevcut şekilde çalışır |

## 17.3 TCP Framing Testleri

| Test | Giriş | Beklenen Sonuç |
|---:|---|---|
| 1 | Bir satırı 3 TCP parçasına böl | Parser yalnızca tam satırda çalışır |
| 2 | 3 satırı tek TCP paketinde gönder | Üç satır ayrı işlenir |
| 3 | `\r\n` ile satır gönder | `\r` temizlenir |
| 4 | Geçersiz UTF-8 byte gönder | Uygulama çökmez, replacement decode kullanır |
| 5 | Çok uzun ve newline içermeyen veri gönder | RX buffer sınırı devreye girer |

## 17.4 Güvenlik Testleri

| Test | Senaryo | Beklenen Sonuç |
|---:|---|---|
| 1 | Rover hareket ederken GUI Disconnect | Bridge stop gönderir |
| 2 | Rover hareket ederken GUI process kill | Bridge socket kopmasını algılar ve stop gönderir |
| 3 | Wi-Fi bağlantısını kes | Rover timeout süresi içinde durur |
| 4 | Raspberry Pi güç kaybı | H7 watchdog rover'ı durdurur |
| 5 | Bridge process kill | H7 watchdog durdurur, systemd bridge'i yeniden başlatır |
| 6 | H7 USB kablosunu çıkar | Bridge serial hatayı loglar ve reconnect dener |
| 7 | Reconnect sonrası W tuşuna basmadan bekle | Rover kendiliğinden hareket etmez |
| 8 | Eski tuning queue varken bağlantıyı kes | Queue iptal edilir |

## 17.5 Çoklu İstemci Testi

| Test | Adım | Beklenen Sonuç |
|---:|---|---|
| 1 | Birinci GUI bağlanır | Kabul edilir |
| 2 | İkinci GUI bağlanmayı dener | Reddedilir veya hemen kapatılır |
| 3 | Birinci GUI ayrılır | Client ownership temizlenir |
| 4 | İkinci GUI tekrar bağlanır | Kabul edilir |

## 17.6 Uzun Süreli Test

En az 30–60 dakika boyunca:

- motor telemetrisi
- IMU stream
- manipulation polling
- drill polling
- operating mode
- UART error telemetry
- tuning/config read

aynı anda test edilmelidir.

İzlenecek metrikler:

| Metrik | Beklenti |
|---|---|
| GUI RAM | Sürekli artmamalı |
| GUI CPU | Kabul edilebilir seviyede kalmalı |
| Bridge RAM | Sabit veya sınırlı dalgalanma |
| Bridge CPU | Sürekli %100 olmamalı |
| TCP reconnect | Kontrollü çalışmalı |
| Serial reconnect | Kontrollü çalışmalı |
| Parser gecikmesi | Gözle görülür backlog olmamalı |
| GUI responsive durumu | Donmamalı |
| Telemetri kaybı | Kabul edilebilir seviyede veya sıfır |
| Log boyutu | Sınırsız büyümemeli |

---

# 18. Hata Senaryoları ve Beklenen Davranışlar

| Hata | GUI Davranışı | Bridge Davranışı | H7 Davranışı |
|---|---|---|---|
| Yanlış IP | Hata gösterir | Etkilenmez | Etkilenmez |
| Yanlış port | Connection refused | Etkilenmez | Etkilenmez |
| Wi-Fi kısa süreli kopma | Disconnect state | Stop gönderir | Stop uygular |
| Wi-Fi tamamen kopma | Reconnect bekler | Client bekler | Watchdog stop |
| GUI crash | Socket kapanır | Stop gönderir | Stop uygular |
| Bridge crash | TCP kapanır | systemd restart | Watchdog stop |
| Pi power loss | TCP kapanır | Çalışamaz | Watchdog stop |
| H7 serial disconnect | TCP bağlı kalabilir ama hata gösterilmeli | Serial reconnect dener | Haberleşme yok |
| H7 reboot | Telemetri kesilir ve geri gelir | Serial açık kalabilir/reconnect | Boot sonrası tekrar çalışır |
| İkinci client | Bağlanamaz | Reddeder | Etkilenmez |
| Uzun komut kuyruğu | Disconnect'te iptal | Eski veri tutulmaz | Eski komut uygulanmaz |
| Parser hatası | GUI loglar, çökmemeli | Etkilenmez | Etkilenmez |

---

# 19. Kod Temizliği Kontrol Listesi

TCP dönüşümü tamamlandıktan sonra PC GUI kodunda aşağıdaki ifadeler aratılmalıdır:

```text
serial
SerialReaderThread
self.ser
ttyACM
ttyUSB
COM
baud
list_ports
SerialException
RX-H7
TX-H7
H7 Console
```

Beklenen:

- Uygulama mantığında serial referansı kalmamalı.
- Yalnızca geçmiş yorum veya dokümantasyon varsa temizlenmeli.
- PC requirements dosyasında `pyserial` olmamalı.
- Raspberry Pi bridge requirements dosyasında `pyserial` bulunmalı.

---

# 20. Önerilen Dosya Ayrımı

Mevcut uygulama tek dosya olarak kalacaksa dönüşüm yapılabilir. Ancak orta vadede şu ayrım daha sağlıklı olacaktır:

```text
earendil/
├── main.py
├── gui/
│   ├── main_window.py
│   ├── motor_panel.py
│   ├── imu_panel.py
│   ├── arm_panel.py
│   └── drill_panel.py
├── transport/
│   ├── tcp_client.py
│   └── connection_state.py
├── parsers/
│   ├── motor_parser.py
│   ├── imu_parser.py
│   ├── arm_parser.py
│   └── drill_parser.py
└── config/
    └── network.json
```

Ancak ilk dönüşüm sırasında büyük dosya ayrıştırması ile TCP refactor aynı anda yapılmamalıdır.

Önerilen yaklaşım:

1. Önce taşıma katmanını serial'dan TCP'ye geçir.
2. Tüm testleri tamamla.
3. Daha sonra dosya/modül ayrımına geç.

Bu, hata kaynağını azaltır.

---

# 21. Nihai Kabul Kriterleri

| No | Kabul Kriteri |
|---:|---|
| 1 | GUI yalnızca kontrol bilgisayarında çalışıyor. |
| 2 | Raspberry Pi yalnızca TCP–Serial bridge görevi görüyor. |
| 3 | STM32H723 Raspberry Pi'ye USB Serial/UART üzerinden bağlı. |
| 4 | PC GUI'de `pyserial` bağımlılığı bulunmuyor. |
| 5 | GUI'de serial port seçimi bulunmuyor. |
| 6 | GUI'de baud rate seçimi bulunmuyor. |
| 7 | GUI'de serial monitor bulunmuyor. |
| 8 | GUI'de H7 Console bulunmuyor. |
| 9 | GUI'de manuel H7 komut input'u bulunmuyor. |
| 10 | GUI'de Raspberry Pi IP ve TCP port alanı bulunuyor. |
| 11 | GUI bağlantı durumu açık biçimde gösteriliyor. |
| 12 | Tüm GUI komutları merkezî `_send_cmd()` üzerinden TCP'ye gidiyor. |
| 13 | Tüm H7 satırları TCP buffer sonrası `_on_rx_line()` fonksiyonuna gidiyor. |
| 14 | Motor telemetry parser değişmeden çalışıyor. |
| 15 | IMU parser değişmeden çalışıyor. |
| 16 | Manipulation parser değişmeden çalışıyor. |
| 17 | Drill parser değişmeden çalışıyor. |
| 18 | Motor tuning ve cfgread TCP üzerinden çalışıyor. |
| 19 | TCP stream parçalanması doğru yönetiliyor. |
| 20 | TCP bağlantısı kopunca GUI kontrol state'leri temizleniyor. |
| 21 | TCP bağlantısı kopunca Raspberry Pi H7'ye stop gönderiyor. |
| 22 | Pi tamamen kaybolunca H7 watchdog sistemi durduruyor. |
| 23 | Reconnect sonrası eski hareket komutu uygulanmıyor. |
| 24 | Aynı anda yalnızca bir kontrol istemcisi bağlanabiliyor. |
| 25 | Raspberry Pi açıldığında bridge otomatik başlıyor. |
| 26 | Bridge çökerse systemd otomatik yeniden başlatıyor. |
| 27 | H7 serial bağlantısı kopup geldiğinde bridge yeniden bağlanıyor. |
| 28 | Ham H7 telemetrisi GUI Console'a basılmıyor. |
| 29 | Bridge logları `journalctl` üzerinden görülebiliyor. |
| 30 | Sistem yoğun telemetri altında en az 30–60 dakika kararlı çalışıyor. |
| 31 | Outdoor private Wi-Fi ağı üzerinde saha testi tamamlanmış. |
| 32 | Wi-Fi kopması sırasında rover güvenli şekilde duruyor. |

---

# 22. Proje Tamamlanma Tanımı

Proje aşağıdaki koşullar birlikte sağlandığında tamamlanmış sayılacaktır:

```text
PC GUI
  ├── yalnızca TCP client
  ├── IP/port ile bağlantı
  ├── serial bağımlılığı yok
  ├── serial monitor yok
  ├── mevcut kontrol ve parser özellikleri korunmuş
  └── disconnect durumunda güvenli state temizliği

Raspberry Pi
  ├── tek TCP client kabul ediyor
  ├── TCP ↔ Serial byte bridge
  ├── disconnect'te stop gönderiyor
  ├── serial reconnect yapıyor
  ├── systemd ile otomatik başlıyor
  └── journal üzerinden loglanıyor

STM32H723
  ├── mevcut komut protokolünü kullanıyor
  ├── mevcut telemetri formatını kullanıyor
  ├── bridge üzerinden haberleşiyor
  └── watchdog ile bağımsız fail-safe sağlıyor
```

Nihai veri akışı:

```text
Kullanıcı kontrolü
        ↓
PC PySide6 GUI
        ↓
_send_cmd()
        ↓
QTcpSocket
        ↓
Private Outdoor Wi-Fi
        ↓
Raspberry Pi TCP Server
        ↓
TCP–Serial Bridge
        ↓
USB Serial / UART
        ↓
STM32H723
        ↓
Motor / IMU / Manipulation / Drill sistemleri
```

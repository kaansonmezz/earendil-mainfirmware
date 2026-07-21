# ISSUES

## Network ve Güvenlik

- [ ] TCP bağlantısı zaman zaman kopuyor.
- [ ] GUI tarafında AUTONOMOUS mod için heartbeat gereksinimi kaldırıldı, ancak henüz test edilmedi.
- [ ] E-Stop sistemi bulunmuyor.

## IMU ve Manyetometre

- [ ] IMU ve MAG zaman zaman takılıyor.
- [ ] Veri akışı devam etmesine rağmen ölçümler bir saniyeden kısa süreyle sabit kalabiliyor.
- [ ] Sorun tekrar edilebilir durumda.

## Turn Ratio Davranışları

- [ ] Turn Ratio `0.00`: Tank dönüşü.
- [ ] Turn Ratio `0.50`: Brake Arc Turn.
- [ ] Turn Ratio `0.60`: Pivot Turn.

## Mobility Modları

- [ ] Hızlı kullanım için Mobility modlarına hazır değerler atanmalı.

## Manipülasyon Sistemi

- [ ] Manipülasyon sisteminin GUI komutlarının H7 tarafında karşılıkları bulunmuyor.
- [ ] Tüm manipülasyon komutları yalnızca `arm` prefix'i ile iletiliyor.

## Oyun Kolu

- [ ] GUI'de oyun kolu verilerini okuyan bir sistem bulunmuyor.

## Duty Mode Ayarları

- [ ] Duty Mode varsayılan PWM değerleri gereğinden fazla; azaltılmalı.
- [ ] GUI Duty Mode'da Shift/Ctrl ile duty artırma ve azaltma adımı şu anda `100 PWM`; tercihen `50 PWM` yapılmalı.
- [ ] Maksimum PWM değeri `4000` yerine yaklaşık `1500–1800` aralığına düşürülmeli.

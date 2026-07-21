# TCP UART Bridge

## Seri portu kontrol et

```bash
ls -la /dev/serial/by-id/
```

## Port 5000 kullanımda mı kontrol et

```bash
sudo ss -ltnp | grep ':5000'
```

Eski bridge süreçlerini kapatmak için:

```bash
pkill -9 -f tcp_uart_bridge.py
```

## Localhost bağlantısı

GUI ve bridge aynı Raspberry Pi üzerinde çalışıyorsa:

```bash
cd ~/Desktop

python3 tcp_uart_bridge.py \
  --host 127.0.0.1 \
  --port 5000 \
  --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_003A00303434511834313937-if02 \
  --baud 115200 \
  --allow-client 127.0.0.1/32 \
  --log-level INFO
```

GUI bağlantısı:

```text
Host: 127.0.0.1
Port: 5000
```

## Private network bağlantısı

Raspberry Pi IP adresini öğren:

```bash
hostname -I
```

Kontrol bilgisayarının IP adresini öğren:

```bash
hostname -I
```

Bridge'i Raspberry Pi üzerinde başlat:

```bash
cd ~/Desktop

python3 tcp_uart_bridge.py \
  --host 0.0.0.0 \
  --port 5000 \
  --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_003A00303434511834313937-if02 \
  --baud 115200 \
  --allow-client <PC_IP>/32 \
  --log-level INFO
```

`10.201.6.104` yerine GUI'nin çalıştığı kontrol bilgisayarının IP adresini yaz.

GUI bağlantısı:

```text
Host: RASPBERRY_PI_IP
Port: 5000
```

## Hem localhost hem private network

```bash
cd ~/Desktop

python3 tcp_uart_bridge.py \
  --host 0.0.0.0 \
  --port 5000 \
  --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_003A00303434511834313937-if02 \
  --baud 115200 \
  --allow-client 127.0.0.1/32 \
  --allow-client 10.201.6.104/32 \
  --log-level INFO
```

Bridge tek istemci kabul eder. Localhost GUI ile private-network GUI aynı anda bağlanamaz.

Bridge'i kapatmak için:

```text
Ctrl+C
```

`Ctrl+Z` kullanma; süreç duraklatılır fakat portu bırakmaz.

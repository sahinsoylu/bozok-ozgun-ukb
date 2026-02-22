# 🚀 BOZOK Uçuş Kontrol Bilgisayarı (UKB)
[![tr](https://img.shields.io/badge/lang-tr-red.svg)](https://github.com/sahinsoylu/bozok-ozgun-ukb/blob/main/README.md)
[![en](https://img.shields.io/badge/lang-en-green.svg)](https://github.com/sahinsoylu/bozok-ozgun-ukb/blob/main/README.en.md)

**2025 TEKNOFEST Roket Yarışması – 1.lik**  
Geliştiren: **Bozok Roket Takımı**

Bu depo, 2025 TEKNOFEST Roket Yarışması kapsamında uçuşta aktif olarak kullanılmış olan özel tasarım Uçuş Kontrol Bilgisayarı (UKB) yazılımını içermektedir.

Sistem yarışma uçuşlarında aktif görev yapmış ve tüm aviyonik gereksinimleri karşılamıştır.

---

# 📌 Sistem Genel Bakış

BOZOK Uçuş Kontrol Bilgisayarı, yüksek güvenilirlik gerektiren roket görevleri için tasarlanmış gerçek zamanlı bir gömülü sistemdir.

Yazılım:

- FreeRTOS üzerinde çalışır  
- Ana uçuş algoritmasını 80 Hz frekansta yürütür  
- LoRa üzerinden telemetri gönderir  
- Basınç ve IMU verileriyle uçuş durumunu belirler  
- Apogee (en yüksek nokta) tespiti yapar  
- Kurtarma sistemini tetikler  
- SIT ve SUT test modlarını destekler  

---

# ⚙️ Gerçek Zamanlı Mimari

Sistem FreeRTOS tabanlıdır ve iki ana görevden oluşur:

---

## 1️⃣ sensorTask

Çalışma frekansı:

- NORMAL modda: 80 Hz  
- SIT / SUT modlarında: 10 Hz  

Görevleri:

- IMU veri okuma (I2C)
- BMP390 basınç ve irtifa hesaplama
- UART interrupt ile GPS NMEA parsing
- FIFO tabanlı düşey hız hesaplama (Linear Regression)
- Uçuş durum makinesinin yönetimi
- Apogee tespit algoritması
- Kurtarma tetikleme kararı

---

## 2️⃣ loraTask

Çalışma frekansı:

- Her 500 ms

Görevleri:

- Telemetri paketi oluşturma
- Big Endian formatta veri kodlama
- 8-bit checksum ekleme
- UART üzerinden LoRa modülüne iletim

---

# 🧠 Uçuş Durum Makinesi

Sistemde aşağıdaki uçuş durumları bulunmaktadır:

```
ON_GROUND
PROTECTED_FLIGHT
ALTITUDE_PROTECTED_FLIGHT
FREE_FLIGHT
APOGEE
TRIGGER
```

---

## 🚀 Fırlatma Tespiti

- NORMAL modda: Basınç düşüş eşiğine göre  
- SUT modunda: İvme eşiğine göre  

---

## 🎯 Apogee (En Yüksek Nokta) Tespiti

### NORMAL Mod

- Minimum basınç sürekli takip edilir  
- Belirli eşik üzerinde doğrulanmış basınç artışı aranır  
- Gürültü kaynaklı hatalı tetiklemeleri önlemek için sayaç doğrulaması kullanılır  

### SUT Mod

- İrtifanın sürekli azalmaya başlaması kontrol edilir  
- Sayaç tabanlı doğrulama ile güvenilirlik sağlanır  

---

# 📈 Düşey Hız Hesaplama

Düşey hız basit türev alınarak hesaplanmaz.

Bunun yerine FIFO tamponunda saklanan irtifa-zaman örnekleri üzerinden:

Linear Regression (En Küçük Kareler Yöntemi)

ile hesaplanır.

Avantajları:

- Gürültüye dayanıklı  
- Basınç jitter’ına karşı stabil  
- Apogee tespitinde güvenilir  

---

# 📡 Telemetri Paket Yapısı

Telemetri paketi şunları içerir:

- Başlık: 0xAA
- İvme verileri (int16, ölçeklenmiş)
- Jiroskop verileri (int16, ölçeklenmiş)
- Quaternion (float)
- GPS enlem & boylam (int32, ölçeklenmiş)
- GPS irtifa
- Barometrik irtifa
- Yüzey açısı
- Uçuş durumu
- 8-bit checksum
- Bitiş (LF)

Tüm çok baytlı veriler Big Endian formatta iletilir.

---

# 🧪 Test Modları

## 🔹 NORMAL
Gerçek uçuş modu.

## 🔹 SIT (System Integration Test)
- Filtrelenmiş sensör verileri UART üzerinden gönderilir  
- Entegrasyon doğrulaması için kullanılır  

## 🔹 SUT (Sensor Unit Test)
- Harici simüle edilmiş sensör verisi kabul eder  
- Hardware-in-the-loop test imkanı sağlar  

---

# 🔐 Güvenlik ve Dayanıklılık

- Deterministik zamanlama (FreeRTOS)
- Sayaç tabanlı apogee doğrulaması
- Gürültüye dayanıklı basınç algoritması
- Paylaşılan veriler için kritik bölge koruması
- DMA tabanlı UART alımı
- Fail-safe reset fonksiyonu

---

# 🛠 Kullanılan Teknolojiler

- STM32 Mikrodenetleyici
- FreeRTOS
- LoRa Haberleşme
- UART DMA
- I2C Sensör İletişimi
- C (Gömülü Sistemler)

---

# 🏆 Başarı

🥇 2025 TEKNOFEST Roket Yarışması – 1.lik

---

# ⚠️ Sorumluluk Reddi

Bu depo eğitim ve araştırma amaçlı paylaşılmıştır.  
Uçuş kritik sistemler gerçek görevlerde kullanılmadan önce kapsamlı test ve doğrulama süreçlerinden geçirilmelidir.

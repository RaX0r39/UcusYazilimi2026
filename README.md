# 🚀 Trakya Roket 2026 - Uçuş Yazılımı (V2.0)

![ESP32](https://img.shields.io/badge/Hardware-ESP32--WROOM--32-blue?style=for-the-badge&logo=espressif)
![Framework](https://img.shields.io/badge/Framework-Arduino%20/%20PlatformIO-orange?style=for-the-badge&logo=arduino)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

**Trakya Roket Takımı 2026** için geliştirilmiş, yüksek performanslı ve hata toleranslı uçuş kontrol yazılımı. Bu yazılım, çift çekirdekli asenkron mimari, gelişmiş filtreleme algoritmaları ve kapsamlı yer test modları (SİT/SUT) ile donatılmıştır.

---

## 🛠️ Teknik Özellikler & Mimari

Sistem, ESP32'nin **Dual-Core** yapısını kullanarak kritik uçuş görevlerini ve çevresel birimleri birbirinden ayırır.

### 🧠 Çift Çekirdekli (Dual-Core) İşleme
*   **CORE 0 (Uçuş Kontrolü):** Sensörlerden veri toplama, **Kalman Filtresi** uygulamaları ve **Uçuş Algoritması** (Durum Makinesi) bu çekirdekte saniyede 100 kez (100 Hz) koşturulur.
*   **CORE 1 (Haberleşme & Kayıt):** LoRa üzerinden veri iletimi, SD karta loglama ve yer istasyonu komutlarının dinlenmesi görevlerini üstlenir.

### 🛰️ Sensör Füzyonu & Filtreleme
- **BNO055 (IMU):** 9-Eksenli oryantasyon ve doğrusal ivme verisi.
- **BME280 (Barometre):** Yüksek hassasiyetli basınç ve irtifa takibi.
- **NEO-7M (GPS):** Global konum ve irtifa verisi.
- **Kalman Filtresi:** Tüm sensör verileri gürültüden arındırılarak güvenilir veri seti oluşturulur.

---

## 🏗️ Uçuş Durum Makinesi (State Machine)

Roket yazılımı, uçuşun her anını 5 temel durumda takip eder:

1.  **HAZIR:** Rampa üzerinde, sensörler kalibre edilmiş, fırlatma bekleniyor.
2.  **YUKSELIYOR:** İvme eşiği aşıldı, motor yanma ve sürüklenme fazı.
3.  **INIS_1 (Drogue):** Apogee (En yüksek nokta) tespit edildi, birinci paraşüt ateşlendi.
4.  **INIS_2 (Ana):** Belirlenen irtifada (örn. 600m) ana paraşüt ateşlendi.
5.  **INDI:** Hareket kesildi, GPS konumu üzerinden kurtarma bekliyor.

---

## 🧪 SİT & SUT Test Modları

Yazılım, yarışma gereksinimlerine tam uyumlu gelişmiş test modları içerir:

-   **SİT (Sensör İzleme Testi):** Sensör verilerinin binary protokol üzerinden canlı izlenmesi.
-   **SUT (Sentetik Uçuş Testi):** Uçuş algoritmasının PC üzerinden gönderilen simülasyon verileriyle test edilmesi.

> [!TIP]
> Test süreçleri için `SİT_SUT/` klasörü altındaki Python simülatörünü ve birim testlerini kullanabilirsiniz.

---

## 📂 Proje Yapısı

```bash
├── src/
│   ├── main.cpp            # Ana uçuş yazılımı ve algoritma
│   └── SİT_SUT/
│       └── SİT-SUT.cpp      # SİT/SUT entegrasyon dosyası
├── SİT_SUT/
│   ├── sit_sut_test.py      # Python yer istasyonu simülatörü
│   └── sit-sut-dokümantasyon.md # Teknik detaylar
├── test/
│   └── test_sitsut/         # PlatformIO birim testleri
└── platformio.ini           # Kütüphane ve donanım bağımlılıkları
```

---

## 🚀 Kurulum ve Kullanım

1.  **PlatformIO IDE** (VS Code) üzerinden projeyi açın.
2.  Bağımlılıkların otomatik yüklenmesini bekleyin.
3.  **Port Seçimi:** `SİT_SUT/sit_sut_test.py` dosyasını çalıştırarak roketinizi bilgisayardan yönetebilirsiniz.
4.  **Derleme:** `pio run` veya IDE üzerinden derle/yükle butonunu kullanın.

---

## 🤝 Katkıda Bulunma

Bu proje Trakya Roket Takımı iç kullanımı için geliştirilmektedir. Ancak eğitim amaçlı incelemeler ve geri bildirimler her zaman memnuniyetle karşılanır.

---
**Trakya Roket Takımı 2026**


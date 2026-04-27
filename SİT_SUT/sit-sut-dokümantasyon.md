# Trakya Roket 2026 - SİT/SUT Sistem Dokümantasyonu

Bu doküman, uçuş yazılımı içerisinde yer alan **Sensör İzleme Testi (SİT)** ve **Sentetik Uçuş Testi (SUT)** sistemlerinin teknik detaylarını, haberleşme protokollerini ve kullanım yönergelerini içerir.

---

## 1. Sisteme Genel Bakış

SİT/SUT sistemi, roketin donanım ve yazılım bütünlüğünü uçuş öncesinde doğrulamak için geliştirilmiştir. Sistem **çift çekirdek (Dual Core)** mimarisi üzerinde asenkron olarak çalışır.

- **SİT (Sensör İzleme Testi):** Gerçek zamanlı sensör verilerinin (BNO055, BME280) TTL üzerinden yer istasyonuna binary formatta aktarılmasıdır.
- **SUT (Sentetik Uçuş Testi):** Donanımsal sensörlerin devre dışı bırakılıp, algoritmanın dışarıdan gelen simülasyon verileriyle beslenmesidir.

---

## 2. Haberleşme Protokolü

Tüm iletişim **TTL (UART0)** portu üzerinden **9600 Baud** hızında gerçekleşir. Paketler binary (ham byte) formatındadır.

### 2.1. Komut Paketleri (PC -> ROKET)
Komutlar her zaman **5 Byte** uzunluğundadır.

| Byte | Tanım | Değer |
| :--- | :--- | :--- |
| 1 | Header | `0xAA` |
| 2 | Komut | `0x20` (SİT), `0x22` (SUT), `0x24` (STOP) |
| 3 | Checksum | `CMD + 0x6C` |
| 4 | Footer 1 | `0x0D` (\r) |
| 5 | Footer 2 | `0x0A` (\n) |

### 2.2. Veri Paketleri (ROKET <-> PC)
SİT gönderimi ve SUT veri alımı için kullanılan paket **36 Byte** uzunluğundadır.

| Bölüm | Boyut | Tanım |
| :--- | :--- | :--- |
| Header | 1 Byte | `0xAB` |
| Payload | 32 Byte | 8 adet `float32` (İrtifa, Basınç, İvmeX/Y/Z, AçıX/Y/Z) |
| Checksum | 1 Byte | Payload byte'larının toplamı (8-bit sum) |
| Footer | 2 Byte | `0x0D`, `0x0A` |

---

## 3. Çalışma Modları

### 3.1. SİT Modu İşleyişi
1. Yer istasyonundan `0x20` komutu gönderilir.
2. `sitSutMod` değişkeni `MOD_SIT` olur.
3. Core 0 (Task1), her döngüde güncel sensör verilerini `0xAB` header'ı ile TTL'den basmaya başlar.

### 3.2. SUT Modu İşleyişi
1. Yer istasyonundan `0x22` komutu gönderilir.
2. Core 0, donanımsal sensör okumayı ve Kalman filtrelerini askıya alır.
3. Yer istasyonu `0xAB` header'ı ile 36 byte'lık veri paketleri göndermeye başlar.
4. Core 1 (Task2), gelen paketleri parse ederek küresel sensör değişkenlerini günceller.
5. Uçuş algoritması bu "yapay" verilerle sanki uçuyormuş gibi kararlar üretir.

---

## 4. Test ve Doğrulama

### 4.1. Python Test Aracı (`sit_sut_test.py`)
Bilgisayar üzerinden sistemi kontrol etmek için kullanılan scripttir. Şu işlevleri sunar:
- Manuel komut gönderimi.
- SİT verilerinin canlı grafiksel/metinsel takibi.
- Otomatik uçuş senaryosu (simülasyon) koşturma.

### 4.2. Birim Testleri (`test/test_main.cpp`)
PlatformIO Unity framework'ü kullanılarak yazılmıştır. `pio test` komutu ile çalıştırılır.
- Checksum hesaplama doğruluğu.
- Paket yapısı (struct padding) uyumluluğu.
- Protokol limit testlerini içerir.

---

## 5. Önemli Notlar ve Güvenlik

> [!IMPORTANT]
> SUT modundayken roketin gerçek GPS verileri güncellenmez; ancak LoRa telemetrisi SUT verileriyle birlikte akmaya devam eder.

> [!WARNING]
> SİT/SUT aktifken fünye çıkışları algoritma tarafından tetiklenebilir. Testler sırasında ateşleyicilerin (e-match) bağlı olmadığından emin olunmalıdır.

---
*Hazırlayan: Antigravity AI Coding Assistant*
*Tarih: 27 Nisan 2026*

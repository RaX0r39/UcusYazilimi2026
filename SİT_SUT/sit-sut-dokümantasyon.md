# 🚀 Trakya Roket 2026 - SİT/SUT Teknik El Kitabı

Bu doküman, uçuş yazılımı içerisinde yer alan **Sensör İzleme Testi (SİT)** ve **Sentetik Uçuş Testi (SUT)** sistemlerinin çalışma prensiplerini, veri yapılarını ve operasyonel süreçlerini detaylandırmak için hazırlanmıştır.

---

## 1. Giriş: SİT ve SUT Nedir?

Roket sistemlerinde uçuş öncesi testler kritik öneme sahiptir. Bu sistem, roketi fiziksel olarak fırlatmadan tüm elektronik ve yazılımsal süreçleri simüle etmemize olanak tanır.

*   **SİT (Sensör İzleme Testi):** "Roket şu an ne görüyor?" sorusunun cevabıdır. Sensörlerden gelen ham verileri filtrelemeden veya işlemeden olduğu gibi izlememizi sağlar.
*   **SUT (Sentetik Uçuş Testi):** "Roket bu veriyi alsa ne yapardı?" sorusunun cevabıdır. Uçuş algoritmasını, dışarıdan (PC) gönderilen yapay verilerle besleyerek apogee tespiti ve paraşüt açma gibi kritik kararları test etmemizi sağlar.

---

## 2. Mimari Yapı ve Veri Akışı

Sistem, ESP32'nin **çift çekirdekli (Dual-Core)** yapısını kullanarak uçuş güvenliğini tehlikeye atmadan çalışır.

### 🧠 Çekirdek Görev Dağılımı
| Çekirdek | Görev Adı | Sorumluluk |
| :--- | :--- | :--- |
| **Core 0** | `UcusGörevi` | Sensör okuma, SUT verilerini algoritmaya besleme, Fünye kontrolü. |
| **Core 1** | `HaberlesmeGörevi` | TTL hattını dinleme, Komutları parse etme, LoRa/SD loglama. |

### 🔄 Veri Akış Şeması
1.  **Komut Alımı:** Core 1, TTL (UART0) üzerinden gelen 5 byte'lık komutu yakalar.
2.  **Mod Değişimi:** Komut geçerliyse küresel `sitSutMod` değişkeni güncellenir.
3.  **Algoritma Tetikleme:** Core 0, bu değişkeni kontrol ederek ya sensörleri okur ya da TTL'den gelen veriyi bekler.
4.  **Geri Bildirim:** Core 0 veriyi işler, Core 1 ise sonucu LoRa ve SD karta yazar.

---

## 3. Haberleşme Protokolü Detayları

Tüm iletişim **UART0 (TTL)** üzerinden **9600 Baud** hızında, binary formatta yapılır.

### 3.1. Komut Yapısı (5 Byte)
PC'den rokete gönderilen kontrol paketleridir.
`[HEADER][COMMAND][CHECKSUM][FOOTER1][FOOTER2]`

- **Header:** `0xAA` (Sabit)
- **Komutlar:** 
  - `0x20`: SİT Başlat
  - `0x22`: SUT Başlat
  - `0x24`: Testi Durdur (Bekleme Modu)
- **Checksum:** `CMD + 0x6C` (Taşmalar 8-bit olarak hesaplanır)
- **Footer:** `0x0D 0x0A` (\r\n)

### 3.2. Veri Paketi Yapısı (36 Byte)
SİT veri gönderimi ve SUT veri girişi için kullanılan standart pakettir.
`[0xAB][32 Byte Payload][Checksum][0x0D][0x0A]`

**Payload İçeriği (Her biri 4 Byte - Float):**
1. İrtifa (m)
2. Basınç (hPa)
3. İvme X (m/s²)
4. İvme Y
5. İvme Z
6. Açı X (Roll)
7. Açı Y (Pitch)
8. Açı Z (Yaw)

---

## 4. Kullanım Rehberi

### 🛠️ SİT Modu Nasıl Kullanılır?
1. Roketi bilgisayara TTL-USB dönüştürücü ile bağlayın.
2. `sit_sut_test.py` uygulamasını çalıştırın ve portu seçin.
3. **Seçenek 1**'e basın. Ekranda sensörlerden gelen canlı verileri (İrtifa, Basınç vb.) göreceksiniz.
4. Sensörleri hareket ettirerek verilerin değiştiğini doğrulayın.

### 🚀 SUT (Simülasyon) Nasıl Yapılır?
1. **Seçenek 2** ile SUT modunu aktif edin.
2. Roket artık gerçek sensörleri okumayı bırakır ve sizden veri bekler.
3. **Seçenek 4**'e basarak hazır bir "Yükseliş Senaryosu" başlatın.
4. Roketin LoRa üzerinden gönderdiği telemetriyi izleyerek; irtifa yükseldiğinde algoritmanın "Yükseliyor" durumuna geçip geçmediğini kontrol edin.

---

## 5. Güvenlik ve Uyarılar

> [!IMPORTANT]
> **Donanım Kilidi:** SUT modundayken algoritma paraşütleri gerçekten ateşleyebilir. Test sırasında fünyelerin takılı olmadığından emin olun veya sadece yazılımsal durumları izleyin.

> [!WARNING]
> **Gecikme Payı:** TTL üzerinden veri gönderirken saniyede 20 paketi (20 Hz) geçmemeye özen gösterin. Yüksek hızlar buffer taşmasına neden olabilir.

> [!TIP]
> Test bittikten sonra mutlaka **Seçenek 3 (Durdur)** ile sistemi bekleme moduna alın. Aksi takdirde roket gerçek sensörleri okumaya geri dönmez.

---
*Trakya Roket Takımı 2026 - Yazılım Alt Birimi*

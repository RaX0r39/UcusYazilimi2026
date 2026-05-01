# Trakya Roket 2026 - Bilimsel Görev Yükü Teknik Raporu

### iv. Bilimsel Görev Yükü Veri Mimarisi ve Kesintisiz İletimi

Bilimsel görev yükü (BGY), roketin uçuş profilini etkilemeden bağımsız bilimsel veriler toplamak ve bu verileri gerçek zamanlı olarak yer istasyonuna indirmek üzere tasarlanmıştır. Veri mimarisi, ana uçuş bilgisayarı (UKB) ile modüler bir uyum içerisindedir ancak sensör çeşitliliği ve veri paketi içeriği bilimsel görev gereksinimlerine göre özelleştirilmiştir.

#### 1. Görev Akışı ve Ayrılma Mekanizması
Bilimsel görev yükü, roketin ana gövdesinden uçuşun tepe noktasında (**Apogee**) ayrılacak şekilde entegre edilmiştir. 
- **Ayrılma Zamanlaması:** UKB algoritması, BME280 (Barometrik) ve BNO055 (Ataletsel) sensör füzyonu ile apogee anını saptar.
- **Tetikleme:** Apogee doğrulaması yapıldığı anda UKB, görev yükü ayrılma sistemini aktif eder. Görev yükü, ayrılma anından itibaren (veya tercihen kalkıştan itibaren) veri indirme görevini icra etmeye başlar.

#### 2. Veri Mimarisi ve Protokol Detayları (Kritik Tasarım Seviyesi)
Görev yükü veri mimarisi, yüksek hızlı veri işleme ve hatasız iletim prensibi üzerine kurulmuştur. Veri akışı, ESP32'nin çift çekirdekli yapısı kullanılarak optimize edilmiştir.

*   **Veri Edinme (Core 0):** BME280 ve diğer yardımcı atmosferik sensörlerden gelen hava verileri 100 Hz frekansla okunur. Okunan veriler, titreşim ve sensör gürültüsünden arındırılmak üzere **Kalman Filtresi**'nden geçirilir.
*   **Hareket Verisi Optimizasyonu:** Gereksinim doğrultusunda, görev yükü telemetri paketinden ivme ve jiroskop gibi yüksek bant genişliği işgal eden hareket verileri çıkarılmış; odak noktası tamamen **hava verilerine** (Sıcaklık, Nem, Basınç vb.) kaydırılmıştır.
*   **Haberleşme (Core 1):** Toplanan veriler FreeRTOS kuyrukları üzerinden ikinci çekirdeğe aktarılır. Burada **Çerçevelenmiş İkili (Framed Binary)** formata dönüştürülür.
*   **Hata Denetimi:** RF iletimi esnasında oluşabilecek bozulmaları önlemek amacıyla her veri paketine **CRC16-CCITT** hata kontrol kodu eklenir.

> **[GÖRSEL YERLEŞİMİ: Bilimsel Görev Yükü Veri Akış Diyagramı]**
> *(Atmosferik sensörlerden başlayıp LoRa üzerinden yer istasyonuna ulaşan veri yolunu gösteren blok diyagram.)*

#### 3. Telemetri Paket Yapısı (Scientific Payload Struct)
Veri mimarisinin merkezinde yer alan paket yapısı aşağıda teknik detayıyla sunulmuştur:

| Veri Alanı | Veri Tipi | Açıklama |
| :--- | :--- | :--- |
| **basinc** | float (4B) | Ortam atmosferik basıncı (hPa) |
| **sicaklik** | float (4B) | Ortam sıcaklığı (°C) |
| **nem** | float (4B) | Bağıl nem oranı (%) |
| **irtifa** | float (4B) | Basınç tabanlı hesaplanan anlık irtifa (m) |
| **gpsEnlem** | float (4B) | Konum - Enlem koordinatı |
| **gpsBoylam** | float (4B) | Konum - Boylam koordinatı |
| **paketNo** | uint32_t (4B) | Veri sürekliliği ve kayıp takibi için sayaç |

#### 4. Veri İletim Frekansı ve Uyumluluk
Gereksinim dokümanında belirtilen "asgari 5 Hz" veri indirme frekansı, Trakya Roket veri mimarisi ile fazlasıyla aşılmaktadır:
- **Aktarım Hızı:** Görev yükü verileri yer istasyonuna **10 Hz (saniyede 10 paket)** hızında indirilmektedir.
- **Bant Genişliği Verimliliği:** Hareket verilerinin çıkarılmasıyla paket boyutu küçültülmüş, bu sayede 9600 baud gibi düşük hızlarda bile %100 doluluk oranına ulaşmadan 10 Hz iletim kararlılığı sağlanmıştır.

#### 5. Gereksinimin Karşılandığına Dair Yazılı Teyit
Bilimsel görev yükü veri mimarisi; apogee noktasında ayrılma fonksiyonunu, sadece atmosferik verilere odaklanan optimize edilmiş paket yapısını ve saniyede 10 paket (10 Hz) veri indirme kapasitesini bünyesinde barındırmaktadır.

**Teyit Edilir:** Bilimsel görev yükü tasarımı ve veri mimarisi, asgari 5 Hz veri indirme frekansı ve hava verisi odaklı iletim gereksinimlerini tam olarak karşılamakta olup **Kritik Tasarım Seviyesi (KTR)** gerekliliklerine %100 uygundur.

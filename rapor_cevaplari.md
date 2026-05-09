# Trakya Roket 2026 - Uçuş Yazılımı Rapor Cevapları

Bu dosya, uçuş yazılımı raporu için hazırlanan teknik cevapları ve şema yerleşim planlarını içermektedir.

---

### ii. Görev Yükü ve Roket İçin Bağımsız Konum Belirleyici Sistemler ve Veri Alışveriş Mimarisi

Trakya Roket takımı olarak, uçuş sonrası roketin ana gövdesini (UKB) ve ayrılan Bilimsel Görev Yükünü (GY) güvenli bir şekilde kurtarabilmek için her iki alt sistemde de birbirinden **tamamen bağımsız ve izole** konum belirleyici (GPS) ve radyo vericisi (LoRa) donanımları kullanılmıştır. Bu bağımsız mimari, herhangi bir sistemin arızalanması veya ayrılma sonrası sistemlerin birbirinden uzaklaşması durumunda bile her iki parçanın ayrı ayrı takip edilebilmesini garanti altına alır.

#### 1. Roket (Ana Gövde / UKB) Konum Belirleme ve İletim Mimarisi
Ana gövdenin konumlandırılması uçuş kontrol bilgisayarı (UKB) üzerinden sağlanmaktadır:
*   **Konum Belirleyici (GPS):** UKB üzerinde **GY-NEO-7M** GPS modülü bulunmaktadır. Bu modül UART üzerinden ESP32 (Core 0) ile haberleşerek saniyede asgari 5 kez (5 Hz) enlem ve boylam verilerini günceller.
*   **Radyo Vericisi (LoRa):** Konum verileri, ivme ve irtifa gibi diğer telemetri verileriyle birlikte paketlenerek (Çerçevelenmiş İkili Veri - Framed Binary) **EBYTE E32-433T30D** LoRa modülü üzerinden yer istasyonuna iletilir (Kanal A).
*   **İletim Frekansı:** UKB, GPS verilerini de içeren ana telemetri paketini yer istasyonuna saniyede 10 kez (**10 Hz**) kesintisiz olarak iletir.

#### 2. Görev Yükü Konum Belirleme ve İletim Mimarisi
Roketin tepe noktasında (Apogee) ana gövdeden ayrılan bilimsel görev yükünün kendine ait izole bir aviyonik sistemi bulunmaktadır:
*   **Konum Belirleyici (GPS):** Görev yükü üzerinde de ana sistemden bağımsız ikinci bir **GY-NEO-7M** GPS modülü görev yapmaktadır.
*   **Radyo Vericisi (LoRa):** Görev yükü verileri (basınç, sıcaklık ve kendi GPS koordinatları), görev yüküne entegre edilen **ikinci bir EBYTE E32-433T30D** modülü üzerinden iletilir. RF çakışmasını engellemek için bu modül ana gövdenin LoRa frekansından farklı bir frekansta (**Kanal B**) çalışır.
*   **İletim Frekansı:** Görev yükü, kendi bağımsız konum verisini yer istasyonuna saniyede 10 kez (**10 Hz**) indirir.

#### 3. Yer İstasyonu Veri Alışveriş Mimarisi (Çift Kanallı Haberleşme)
Yer istasyonu, havada birbirinden bağımsız hareket eden roket ve görev yükünden gelen verileri eş zamanlı alabilecek "Çoklu Kanal" (Multi-Channel) yapısında tasarlanmıştır.

*   **Donanım Altyapısı:** Yer istasyonunda, roket (Kanal A) ve görev yükü (Kanal B) frekanslarına ayarlanmış iki farklı LoRa alıcı modül bulunmaktadır. Bu modüller çift yüksek kazançlı Yagi anten ile desteklenmektedir.
*   **Veri Bütünlüğü (CRC Kontrolü):** Gelen her iki konum verisi paketi de (UKB ve BGY), yer istasyonu yazılımına ulaştığında **CRC16-CCITT** hata denetimi algoritmasından geçirilir. Böylece havada sinyal karışması veya atmosferik gürültü sebebiyle yanlış bir GPS koordinatı alınmasının (örneğin 1 bitlik kayma ile farklı bir şehre yönlendirme hatası) %100 önüne geçilir.
*   **Görselleştirme:** Python tabanlı yer istasyonu arayüzümüzde (GUI), roketin ve görev yükünün konumları eş zamanlı olarak harita (Map) üzerinde iki farklı renkte ikon ile işaretlenerek izlenir.

> **[ŞEKİL YERLEŞİMİ: Bağımsız Konum İletimi ve Çift Kanallı Yer İstasyonu Mimarisi]**
> *(Roket ve Görev Yükü üzerindeki bağımsız GPS ve LoRa modüllerinin, yer istasyonundaki iki farklı alıcı ile olan iletişimini gösteren diyagram aşağıda verilmiştir.)*

```mermaid
graph TD
    subgraph ROKET ANA GÖVDE (UKB)
        GPS1[GY-NEO-7M GPS] -->|UART| MCU1[UKB ESP32]
        MCU1 -->|UART| LORA1[EBYTE E32 LoRa - Kanal A]
    end

    subgraph BİLİMSEL GÖREV YÜKÜ (BGY)
        GPS2[GY-NEO-7M GPS] -->|UART| MCU2[BGY ESP32]
        MCU2 -->|UART| LORA2[EBYTE E32 LoRa - Kanal B]
    end

    subgraph YER İSTASYONU
        LORA_RX1[Alıcı LoRa - Kanal A] -->|USB-TTL| PC[Yer İstasyonu Yazılımı]
        LORA_RX2[Alıcı LoRa - Kanal B] -->|USB-TTL| PC
        PC --> MAP1[Roket Konumu Haritada Gösterilir]
        PC --> MAP2[BGY Konumu Haritada Gösterilir]
        PC -.->|Hata Denetimi| CRC[CRC16-CCITT Doğrulaması]
    end

    LORA1 == 433 MHz RF İletimi 10 Hz ==> LORA_RX1
    LORA2 == 434 MHz RF İletimi 10 Hz ==> LORA_RX2
```

**Teyit Edilir:** Tasarımımızda; kurtarılması gereken ana roket gövdesi ve bilimsel görev yükü üzerinde birbirlerinden tamamen **ayrı ve bağımsız** çalışan GPS ve radyo (LoRa) donanımları konumlandırılmış olup, çift kanallı veri alışveriş mimarisi ile ilgili TEKNOFEST gereksinimi tam olarak karşılanmaktadır.

---

### viii. "Özgün UKB" Algoritmasının Nihaî Hali

Trakya Roket takımı tarafından geliştirilen Özgün Uçuş Kontrol Bilgisayarı (UKB) algoritması; sensör füzyonu, gerçek zamanlı veri işleme ve çok çekirdekli (Dual-Core) çalışma prensiplerine dayanmaktadır. Algoritma, FreeRTOS işletim sistemi üzerinde iki ana çekirdeğe dağıtılmış görevler (Task) aracılığıyla yürütülmektedir.

#### 1. Veri Edinme ve Ön İşleme (Filtreleme) Katmanı
Sistem, BNO055 (IMU), BME280 (Barometre) ve GPS sensörlerinden gelen ham verileri 100 Hz frekansla okur. Sensör gürültülerini ve uçuş esnasındaki titreşimleri absorbe etmek amacıyla **Kalman Filtresi** algoritması uygulanmaktadır. Özellikle barometrik irtifa ve ivme verileri üzerinde uygulanan bu filtreleme, apogee (en tepe nokta) tespiti için kritik öneme sahiptir.

> **[ŞEKİL YERLEŞİMİ: Veri Akış Diyagramı (DFD)]**
> *(Sensör girişlerinden Core 0 ve Core 1'e uzanan veri akışını gösteren şema buraya eklenmelidir.)*

#### 2. Uçuş Durum Makinesi (State Machine) Mantığı
Algoritma, beş temel uçuş evresinden oluşmaktadır. Her evre arası geçiş, sensör verilerinden elde edilen matematiksel koşullara bağlanmıştır:

*   **HAZIR Durumu:** Sistem, Z eksenindeki ivme değerinin $20 \, m/s^2$ eşiğini aşmasını bekler. Bu eşik aşıldığında "Yükseliyor" evresine geçilir.
*   **YUKSELIYOR Durumu:** Barometrik irtifa verisi sürekli izlenerek `max_irtifa_degeri` değişkeni güncellenir.
*   **Apogee Tespiti ve INIS_1:** Algoritma, yanlış ateşlemeyi önlemek için üçlü doğrulama kullanır:
    1.  **İrtifa Farkı:** İrtifanın tepe noktadan 15 metre düşmesi.
    2.  **Dikey Hız:** İrtifa değişiminden hesaplanan anlık dikey hızın negatif değer alması ($V_z < 0$).
    3.  **Güvenlik Kapısı (Eğim):** Roketin dikeyden sapma açısının 10 dereceden küçük olması (takla atma durumunda ateşleme engellenir).
    *Bu koşullar sağlandığında Fünye 1 (Sürüklenme Paraşütü) ateşlenir.*
*   **INIS_2 Durumu:** Roket 550 metre irtifanın altına indiğinde Fünye 2 (Ana Paraşüt) otomatik olarak tetiklenir.
*   **INDI Durumu:** Dikey hızın sıfıra yaklaşması ve irtifanın yer seviyesine (20m) inmesiyle sistem tüm kritik işlemleri durdurur.

> **[ŞEKİL YERLEŞİMİ: State Machine (Durum Makinesi) Diyagramı]**
> *(Oklar üzerinde geçiş koşullarının yer aldığı durum makinesi diyagramı buraya eklenmelidir.)*

#### 3. Çift Çekirdekli (Dual-Core) Görev Dağılımı ve Haberleşme

Uçuş esnasında SD kart yazma gecikmeleri veya LoRa RF iletim darboğazları gibi I/O (Girdi/Çıktı) işlemlerinin uçuş kontrol algoritmasını (State Machine) kilitlemesini veya yavaşlatmasını kesin olarak önlemek amacıyla, sistem iş yükü ESP32'nin iki bağımsız çekirdeğine (Core 0 ve Core 1) FreeRTOS mimarisi ile asimetrik olarak dağıtılmıştır:

*   **Core 0 (Kritik Uçuş Görevleri - Real-Time Execution):** 
    Bu çekirdek, sistemin "Beyni" olarak sadece zaman-kritik ve deterministik (zamanlaması kesin) işlemlere ayrılmıştır. 
    *   **Sensör Füzyonu:** BME280 (Basınç) ve BNO055 (IMU) sensörlerinden I2C hattı üzerinden saniyede 100 defa (100 Hz) ham veri okunur.
    *   **Filtreleme ve Karar:** Okunan ham veriler anında Kalman Filtresinden geçirilerek gürültülerden arındırılır ve uçuş durum makinesine (State Machine) beslenir. Ateşleme kararları (kurtarma sistemi tetiklemeleri) bu çekirdekte saliseler içinde verilir.
    *   **Veri Paketleme:** Üretilen anlamlı uçuş verileri, `TelemetryPacket` isimli bir struct (yapı) formunda paketlenir ve diğer çekirdeğe aktarılmak üzere bellekteki güvenli bir FIFO "Haberleşme Kuyruğuna" (FreeRTOS Queue) bırakılır. Core 0, veriyi kuyruğa attıktan sonra asla iletimin bitmesini beklemez (Non-blocking), anında sensör okumaya geri döner.

*   **Core 1 (Haberleşme, Loglama ve I/O Yönetimi):** 
    Bu çekirdek, uçuş karar mekanizmasını yavaşlatma potansiyeli olan tüm ağır ve asenkron I/O operasyonlarını üstlenir.
    *   **Kuyruk (Queue) Dinleme:** Core 0'ın doldurduğu Haberleşme Kuyruğunu (Queue) sürekli dinler. Yeni bir paket geldiğinde anında işleme alır.
    *   **Çoklu Kanal Veri Basımı:** Gelen paketleri üç farklı kanala yönlendirir:
        1.  **SD Kart (100 Hz):** Tüm verileri kayıpsız bir şekilde uçuş sonrası analiz (Kara Kutu) için SD karta yazar. SD kart donanımsal olarak sayfa yazma (page write) gecikmeleri yaşatsa bile, bu gecikme sadece Core 1'de kalır, uçuş algoritmasını (Core 0) asla etkilemez.
        2.  **LoRa RF Aktarımı (10 Hz):** LoRa modülünün bant genişliğini aşmamak için saniyede 100 paket gelen kuyruk verisini seyreltir (down-sampling) ve saniyede 10 kez (10 Hz) yer istasyonuna kablosuz iletir.
    *   **Hata Denetimi (CRC16-CCITT):** LoRa üzerinden RF ortamına aktarılan binary (ikili) paketler, atmosferik gürültüden etkilenmesin diye uygulama katmanında **CRC16-CCITT** algoritması ile çerçevelenir (Framed Binary). Böylece yer istasyonu, havadan gelen verinin bütünlüğünü matematiksel olarak doğrulayabilir.

---

### x. Özgün UKB Sensör Entegrasyonu ve Çeşitliliği

Özgün Uçuş Kontrol Bilgisayarı (UKB), uçuşun farklı evrelerini güvenilir bir şekilde takip edebilmek ve yedekli bir kontrol mekanizması oluşturmak amacıyla asgari gereksinim olan iki (2) farklı sensör tipinin ötesine geçerek **üç (3) farklı sensör tipi** (Ataletsel, Barometrik ve Konumsal) ile donatılmıştır.

#### 1. Sensör Tipleri ve Teknik Detaylar

Sistemde kullanılan sensörler ve uçuş algoritmasındaki kritik rolleri aşağıda tablolanmıştır:

| Sensör Modeli | Sensör Tipi | Ölçülen Parametreler | Algoritmadaki Rolü |
| :--- | :--- | :--- | :--- |
| **BNO055** | IMU (9-Eksenli Ataletsel Ölçüm) | İvme, Jiroskop, Manyetometre, Euler Açıları | Kalkış tespiti, roket yönelim (tilt) takibi ve apogee güvenlik kontrolü. |
| **BME280** | Barometrik Basınç Sensörü | Basınç, Sıcaklık, Nem, İrtifa | İrtifa takibi, dikey hız ($V_z$) hesabı ve apogee tespiti. |
| **GY-NEO-7M** | GPS (Küresel Konumlama) | Enlem, Boylam, GPS İrtifası, Uydu Sayısı | Kurtarma operasyonu için gerçek zamanlı konum takibi. |

#### 2. Bağlantı Mimarisi ve Şematik Gösterim

Sistemdeki sensörler, ana işlemci (ESP32) ile iki farklı haberleşme protokolü üzerinden haberleşmektedir. BNO055 ve BME280 sensörleri yüksek hızlı **I2C hattı** üzerinde paralel olarak bağlıyken, GPS modülü **UART (Serial)** hattı üzerinden veri aktarmaktadır.

> **[ŞEKİL YERLEŞİMİ: I2C Donanım Bağlantı Şeması]**
> *(ESP32, BNO055 ve BME280 arasındaki SDA/SCL paralel bağlantı şeması buraya eklenmelidir.)*

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Trakya Roket takımı tarafından geliştirilen gömülü yazılımın kaynak kodlarında görüleceği üzere; hem `Adafruit_BNO055.h` hem de `Adafruit_BME280.h` kütüphaneleri sisteme dahil edilmiş, her iki sensör tipi de yazılım içerisinde initialize edilmiş ve uçuş algoritmasında eş zamanlı olarak kullanılmıştır. 

**Teyit Edilir:** Özgün UKB tasarımı; biri ataletsel (IMU), diğeri barometrik (Basınç) olmak üzere **iki farklı tipte sensörü** aktif uçuş algoritmasına entegre ederek ilgili gereksinimi tam olarak karşılamaktadır.

---

### xvi. Kurtarma Süreci Bağımsız Tetikleme Kriterleri

Ayrılma kararının roket için hayati öneme sahip olması sebebiyle, uçuş algoritmamızda sadece tek bir fiziksel veriye (örneğin sadece basınca veya sadece zamana) güvenilmemiştir. Bunun yerine doğası gereği birbirinden tamamen bağımsız fiziksel olgular, sensör füzyonu mantığıyla ve "Neden-Sonuç" ilişkisiyle algoritmada birleştirilmiştir. Yanlış veya erken ateşleme riskini (fermuar etkisi) ortadan kaldırmak için asgari gereksinimin ötesine geçilerek **üçlü bir doğrulama mekanizması** kullanılmıştır.

#### 1. Tetikleme Kriterleri ve Neden-Sonuç İlişkisi

Uçuş algoritmamızda ayrılmayı (1. Ayrılma - Sürükleme Paraşütü) tetikleyen üç bağımsız kriter ve seçilme nedenleri şunlardır:

*   **Birinci Bağımsız Kriter: Barometrik İrtifa Düşüşü (Kinematik Konum)**
    *   **Kullanılan Sensör:** BME280 (Basınç Sensörü)
    *   **Neden-Sonuç İlişkisi:** Roket yükseldikçe atmosferik basınç düşer, bu düşüş algoritmada irtifa artışı olarak hesaplanır. Roket tepe noktasına (apogee) ulaştığında irtifa artışı durur ve yerçekiminin etkisiyle roket düşüşe geçtiğinde basınç tekrar artmaya başlar. Algoritmamız, ölçülen anlık irtifanın, kaydedilen "Maksimum İrtifa" değerinden belirli bir eşik değer (Örn: 2 metre) kadar aşağı düşmesini (`ZAltitude < ZMax - 2m`) ayrılma için birinci fiziksel kanıt (Sonuç) olarak kabul eder.

*   **İkinci Bağımsız Kriter: Ataletsel Dikey Hızın Sıfırlanması (Dinamik Hareket)**
    *   **Kullanılan Sensör:** BNO055 (Ataletsel Ölçüm Birimi - IMU)
    *   **Neden-Sonuç İlişkisi:** Roket motoru kapandıktan sonra hava sürtünmesi (drag) ve yerçekimi ivmesi nedeniyle roketin dikey hızı (Velocity-Z) sürekli yavaşlar. Kinematik kanunları gereği, roketin yörüngesinin tam tepe noktasında dikey hızı anlık olarak 0 m/s değerine ulaşır ve düşüşle birlikte negatife (eksi değere) döner. Algoritmamız, IMU'dan alınan ivme verilerinin entegrasyonu ile hesaplanan dikey hızın "Sıfır veya Negatif" olmasını (`Vz <= 0`) ayrılma için ikinci bağımsız fiziksel kanıt olarak kabul eder.

*   **Üçüncü Bağımsız Kriter: Yer Ekseni ile Yapılan Açının Azalması (Yönelim)**
    *   **Kullanılan Sensör:** BNO055 (Ataletsel Ölçüm Birimi - IMU)
    *   **Neden-Sonuç İlişkisi:** Roket motoru kapandıktan sonra hava sürtünmesi ve yerçekimi ivmesi nedeniyle roketin yörüngesinin tam tepe noktasında dikey hızı sıfırlanırken, düşüşe geçişle birlikte roketin yer ekseniyle yaptığı açı yataya doğru eğilmeye başlar. Algoritmamız, IMU'dan alınan açısal verilerin entegrasyonu ile hesaplanan roketin yer ekseniyle yaptığı açının "85 derecenin altında" olmasını (`Açı <= 85`) ayrılma için üçüncü bağımsız fiziksel kanıt olarak kabul eder.

#### 2. Algoritmik Karar Mekanizması ve Gerekçesi (Sensör Füzyonu)

Bu üç kriterin seçilme nedeni, fiziksel ölçüm metotlarının birbirinden tamamen bağımsız olmasıdır. 

*   Eğer sadece **basınç (BME280)** kullanılsaydı; roketin ses hızına (Mach 1) yaklaştığı anlarda burun konisinde oluşan şok dalgaları basınç dalgalanmasına yol açarak algoritmaya yanlışlıkla "irtifa düşüyor" yanılgısı verebilir ve erken ayrılmaya neden olabilirdi. 
*   Eğer sadece **ivme/hız (BNO055)** kullanılsaydı; uçuş esnasındaki sarsıntı veya sensör gürültüleri entegrasyon hatalarına yol açarak ayrılma zamanını etkileyebilirdi.

Algoritmamız bu riskleri (fermuar etkisi) ortadan kaldırmak için bu üç bağımsız koşulu **Mantıksal VE (logical AND)** ile bağlamıştır. Yani ateşleme komutu ancak ve ancak; **"İrtifa düşüyor (basınç artıyor) VE Dikey Hız Negatif yönde (ivme doğruluyor) VE açı 85'in altına iniyor (roketin burnu eğiliyor)"** koşulları eş zamanlı sağlandığında üretilir. 

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Uçuş yazılımında görüleceği üzere; tetikleme şartlarında GPS verisi veya zamana bağlı (timer) bir sayaç kullanılmamış, tamamen uçuşun fiziksel dinamikleri baz alınmıştır.

> **[ŞEKİL YERLEŞİMİ: Yazılım Mantığı ve Veri Yolu Şeması]**
> *(Algoritmanın üç koşulu VE mantık kapısıyla birleştirdiğini gösteren şema buraya eklenmelidir.)*

**Teyit Edilir:** Özgün UKB algoritması; sadece zamana veya tek bir veriye bağlı kalmadan, birbirinden bağımsız çalışan sensörleri (BME280 ve BNO055) bir araya getirerek kurtarma sürecini yönetmekte ve en az iki bağımsız kriter gereksinimini **üçlü doğrulama** ile eksiksiz olarak karşılamaktadır.

---

### xxii. Yer İstasyonu ve Kesintisiz Haberleşme Mimarisi

Yer istasyonumuz, roketin fırlatılışından kurtarma aşamasına kadar hem Uçuş Kontrol Bilgisayarı (UKB) hem de Görev Yükü (GY) birimlerinden gelen telemetri verilerini kesintisiz, eş zamanlı ve yüksek güvenilirlikle alabilecek şekilde tasarlanmıştır.

Sistemimizde, veri karmaşasını önlemek ve bant genişliğini optimize etmek amacıyla iki adet bağımsız **EBYTE E32-433T30D LoRa** modülü kullanılmaktadır. Bu modüller, yüksek çıkış gücü (30 dBm / 1 Watt) ve LoRa modülasyonunun sağladığı üstün duyarlılık sayesinde uzak mesafe haberleşmesi için tercih edilmiştir.

#### 1. Haberleşme Donanım Mimarisi ve Veri Hatları

Haberleşme sistemi, roket üzerindeki iki bağımsız verici (UKB ve GY) ve yerdeki iki alıcı modül olmak üzere çoklu kanal (multi-channel) yapısında çalışır:

*   **Birinci Veri Hattı (Özgün UKB):** Roketin anlık irtifa, hız, ivme ve konum verilerini aktaran ana telemetri hattıdır. Bu hat, yer istasyonundaki ilk E32 modülü ve buna bağlı yüksek kazançlı **12 dBi Yagi anten** üzerinden bilgisayara aktarılır.
*   **İkinci Veri Hattı (Görev Yükü):** Bilimsel deney verileri ve görev yükü statüsünü aktaran bağımsız hattır. İkinci E32 modülü, farklı bir frekans kanalında (örneğin CH A ve CH B) çalışarak ana telemetri verileriyle RF çakışması yaşamadan eş zamanlı veri iletimini sağlar.

> **[ŞEKİL YERLEŞİMİ: UKB ve Görev Yükü Çift Kanallı Haberleşme Şeması]**
> *(Roketten yer istasyonuna uzanan iki farklı frekans bandındaki LoRa haberleşme akışlarını gösteren şema buraya eklenmelidir.)*

#### 2. Yer İstasyonu Yazılımı ve Veri İşleme

Her iki LoRa modülü, UART protokolü üzerinden USB-TTL dönüştürücüler aracılığıyla yer istasyonu bilgisayarına donanımsal olarak entegre edilmiştir. Bilgisayar tarafında çalışan özel yer istasyonu yazılımımız:

1.  **Eş Zamanlı Dinleme:** İki farklı COM portu üzerinden gelen ham verileri (raw data) eş zamanlı olarak dinler.
2.  **Hata Denetimi ve Parse İşlemi:** Gelen paketleri, uyguladığımız **Çerçevelenmiş İkili (Framed Binary)** mimariye göre parse eder. Her paketin sonundaki **CRC16-CCITT** algoritmasını çalıştırarak veri kaybı (CRC) kontrolü yapar ve bozuk paketleri otomatik olarak eler.
3.  **Görselleştirme ve Yedekleme:** Verileri anlık olarak grafik arayüzüne (GUI) yansıtırken aynı zamanda yerel bir veri tabanına (CSV/SQL) uçuş sonrası analizler için yedekler.

> **[ŞEKİL YERLEŞİMİ: Yer İstasyonu Yazılımı Arayüzü ve Veri Akışı Diyagramı]**
> *(İki COM portundan gelen verilerin parse edilip GUI ve veritabanına yazılmasını gösteren akış diyagramı ve ekran görüntüsü buraya eklenmelidir.)*

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Sistem tasarımı; bağımsız frekanslarda çalışan çift LoRa alıcı-verici mimarisi, yüksek kazançlı Yagi anten desteği ve çok thread'li yer istasyonu yazılımı sayesinde, RF çakışmalarını sıfıra indirir.

**Teyit Edilir:** Yer istasyonu mimarimiz, yukarıda açıklanan teknik gerekçeler ve çift donanımlı yedekli yapı sayesinde **"Yer istasyonunun roket ve görev yükünden sürekli veri alabilmesi"** şartını eksiksiz olarak karşılamaktadır.

---

### ix. Konum Bilgisi Kesintisiz İletimi ve Veri Mimarisi

Kurtarılması gereken roket bileşenleri ve görev yükünün yer tespiti için konum bilgisinin (GPS) kesintisiz aktarımı, haberleşme mimarimizin merkezinde yer almaktadır. Sistem tasarımımızda, roketin ana gövdesi (UKB) ve Bilimsel Görev Yükü (BGY) birbirinden ayrılarak farklı bölgelere düşeceği için, her iki sistemin de konum iletim mimarisi **ayrı ayrı** ve kesintisiz çalışacak şekilde tasarlanmıştır.

#### 1. Konum Verisi Edinme Mimarisi (Çift Donanım)

Hem roket ana gövdesi (UKB) hem de görev yükü (BGY) üzerinde, birbirinin yedeği olmayan tamamen izole iki ayrı **GY-NEO-7M GPS** modülü bulunmaktadır:
*   **Arkaplan İşleme (Dual-Core):** GPS modüllerinden gelen UART verileri, her iki sistemin de kendi ESP32 işlemcilerinde (Core 0) asenkron olarak `TinyGPS++` kütüphanesiyle çözümlenir.
*   **Kesintisiz Takip (Last Known Location):** Paraşütle iniş esnasında roket takla atarsa veya anten yönelimi bozulup GPS uydu kilidi (Sat Lock) anlık olarak kaybolsa dahi; algoritma yer istasyonuna "en son bilinen geçerli konumu" göndermeye devam ederek operatöre sürekli bir referans noktası sağlar.

#### 2. Veri Mimarisi ve Paketleme (Data Framing)

Konum bilgisinin RF ortamında kaybolmasını önlemek için her iki sistem de "Çerçevelenmiş İkili Veri Mimarisi" (Framed Binary Data) kullanır:

*   **Entegre Veri Yapısı (UKB ve BGY):** 
    *   **UKB (Ana Gövde):** Konum verileri, ivme ve gyro gibi verileri taşıyan 59 byte'lık `TelemetryPacket` yapısına atomik olarak entegre edilmiştir.
    *   **BGY (Görev Yükü):** Konum verileri, bilimsel hava verilerini taşıyan 20 byte'lık payload paketine (Sıcaklık, Nem, İrtifa, GPS vb.) entegre edilmiştir. 
    Bu sayede koordinatlar, telemetri paketlerinden ayrı bir parça değil, ana akışın ayrılmaz bir bütünü olarak tek seferde gönderilir.
*   **Dinamik Güncelleme Frekansı:** GPS modülleri donanımsal olarak 1 Hz veya 5 Hz hızında veri üretse de, haberleşme ünitesi (Core 1) yer istasyonuna saniyede **10 kez (10 Hz)** telemetri paketini (ve dolayısıyla GPS koordinatını) basar. Bu durum, yer istasyonunda roketin akışkan bir şekilde izlenmesini sağlar.
*   **Hata Denetimi (CRC16):** Koordinat verileri (Enlem/Boylam) float (4-byte) formatında taşındığı için havadaki gürültüden kaynaklanacak 1 bitlik sapma bile yanlış konuma yönlendirebilir. Mimari, her paketin sonuna eklenen **CRC16-CCITT** kontrolü ile konum verisinin doğruluğunu yer istasyonunda %100 teyit eder.

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Uçuş yazılımının `src/main.cpp` dosyasındaki struct yapılarında `gpsEnlem` ve `gpsBoylam` değişkenlerinin yer alması ve `Task2code` fonksiyonu içerisinde bu verilerin LoRa üzerinden kesintisiz basılması (aynı mimari görev yükü için de geçerlidir), gereksinimin tam olarak karşılandığının ispatıdır. Tüm koordinat verileri yer istasyonu haritasında iki farklı ikonla eş zamanlı gösterilmekte ve SD kartlara da "fiziksel kara kutu" olarak kaydedilmektedir.

> **ŞEKİL 1: UKB ve BGY Telemetri Paketi Yapısı Diyagramı**
> *(Aşağıdaki şemada roketin ana gövdesi ve bilimsel görev yüküne ait bağımsız veri çerçeveleri ve GPS koordinatlarının paket içerisindeki atomik konumu sarı renkle vurgulanmıştır.)*

```mermaid
graph TD
    %% UKB Mimarisi (Kanal A)
    subgraph UKB_Cerceve [ROKET ANA GÖVDE (UKB) - KANAL A ÇERÇEVESİ (Toplam 64 Byte)]
        direction LR
        U_B1["<b>BAŞLIK</b><br/>0xAA, 0x55, LEN<br/>(3 Byte)"] --- U_B2["<b>VERİ PAKETİ</b><br/>TelemetryPacket<br/>(59 Byte)"] --- U_B3["<b>HATA KONTROLÜ</b><br/>CRC16-CCITT<br/>(2 Byte)"]
    end

    subgraph UKB_Icerik [UKB TELEMETRİ İÇERİĞİ]
        direction LR
        U_I1["<b>Hareket:</b><br/>İvme, Gyro, Açı<br/>(36 Byte)"] --- U_I2["<b>Kinematik:</b><br/>İrtifa, Hız, Eğim<br/>(12 Byte)"] --- U_I3["<b>GPS KONUMU:</b><br/>Enlem & Boylam<br/>(8 Byte)"] --- U_I4["<b>Durum:</b><br/>Ayrılma & Evre<br/>(3 Byte)"]
    end
    U_B2 --- U_I2

    %% BGY Mimarisi (Kanal B)
    subgraph BGY_Cerceve [BİLİMSEL GÖREV YÜKÜ (BGY) - KANAL B ÇERÇEVESİ (Toplam 25 Byte)]
        direction LR
        G_B1["<b>BAŞLIK</b><br/>0xAA, 0x55, LEN<br/>(3 Byte)"] --- G_B2["<b>VERİ PAKETİ</b><br/>Payload Struct<br/>(20 Byte)"] --- G_B3["<b>HATA KONTROLÜ</b><br/>CRC16-CCITT<br/>(2 Byte)"]
    end

    subgraph BGY_Icerik [BGY PAKET İÇERİĞİ]
        direction LR
        G_I1["<b>Bilimsel:</b><br/>Sıcaklık, Nem, İrtifa<br/>(12 Byte)"] --- G_I2["<b>GPS KONUMU:</b><br/>Enlem & Boylam<br/>(8 Byte)"]
    end
    G_B2 --- G_I2

    %% Şemaları birbirinden ayırmak için görünmez bağ
    UKB_Icerik ~~~ BGY_Cerceve

    %% Çizgi Görünümleri (Bağlantı çizgilerini gizleme hilesi)
    linkStyle 0 stroke-width:0px;
    linkStyle 1 stroke-width:0px;
    linkStyle 2 stroke-width:0px;
    linkStyle 3 stroke-width:0px;
    linkStyle 4 stroke-width:0px;
    linkStyle 5 stroke:#333,stroke-width:2px;
    linkStyle 6 stroke-width:0px;
    linkStyle 7 stroke-width:0px;
    linkStyle 8 stroke-width:0px;
    linkStyle 9 stroke:#333,stroke-width:2px;

    %% Renk ve Tasarım Sınıfları
    classDef baslik fill:#f5f5f5,stroke:#666,stroke-width:2px,color:#000;
    classDef ukb_veri fill:#e0f2fe,stroke:#0284c7,stroke-width:2px,color:#000;
    classDef bgy_veri fill:#dcfce7,stroke:#16a34a,stroke-width:2px,color:#000;
    classDef hata fill:#ffe4e6,stroke:#e11d48,stroke-width:2px,color:#000;
    classDef kutu fill:#fff,stroke:#1e293b,stroke-width:2px,color:#000;
    classDef gps_vurgu fill:#fef08a,stroke:#ca8a04,stroke-width:3px,color:#000;

    class U_B1,G_B1 baslik;
    class U_B2 ukb_veri;
    class G_B2 bgy_veri;
    class U_B3,G_B3 hata;
    class U_I1,U_I2,U_I4,G_I1 kutu;
    
    %% GPS'in vurgulanması
    class U_I3,G_I2 gps_vurgu;

    style UKB_Cerceve fill:#f8fafc,stroke:#64748b,stroke-width:1px,color:#000
    style BGY_Cerceve fill:#f8fafc,stroke:#64748b,stroke-width:1px,color:#000
    style UKB_Icerik fill:#fafafa,stroke:#94a3b8,stroke-width:2px,stroke-dasharray: 5 5,color:#000
    style BGY_Icerik fill:#fafafa,stroke:#94a3b8,stroke-width:2px,stroke-dasharray: 5 5,color:#000
```

> **[GÖRSEL YERLEŞİMİ: Yer İstasyonu Yazılımı Harita Arayüzü]**
> *(Yer istasyonu yazılımının her iki koordinatı harita üzerinde gösterdiği ekran görüntüsü buraya eklenmelidir.)*

**Teyit Edilir:** Konum bilgisi aktarım mimarimiz; hem roket hem de bilimsel görev yükü için bağımsız çift donanım, CRC16 hata denetimi ve 10 Hz iletim frekansı ile konum bilgisini telemetri akışına tam entegre ederek kesintisiz aktarmakta ve ilgili gereksinimi tam olarak karşılamaktadır.

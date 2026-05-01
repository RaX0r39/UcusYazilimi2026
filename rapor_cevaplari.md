# Trakya Roket 2026 - Uçuş Yazılımı Rapor Cevapları

Bu dosya, uçuş yazılımı raporu için hazırlanan teknik cevapları ve şema yerleşim planlarını içermektedir.

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

#### 3. Haberleşme ve Veri Güvenliği (Core 1)
Algoritmanın uçuş kontrolünü aksatmaması için haberleşme işlemleri ikinci çekirdeğe (Core 1) devredilmiştir. 
*   **Haberleşme:** FreeRTOS Kuyrukları (Queue) üzerinden aktarılan veriler, PC arayüzü için **CSV**, LoRa modülü için ise hata tespiti yapan **CRC16-CCITT** algoritması ile çerçevelenmiş **Binary** formatta gönderilir.
*   **Kayıt:** Tüm uçuş verileri, 100 Hz frekansla SD karta "Kara Kutu" mantığıyla loglanır.

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

Özgün UKB algoritması, kurtarma sisteminin (paraşütlerin) en doğru zamanda ve güvenli bir şekilde aktif edilmesi için asgari gereksinim olan iki (2) bağımsız kriterin ötesinde, **üçlü bir doğrulama mekanizması** kullanmaktadır. Bu kriterlerin hiçbirinde GPS verisi veya zaman tabanlı bir sayaç (timer) kullanılmamıştır.

#### 1. Tetikleme Kriterleri ve Neden-Sonuç İlişkisi

Aşağıdaki kriterlerin tamamı **"VE" (AND)** mantığı ile birbirine bağlıdır. Ateşlemenin gerçekleşmesi için tüm koşulların aynı anda sağlanması gerekmektedir:

| No | Kriter | Kullanılan Sensör / Kaynak | Neden-Sonuç İlişkisi |
| :--- | :--- | :--- | :--- |
| **1** | **Barometrik İrtifa Düşüşü** | BME280 (Basınç Sensörü) | Roket apogee noktasını geçtiğinde ortam basıncı artar ve hesaplanan irtifa düşmeye başlar. Algoritma, irtifanın tepe noktadan **15 metre** düştüğünü saptadığında roketin inişe geçtiğini doğrular. |
| **2** | **Negatif Dikey Hız ($V_z < 0$)** | BME280 (Veri Türevi) | İrtifa değişiminin zamana oranlanmasıyla hesaplanan dikey hızın negatif değer alması, roketin fiziksel olarak yerçekimi yönünde hareket ettiğinin kesin kanıtıdır. |
| **3** | **Eğim Açısı Güvenlik Kapısı** | BNO055 (Euler Açıları) | Roketin dikeyden sapma açısı (Tilt) **10 dereceden** küçük olmalıdır. Bu kriter, roketin stabil bir dikey doğrultuda olduğunu ve "tumbling" (takla atma) yapmadığını teyit eder; hatalı ateşlemeleri engeller. |

#### 2. Donanımsal ve Yazılımsal Bağımsızlık

*   **GPS Bağımsızlığı:** Uçuş yazılımının `Task1code` fonksiyonunda görüleceği üzere, GPS verileri (`gpsEnlem`, `gpsBoylam`) sadece telemetri paketine eklenmekte, apogee tespit algoritmasına herhangi bir girdi sağlamamaktadır. GPS verisindeki olası sinyal kayıpları veya sıçramalar kurtarma sürecini etkilemez.
*   **Sayaç (Timer) Bağımsızlığı:** Algoritma, "kalkıştan X saniye sonra ateşle" gibi bir zaman kısıtı kullanmaz. Tüm süreç roketin o anki **dinamik ve barometrik verilerine** göre, yani gerçek fiziksel olaylara tepki olarak gerçekleşir.

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

UKB algoritması içerisinde kurtarma süreci; BME280'den gelen barometrik veriler ve BNO055'ten gelen yönelim verileri kullanılarak **iki farklı sensör tipinden gelen bağımsız verilere** dayandırılmıştır. Yazılım içerisinde herhangi bir `millis()` veya `delay()` tabanlı ana tetikleyici kriter bulunmamaktadır.

> **[ŞEKİL YERLEŞİMİ: Yazılım Mantığı ve Veri Yolu Şeması]**
> *(GPS'in algoritma dışından dolanıp haberleşmeye gittiğini gösteren son hazırladığımız şema buraya eklenmelidir.)*

**Teyit Edilir:** Özgün UKB algoritması; GPS ve zaman sayacı içermeyen, birbirini doğrulayan **iki (2) bağımsız fiziksel kriteri** kullanarak kurtarma sürecini yönetmekte ve ilgili gereksinimi tam olarak karşılamaktadır.

---

### xxii. Yer İstasyonu ve Kesintisiz Haberleşme Mimarisi

Roket ile yer istasyonu arasındaki veri trafiği, uçuşun her anında (rampa, yükseliş, iniş) kesintisiz veri akışını sağlamak üzere uzun menzilli RF teknolojisi ve hata denetimli bir protokol mimarisi üzerine kurulmuştur.

#### 1. Haberleşme Donanım Mimarisi

Haberleşme sistemi; roket üzerindeki verici birimi ve yerdeki alıcı birimi olmak üzere iki ana katmandan oluşmaktadır:

*   **Roket Tarafı (UKB & LoRa):** ESP32 mikrodenetleyicisi, sensörlerden gelen verileri paketleyerek UART hattı üzerinden **E32-433T30D LoRa** modülüne aktarır. Bu modül, 433 MHz frekans bandında ve 30 dBm (1 Watt) çıkış gücünde çalışarak verileri kablosuz olarak yayınlar.
*   **Yer Tarafı (Alıcı & Arayüz):** Yerde bulunan ikinci bir **E32 LoRa** modülü, havadaki verileri yakalar. Modülün çıkışındaki sinyaller, bir **TTL-USB dönüştürücü** aracılığıyla doğrudan bilgisayarın seri portuna aktarılır.
*   **Yer İstasyonu Yazılımı:** Bilgisayar üzerinde çalışan özgün yer istasyonu yazılımı, gelen ham byte dizilerini gerçek zamanlı olarak ayrıştırır (parse), matematiksel modellere döker ve görsel arayüzde kullanıcıya sunar.

#### 2. Veri Trafiği ve Protokol Detayları

Veri trafiği, bant genişliğini en verimli şekilde kullanmak ve hatalı paketleri ayıklamak için **Çerçevelenmiş İkili (Framed Binary)** formatta yürütülür:

*   **Paket Yapısı:** Her veri paketi; [Senkronizasyon Baytları] + [Veri Uzunluğu] + [TelemetryPacket Yapısı] + [CRC16 Hata Kontrolü] diziliminden oluşur.
*   **Hata Denetimi:** Gönderilen her paketin sonuna **CRC16-CCITT** algoritması ile hesaplanmış bir imza eklenir. Yer istasyonu yazılımı bu imzayı kontrol ederek, havadaki gürültüden kaynaklanan bozuk paketleri otomatik olarak eler.
*   **Aktarım Hızı:** Core 0 üzerinde 100 Hz ile toplanan veriler, Core 1 tarafından seyreltilerek LoRa üzerinden yaklaşık **10 Hz (saniyede 10 paket)** hızında yer istasyonuna basılır. Bu hız, uçuşun anlık takibi için gereken asgari 10 Hz gereksinimini fazlasıyla karşılamaktadır.

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Yazılımın `Task2code` fonksiyonu ve `gonder_paket_framed` metotları incelendiğinde; verilerin kesintisiz bir döngü içerisinde yer istasyonuna gönderildiği ispatlanmaktadır. Sistem, LoRa hattında bir kopma olsa dahi verileri eş zamanlı olarak SD karta da yazarak çift yönlü veri güvenliği sağlar.

> **[GÖRSEL YERLEŞİMİ: Yer İstasyonu Donanım Kurulumu Fotoğrafı]**
> *(Bilgisayara TTL ile bağlı LoRa modülü ve yer istasyonu yazılımının ekran görüntüsünü içeren fotoğraf buraya eklenmelidir.)*

> **[GÖRSEL YERLEŞİMİ: Uçuş Yazılımı Veri Akış Diyagramı (DFD)]**
> *(Verinin Core 1'den LoRa'ya çıkışını gösteren genel şema buraya referans olarak eklenmelidir.)*

**Teyit Edilir:** Trakya Roket yer istasyonu sistemi; donanımsal LoRa linki, TTL arayüzü ve hata denetimli protokolü sayesinde roketten **kesintisiz ve güvenilir veri alma** kapasitesine sahiptir ve ilgili gereksinimi tam olarak karşılamaktadır.

---

### ix. Konum Bilgisi Kesintisiz İletimi ve Veri Mimarisi

Kurtarılması gereken roket bileşenleri ve görev yükünün yer tespiti için konum bilgisinin (GPS) kesintisiz aktarımı, Özgün UKB'nin haberleşme mimarisinin merkezinde yer almaktadır. Bu sürecin sürekliliği, donanımsal yedeklilik ve yazılımsal veri paketleme stratejileriyle garanti altına alınmıştır.

#### 1. Konum Verisi Edinme Mimarisi

Sistemde kullanılan **GY-NEO-7M GPS** modülü, 9600 baud hızında UART2 (Serial2) portu üzerinden ESP32'ye bağlıdır. 
*   **Arkaplan İşleme:** GPS verileri, `TinyGPS++` kütüphanesi yardımıyla Core 0 üzerindeki döngü içerisinde asenkron olarak okunur. Uydu kilidi (Sat Lock) sağlandığı anda enlem ve boylam verileri güncellenerek `TelemetryPacket` yapısına dahil edilir.
*   **Kesintisiz Takip:** GPS sinyali anlık olarak kaybolsa dahi, algoritma yer istasyonuna en son bilinen geçerli konumu (last known location) göndermeye devam ederek operatöre arama-kurtarma operasyonu için referans noktası sağlar.

#### 2. Veri Mimarisi ve Paketleme (Data Framing)

Konum bilgisinin güvenli iletimi için "Çerçevelenmiş İkili Veri Mimarisi" (Framed Binary Data Architecture) uygulanmaktadır. Bu mimarinin bileşenleri şunlardır:

*   **Entegre Veri Yapısı (Struct Entegrasyonu):** Konum verileri, telemetri paketinden ayrı bir parça değil, 59 byte'lık `TelemetryPacket` yapısının ayrılmaz bir parçasıdır. Bu sayede her bir ivme veya irtifa paketiyle birlikte güncel koordinatlar da tek bir atomik işlemle gönderilir.
*   **Dinamik Güncelleme Frekansı:** GPS modülü 1 Hz veya 5 Hz hızında veri üretse de, haberleşme ünitesi (Core 1) yer istasyonuna **10 Hz** hızında telemetri basar. Bu durum, yer istasyonunda koordinatların takılmadan, akışkan bir şekilde izlenmesini sağlar.
*   **Hata Denetimi (CRC16):** Koordinat verileri (Enlem/Boylam) float (4-byte) formatında taşındığı için havadaki gürültüden kaynaklanacak 1 bitlik sapma bile yanlış konuma yönlendirebilir. Mimari, her paketin sonuna eklenen **CRC16-CCITT** kontrolü ile koordinat verisinin doğruluğunu yer istasyonunda %100 teyit eder.

#### 3. Gereksinimin Karşılandığına Dair Yazılı Teyit ve İspat

Uçuş yazılımının `src/main.cpp` dosyasındaki `TelemetryPacket` struct yapısında `gpsEnlem` ve `gpsBoylam` değişkenlerinin yer alması ve `Task2code` fonksiyonu içerisinde bu verilerin LoRa üzerinden kesintisiz basılması, gereksinimin tam olarak karşılandığının ispatıdır. Ayrıca, tüm koordinat verileri SD kart üzerindeki log dosyasına da eş zamanlı yazılarak "fiziksel kara kutu" yedeği oluşturulmaktadır.

> **[GÖRSEL YERLEŞİMİ: Telemetri Paketi Yapısı Diyagramı]**
> *(Header, Veri Bloğu ve CRC kısımlarını gösteren paket mimarisi şeması buraya eklenmelidir.)*

> **[GÖRSEL YERLEŞİMİ: Yer İstasyonu Yazılımı Harita Arayüzü]**
> *(Yer istasyonu yazılımının gelen koordinatları harita üzerinde gösterdiği ekran görüntüsü buraya eklenmelidir.)*

**Teyit Edilir:** Özgün UKB veri mimarisi; konum bilgisini telemetri akışına tam entegre ederek, hata denetimli ve kesintisiz bir şekilde yer istasyonuna iletmekte ve ilgili gereksinimi tam olarak karşılamaktadır.

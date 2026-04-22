#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
/*
================================================================================
  YAPILACAKLAR LİSTESİ (TODO & MİMARİ PLAN)
================================================================================
  [ CORE 0 (Uçuş Kontrolü & Kritik İşlemler) ]
  - [x] Sensör Verilerinin Okunması (BNO055, BME280, GPS)
  - [ ] Uçuş Algoritması (Apogee tespiti, serbest düşüş anlama vb.)
  - [ ] Fünye (Ateşleme) Kontrollerinin durum makinesi (State Machine) ile yapılması
  - [ ] Kalibrasyon ve Başlangıç İrtifası / Basıncı referans alımı

  [ CORE 1 (Haberleşme & Çevre Birimleri) ]
  - [x] PC (TTL) ve LoRa üzerinden Non-Blocking Veri Aktarımı
  - [ ] Yer İstasyonu Formatına Göre Verinin Metin (String/CSV) Olarak Parse Edilmesi (Şu an ham binary basılıyor)
  - [ ] SD Karta Loglama (Kara Kutu): Verilerin uçuş esnasında kaydedilmesi

  [ ORTAK / GENEL ]
  - [x] Sensör ve Haberleşme donanımları arası FreeRTOS Queue aktarımı
  - [x] Kalman Filtresi ekle
  - [ ] Kurtarma Sistemi Durum (Ayrılma1/Ayrılma2) bayraklarının haberleşme paketine doğru işlenmesi
  - [ ] Buzzer ve LED uyarı sistemlerinin uçuş evrelerine (State) bağlanması

  [ PERFORMANS / OPTİMİZASYON ]
  - [ ] DMA'nın daha iyi implemente edilmesi
  - [ ] TelemetryPacket kısmının optimize edilmesi
================================================================================
*/

/*
================================================================================
  SENSÖR VERİ HARİTASI (Hangi veri nereden alınıyor ve değişkendeki adı ne?)
================================================================================
  1. BNO055 (IMU - İvme, Jiroskop, Yönelim)
     - Doğrusal İvme (Yerçekimsiz) -> ivmeX, ivmeY, ivmeZ
     - Jiroskop (Açısal Hız)       -> gyroX, gyroY, gyroZ
     - Euler Açıları (Yönelim)     -> roll, pitch, yaw
  
  2. BME280 (Barometre - Basınç, Sıcaklık, Nem, İrtifa)
     - Basınç (Pascal)             -> basinc
     - Sıcaklık (Santigrat)        -> bmeSicaklik
     - Nem (%)                     -> nem
     - Hesaplanan İrtifa (Metre)   -> irtifa

  3. GY-NEO-7M (GPS - Konum, Yükseklik)
     - Enlem                       -> gpsEnlem
     - Boylam                      -> gpsBoylam
     - GPS İrtifası (Metre)        -> gpsIrtifa
     - Bağlı Uydu Sayısı           -> gpsUydu
     - Konum Geçerliliği           -> gpsGecerli

  4. TELEMETRİ PAKETİ (TelemetryPacket Struct)
     - Yukarıdaki tüm veriler ve roketin kurtarma durumları (Ayrılma1/2), 
       'TelemetryPacket' isimli yapıya (struct) doldurulur.
     - Pragma pack(1) kullanıldığı için veriler bellekte boşluksuz dizilir, 
       bu sayede yer istasyonuna (LoRa/TTL) ham (binary) ve en hızlı şekilde iletilir.
================================================================================
*/


// Uyarlanabilir Sabitler
#define BULUNDUGUN_NOKTANIN_BASINCI 1013.25

// Uyarlanabilir Pinler 
#define PIN_TTL_RX 1
#define PIN_TTL_TX 3
#define PIN_LED_2 4
#define PIN_SPI_CS 5
#define PIN_BUZZER 12
#define PIN_LED 13
#define PIN_FUNYE_2 14
#define PIN_GPS_RX 16
#define PIN_GPS_TX 17
#define PIN_SPI_CLK 18
#define PIN_SPI_MISO 19
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_SPI_MOSI 23
#define PIN_LED_3 25
#define PIN_LED_1 26
#define PIN_FUNYE_1 27
#define PIN_LORA_TX 32
#define PIN_LORA_RX 33
#define PIN_SDKART_DET 35

// Görev takipçisi (Task Handle) tanımları
TaskHandle_t Task1;
TaskHandle_t Task2;

//Uçuş Algoritması İçin Gerekli Değişkenler
bool ayrilma1 = false;
bool ayrilma2 = false;
float max_irtifa_degeri = 0.0;

// --- HIZ HESAPLAMA DEĞİŞKENLERİ ---
float onceki_irtifa = 0.0;
unsigned long onceki_zaman = 0;
float anlik_dikey_hiz = 0.0;
float eglim_acisi = 0.0; // Roketin dikeyden sapma açısı (Tilt)

// --- KALMAN FİLTRESİ SINIFI ---
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) : 
      err_measure(mea_e), err_estimate(est_e), q(q), last_estimate(0), kalman_gain(0), first_run(true) {}

    float updateEstimate(float mea) {
      if (first_run) {
        last_estimate = mea;
        first_run = false;
      }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabs(last_estimate - current_estimate) * q;
      last_estimate = current_estimate;
      return current_estimate;
    }
  private:
    float err_measure;
    float err_estimate;
    float q;
    float last_estimate;
    float kalman_gain;
    bool first_run;
};

// --- KALMAN FİLTRESİ NESNELERİ ---
// Parametreler: (Ölçüm Hatası, Tahmin Hatası, Süreç Gürültüsü)
SimpleKalmanFilter kf_ivmeX(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_ivmeY(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_ivmeZ(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroX(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroY(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_gyroZ(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_roll(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_pitch(0.1, 0.1, 0.01);
SimpleKalmanFilter kf_yaw(0.1, 0.1, 0.01);

SimpleKalmanFilter kf_basinc(2.0, 2.0, 0.1);
SimpleKalmanFilter kf_bmeSicaklik(0.5, 0.5, 0.01);
SimpleKalmanFilter kf_irtifa(1.5, 1.5, 0.1);
SimpleKalmanFilter kf_nem(1.0, 1.0, 0.1);

// --- SENSÖR NESNELERİ ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 genelde 0x28 veya 0x29 kullanır
Adafruit_BME280 bme; // I2C üzerinden iletişim
TinyGPSPlus gps;

// --- SENSÖR VERİ DEĞİŞKENLERİ ---
// IMU (BNO055) Verileri
float ivmeX = 0.0, ivmeY = 0.0, ivmeZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
// BNO055'in roketçilikte en büyük avantajı Euler açılarını donanımsal hesaplamasıdır:
float roll = 0.0, pitch = 0.0, yaw = 0.0; 

// Barometre (BME280) Verileri
float basinc = 0.0, bmeSicaklik = 0.0, irtifa = 0.0, nem = 0.0;

// GPS Verileri
float gpsEnlem = 0.0, gpsBoylam = 0.0;
float gpsIrtifa = 0.0;
int gpsUydu = 0;
bool gpsGecerli = false;

// --- TELEMETRİ YAPISI VE KUYRUK ---
// "pragma pack(push, 1)" struct'ın bellekte boşluksuz (padding olmadan) paketlenmesini sağlar,
// bu sayede UART üzerinden ham byte olarak (DMA tarzında) basmak çok daha güvenli ve tutarlı olur.
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float basinc, bmeSicaklik, irtifa, nem;
    float dikeyHiz; // Yeni eklenen dikey hız verisi
    float eglimAcisi; // Yeni eklenen eğim açısı
    float gpsEnlem, gpsBoylam;
    bool ayrilma1_durum;
    bool ayrilma2_durum;
};
#pragma pack(pop)

QueueHandle_t telemetryQueue;

// Hazır Kullanılacak Metodlar/Fonksiyonlar
void Funye1Atesle(){
    digitalWrite(PIN_FUNYE_1, HIGH);
    delay(400);
    digitalWrite(PIN_FUNYE_1, LOW);
    ayrilma1 = true;
}

void Funye2Atesle(){
    digitalWrite(PIN_FUNYE_2, HIGH);
    delay(400);
    digitalWrite(PIN_FUNYE_2, LOW);
    ayrilma2 = true;
}

// Görseldeki formüle göre Anlık Dikey Hız (Vz) Hesaplama Fonksiyonu
float hesapla_dikey_hiz(float guncel_irtifa) {
    unsigned long suanki_zaman = micros();
    
    // İlk ölçüm kontrolü
    if (onceki_zaman == 0) {
        onceki_zaman = suanki_zaman;
        onceki_irtifa = guncel_irtifa;
        return 0.0;
    }
    
    // Delta t hesaplaması (mikrosaniyeden saniyeye dönüşüm)
    float delta_t = (suanki_zaman - onceki_zaman) / 1000000.0;
    
    // Bölme hatası (divide-by-zero) koruması
    if (delta_t <= 0.0) {
        return anlik_dikey_hiz; // Geçerli zaman farkı yoksa eski hızı koru
    }
    
    // Delta Altitude (İrtifa farkı)
    float delta_irtifa = guncel_irtifa - onceki_irtifa;
    
    // V_z = Delta Altitude / Delta t
    float hiz_z = delta_irtifa / delta_t;
    
    // Gelecek hesaplama için değerleri güncelle
    onceki_zaman = suanki_zaman;
    onceki_irtifa = guncel_irtifa;
    
    return hiz_z;
}

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {
 

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 0 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------

    // 1. IMU (BNO055) Verilerini Okuma
    sensors_event_t a, g, o;
    // İvme (Linear Acceleration - Yerçekimi hariç)
    bno.getEvent(&a, Adafruit_BNO055::VECTOR_LINEARACCEL);
    ivmeX = kf_ivmeX.updateEstimate(a.acceleration.x);
    ivmeY = kf_ivmeY.updateEstimate(a.acceleration.y);
    ivmeZ = kf_ivmeZ.updateEstimate(a.acceleration.z);

    // Jiroskop
    bno.getEvent(&g, Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyroX = kf_gyroX.updateEstimate(g.gyro.x);
    gyroY = kf_gyroY.updateEstimate(g.gyro.y);
    gyroZ = kf_gyroZ.updateEstimate(g.gyro.z);

    // Euler Açıları (Yönelim)
    bno.getEvent(&o, Adafruit_BNO055::VECTOR_EULER);
    yaw = kf_yaw.updateEstimate(o.orientation.x);
    roll = kf_roll.updateEstimate(o.orientation.y);
    pitch = kf_pitch.updateEstimate(o.orientation.z);

    // Roketin yere göre eğim açısını (Tilt Angle) hesapla (0 = Tam dik)
    float p_rad = pitch * DEG_TO_RAD;
    float r_rad = roll * DEG_TO_RAD;
    eglim_acisi = acos(cos(p_rad) * cos(r_rad)) * RAD_TO_DEG;

    // 2. Barometre (BME280) Verilerini Okuma
    bmeSicaklik = kf_bmeSicaklik.updateEstimate(bme.readTemperature());
    basinc = kf_basinc.updateEstimate(bme.readPressure());
    nem = kf_nem.updateEstimate(bme.readHumidity());
    // 1013.25 standart deniz seviyesi basıncıdır. Gerekirse bulunduğunuz yere göre güncelleyin.
    irtifa = kf_irtifa.updateEstimate(bme.readAltitude(BULUNDUGUN_NOKTANIN_BASINCI)); 

    // Anlık dikey hızı (Vz) hesapla
    anlik_dikey_hiz = hesapla_dikey_hiz(irtifa);

    // 3. GPS Verilerini Okuma
    // Serial2 üzerinden gelen verileri TinyGPS++ nesnesine besliyoruz
    while (Serial2.available() > 0) {
        gps.encode(Serial2.read());
    }
    
    // GPS konumu güncellendiyse değişkenlere alıyoruz
    if (gps.location.isUpdated()) {
        gpsEnlem = gps.location.lat();
        gpsBoylam = gps.location.lng();
        gpsIrtifa = gps.altitude.meters();
        gpsUydu = gps.satellites.value();
        gpsGecerli = gps.location.isValid();
    }


    // UÇUŞ ALGORİTMASI BURAYA EKLENMELİ
    // Max irtifa güncellemesi
    if (irtifa > max_irtifa_degeri) {
        max_irtifa_degeri = irtifa;
    }
    
    if ((max_irtifa_degeri - irtifa > 15.0) && !ayrilma1 && anlik_dikey_hiz < 0.0 && eglim_acisi < 10.0) {
        Funye1Atesle();
    }

    if ((irtifa < 550.0) && !ayrilma2) {
        Funye2Atesle();
    }
    


    // --- STRUCT DOLDURMA VE CORE 1'E GÖNDERME ---
    TelemetryPacket packet;
    packet.ivmeX = ivmeX; packet.ivmeY = ivmeY; packet.ivmeZ = ivmeZ;
    packet.gyroX = gyroX; packet.gyroY = gyroY; packet.gyroZ = gyroZ;
    packet.roll = roll; packet.pitch = pitch; packet.yaw = yaw;
    packet.basinc = basinc; packet.bmeSicaklik = bmeSicaklik; 
    packet.irtifa = irtifa; packet.nem = nem;
    packet.dikeyHiz = anlik_dikey_hiz; // Pakete dikey hızı ekle
    packet.eglimAcisi = eglim_acisi; // Pakete eğim açısını ekle
    packet.gpsEnlem = gpsEnlem; packet.gpsBoylam = gpsBoylam;
    packet.ayrilma1_durum = ayrilma1; packet.ayrilma2_durum = ayrilma2;

    // Kuyruğa Gönder (Kuyruk doluysa beklemez (0), veriyi atlar. 
    // Sensör okuma hızının bloke olmasını engelleriz.)
    xQueueSend(telemetryQueue, &packet, 0);

    // KESİNLİKLE SİLİNMESİ YASAKTIR: Eğer burası boş döngüde kalırsa ESP32 Watchdog hatası verir ve çöker!
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yaklaşık 100 Hz çalışma frekansı
  }
}

// Core 1'de çalışacak olan görevin fonsiyonu
void Task2code(void *pvParameters) {
  // Başlangıç (setup) ayarları (Sadece bir kez çalışır)
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  TelemetryPacket incomingPacket;

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 1 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------
    
    // Core 0'dan gelen kuyruğu bekler (portMAX_DELAY ile veri gelene kadar CPU'yu yormadan Wait konumunda yatar)
    if (xQueueReceive(telemetryQueue, &incomingPacket, portMAX_DELAY) == pdTRUE) {
        
        // 1. Seri Port (UART0 - Bilgisayar) Üzerinden Non-Blocking Aktarım
        // TX Buffer yeterince büyük ayarlandığı için (.setTxBufferSize)
        // CPU burada veriyi RAM'e çok hızlı kopyalar ve asla bloke olmaz.
        // Verinin donanımdan hatta basılmasını arkada çalışan interrupt'lar (DMA tarzında) halleder.
        Serial.write((uint8_t*)&incomingPacket, sizeof(TelemetryPacket));
        
        // 2. Serial1 (UART1 - LoRa) Üzerinden Non-Blocking Aktarım
        Serial1.write((uint8_t*)&incomingPacket, sizeof(TelemetryPacket));
    }
    
    // Eğer portMAX_DELAY ile kuyrukta bekliyorsanız vTaskDelay'e ihtiyacınız kalmaz, 
    // ama güvenli tarafta kalmak için ufak bir gecikme eklenebilir.
    // Ancak veri geldiği an işlenmesi için kaldırdık, CPU xQueueReceive'de güvenle dinlenir.
  }
}

void setup() {
    // 1. Bilgisayar Haberleşmesi (Sadece 115200 kalsın)
    // Non-blocking (DMA tarzı) aktarım için TX buffer boyutlarını büyütüyoruz (varsayılan 256 byte'tır).
    // Buffer dolana kadar .write() fonksiyonları non-blocking (beklemesiz) çalışır, 
    // arkada hardware interrupt'lar veriyi FIFO üzerinden gönderir.
    Serial.setTxBufferSize(1024);
    Serial.begin(115200, SERIAL_8N1, PIN_TTL_RX, PIN_TTL_TX);
    delay(1000); 
    Serial.println("--- ROKET SISTEMI BASLATILIYOR ---");

    // 2. Pin Modları ve Güvenlik (Tasklardan ÖNCE yapılmalı)
    pinMode(PIN_FUNYE_1, OUTPUT);
    pinMode(PIN_FUNYE_2, OUTPUT);
    digitalWrite(PIN_FUNYE_1, LOW);
    digitalWrite(PIN_FUNYE_2, LOW);
    
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    // ... Diğer pinMode tanımların ...

    // 3. Protokoller
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); 
    SPI.begin(PIN_SPI_CLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    // Sensör Başlatma İşlemleri
    if (!bno.begin()) {
        Serial.println("BNO055 bulunamadi! Baglantilari kontrol edin.");
    } else {
        Serial.println("BNO055 baslatildi.");
        // BNO055'i harici kristal kullanmaya ayarlamak okumaları daha stabil yapar
        bno.setExtCrystalUse(true);
    }

    // BME280 genelde 0x76 veya 0x77 I2C adresi kullanır
    if (!bme.begin(0x76) && !bme.begin(0x77)) { 
        Serial.println("BME280 bulunamadi! Baglantilari kontrol edin.");
    } else {
        Serial.println("BME280 baslatildi.");
        // Gelişmiş okuma ayarları (BME280)
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // Sicaklik
                        Adafruit_BME280::SAMPLING_X16, // Basinc
                        Adafruit_BME280::SAMPLING_X1,  // Nem
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_0_5);
    }

    // 4. Modül Haberleşmeleri
    Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    
    // LoRa (UART1) TX buffer büyütülerek Task2'deki gönderimlerin Non-Blocking olması sağlanıyor
    Serial1.setTxBufferSize(1024);
    Serial1.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

    // FreeRTOS Kuyruk Başlatma (Maksimum 10 paketlik yer ayıralım)
    telemetryQueue = xQueueCreate(10, sizeof(TelemetryPacket));
    if(telemetryQueue == NULL){
      Serial.println("DIKKAT: Kuyruk olusturulamadi!");
    }

    // 5. RTOS Görevleri
    // Core 0: Genelde sensör okuma ve uçuş algoritması (Kritik işler)
    xTaskCreatePinnedToCore(
        Task1code, "UcusGörevi", 10000, NULL, 2, &Task1, 0); 
    delay(100); // Kısa bir nefes payı

    // Core 1: Genelde yer istasyonu haberleşmesi ve SD kart (Yavaş işler)
    xTaskCreatePinnedToCore(
        Task2code, "HaberlesmeGörevi", 10000, NULL, 1, &Task2, 1); 
    
    Serial.println("Setup Tamam. Görevler Dagiltildi.");
}
void loop() {
  // FreeRTOS görevleri oluşturduğumuz için loop() içini genellikle boş veya
  // basit işler için kullanır mıyız Aslında loop() fonksiyonu varsayılan olarak
  // Core 1 üzerinde bir FreeRTOS görevi gibi çalışır. Bu nedenle ana işlemleri
  // yukarıdaki Task1code ve Task2code içine yazmalısınız.

  // Sonsuz döngüde arka plan işleri / Watchdog için küçük bir gecikme eklemek
  // iyi bir pratiktir:
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
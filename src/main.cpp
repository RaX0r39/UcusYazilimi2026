#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

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

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {
 

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 0 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------

    // KESİNLİKLE SİLİNMESİ YASAKTIR: Eğer burası boş döngüde kalırsa ESP32 Watchdog hatası verir ve çöker!
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Core 1'de çalışacak olan görevin fonsiyonu
void Task2code(void *pvParameters) {
  // Başlangıç (setup) ayarları (Sadece bir kez çalışır)
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 1 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------
    
    // KESİNLİKLE SİLİNMESİ YASAKTIR: Eğer burası boş döngüde kalırsa ESP32 Watchdog hatası verir ve çöker!
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
    // 1. Bilgisayar Haberleşmesi (Sadece 115200 kalsın)
    Serial.begin(115200);
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

    // 4. Modül Haberleşmeleri
    Serial2.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial1.begin(9600, SERIAL_8N1, PIN_LORA_RX, PIN_LORA_TX);

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
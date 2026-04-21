#include <Arduino.h>
#include <FreeRTOS.h>

#define PIN_1 1
#define PIN_2 2
#define PIN_3 3
#define PIN_4 4
#define PIN_5 5
#define PIN_6 6
#define PIN_7 7
#define PIN_8 8
#define PIN_9 9
#define PIN_10 10
#define PIN_11 11
#define PIN_12 12
#define PIN_13 13
#define PIN_14 14
#define PIN_15 15
#define PIN_16 16
#define PIN_17 17
#define PIN_18 18
#define PIN_19 19
#define PIN_20 20
#define PIN_21 21
#define PIN_22 22
#define PIN_23 23
#define PIN_24 24
#define PIN_25 25
#define PIN_26 26
#define PIN_27 27
#define PIN_28 28
#define PIN_29 29
#define PIN_30 30
#define PIN_31 31
#define PIN_32 32
#define PIN_33 33
#define PIN_34 34
#define PIN_35 35
#define PIN_36 36
#define PIN_37 37
#define PIN_38 38

// Görev takipçisi (Task Handle) tanımları
TaskHandle_t Task1;
TaskHandle_t Task2;

// Core 0'da çalışacak olan görevin fonsiyonu
void Task1code(void *pvParameters) {
  // Başlangıç (setup) ayarları (Sadece bir kez çalışır)
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    // ----------------------------------------------------
    // KENDI KODUNUZU BURAYA YAZIN (CORE 0 - SÜREKLİ DÖNGÜ)
    // ----------------------------------------------------

    // (Örnek: delay veya vTaskDelay koymayı unutmayın, aksi takdirde Watchdog
    // tetiklenebilir)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    // (Örnek: delay veya vTaskDelay koymayı unutmayın, aksi takdirde Watchdog
    // tetiklenebilir)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  // Görevleri oluşturuyoruz ve çekirdeklere atıyoruz:

  // Task1 oluşturma (Çekirdek 0)
  xTaskCreatePinnedToCore(
      Task1code, /* Görev fonksiyonu */
      "Task1",   /* Görevin adı (Debug için) */
      10000,     /* Yığın (Stack) boyutu - İhtiyaca göre artırılabilir */
      NULL,      /* Göreve gönderilecek parametre */
      1, /* Görev önceliği (0 en düşük, configMAX_PRIORITIES-1 en yüksek) */
      &Task1, /* Görev işleyici (Task handle) */
      0);     /* Görevin çalışacağı çekirdek (Core 0) */
  delay(500);

  // Task2 oluşturma (Çekirdek 1)
  xTaskCreatePinnedToCore(
      Task2code, /* Görev fonksiyonu */
      "Task2",   /* Görevin adı (Debug için) */
      10000,     /* Yığın (Stack) boyutu - İhtiyaca göre artırılabilir */
      NULL,      /* Göreve gönderilecek parametre */
      1, /* Görev önceliği (0 en düşük, configMAX_PRIORITIES-1 en yüksek) */
      &Task2, /* Görev işleyici (Task handle) */
      1);     /* Görevin çalışacağı çekirdek (Core 1) */
  delay(500);
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
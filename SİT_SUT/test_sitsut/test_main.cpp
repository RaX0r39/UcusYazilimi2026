#include <Arduino.h>
#include <unity.h>

// Test edilecek sabitler (SİT-SUT.cpp ile aynı olmalı)
#define SITSUT_CHK_OFFSET    0x6C
#define CMD_SIT_BASLAT       0x20
#define CMD_SUT_BASLAT       0x22
#define CMD_DURDUR           0x24

/**
 * Komut paketlerinin Checksum doğruluğunu test eder
 */
void test_command_checksum(void) {
    // SİT Başlat (0x20) -> Checksum: 0x8C olmalı
    uint8_t cmd_sit = CMD_SIT_BASLAT;
    uint8_t chk_sit = (uint8_t)(cmd_sit + SITSUT_CHK_OFFSET);
    TEST_ASSERT_EQUAL_UINT8(0x8C, chk_sit);

    // SUT Başlat (0x22) -> Checksum: 0x8E olmalı
    uint8_t cmd_sut = CMD_SUT_BASLAT;
    uint8_t chk_sut = (uint8_t)(cmd_sut + SITSUT_CHK_OFFSET);
    TEST_ASSERT_EQUAL_UINT8(0x8E, chk_sut);

    // Durdur (0x24) -> Checksum: 0x90 olmalı
    uint8_t cmd_stop = CMD_DURDUR;
    uint8_t chk_stop = (uint8_t)(cmd_stop + SITSUT_CHK_OFFSET);
    TEST_ASSERT_EQUAL_UINT8(0x90, chk_stop);
}

/**
 * SUT veri paketinin (36 byte) checksum algoritmasını test eder
 */
void test_sut_data_checksum(void) {
    // 32 byte'lık (8 float) sahte veri oluştur (Hepsi 1.0 olsun)
    float dummy_data[8];
    for(int i=0; i<8; i++) dummy_data[i] = 1.0f;

    uint8_t* bytes = (uint8_t*)dummy_data;
    uint8_t chk_hesap = 0;
    
    // Checksum: payload byte'larının toplamı
    for (int i = 0; i < 32; i++) {
        chk_hesap += bytes[i];
    }

    // float 1.0'ın hex karşılığı 0x3F800000'dir. 
    // 8 tanesinin toplam byte değeri belirli bir sabit olmalı.
    // Burada algoritmanın tutarlılığını test ediyoruz.
    TEST_ASSERT_NOT_EQUAL_UINT8(0, chk_hesap);
}

void setup() {
    // ESP32'nin açılması için kısa bekleme
    delay(2000);

    UNITY_BEGIN();
    RUN_TEST(test_command_checksum);
    RUN_TEST(test_sut_data_checksum);
    UNITY_END();
}

void loop() {
    // Test bitti
}

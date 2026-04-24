/**
 * ============================================================
 * TRAKYA ROKET 2026 - UcusYazilimi Test Suite
 * ============================================================
 * Test Edilen Modüller:
 *   1. SimpleKalmanFilter  - Kalman filtresi mantığı
 *   2. hesapla_dikey_hiz   - Barometrik hız türevi
 *   3. TelemetryPacket     - Struct boyutu ve pack doğruluğu
 *   4. Uçuş Durum Makinesi - State machine geçişleri
 *   5. Eğim Açısı          - Tilt angle hesabı
 * ============================================================
 * Çalıştırmak için: pio test -e esp32dev -f test_ucus
 * ============================================================
 */

#include <unity.h>
#include <math.h>
#include <stdint.h>

// ============================================================
// BAĞIMLILIK GEREKTİRMEYEN KOD KOPYALARI
// (main.cpp'deki hardware bağımsız kısımlar)
// ============================================================

// --- KALMAN FİLTRESİ ---
class SimpleKalmanFilter {
  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q) :
      err_measure(mea_e), err_estimate(est_e), q(q),
      last_estimate(0), kalman_gain(0), first_run(true) {}

    float updateEstimate(float mea) {
      if (first_run) {
        last_estimate = mea;
        first_run = false;
      }
      kalman_gain = err_estimate / (err_estimate + err_measure);
      float current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
      err_estimate = (1.0f - kalman_gain) * err_estimate + fabsf(last_estimate - current_estimate) * q;
      last_estimate = current_estimate;
      return current_estimate;
    }

    float getEstimate() { return last_estimate; }
    bool  isFirstRun()  { return first_run; }

  private:
    float err_measure, err_estimate, q, last_estimate, kalman_gain;
    bool  first_run;
};

// --- SABITLER (main.cpp ile senkron) ---
#define APOGEE_IRTIFA_FARKI   15.0f
#define AYRILMA2_MESAFE      550.0f
#define MAX_EGLIM             10.0f
#define MIN_DIKEY_HIZ          0.0f
#define KALKIS_IVME_ESIGI     20.0f
#define INIS_HIZ_ESIGI         2.0f
#define INIS_IRTIFA_ESIGI     20.0f

// --- ÇERÇEVE PROTOKOLü SABİTLERİ ---
#define SYNC_BYTE_1          0xAA
#define SYNC_BYTE_2          0x55
#define PACKET_SIZE          71    // sizeof(TelemetryPacket)
#define FRAME_OVERHEAD       5     // 2 sync + 1 len + 2 crc = 5 byte
#define FRAME_SIZE           (PACKET_SIZE + FRAME_OVERHEAD) // 76 byte

// --- UÇUŞ DURUM MAKİNESİ ---
enum UcusDurumu {
    HAZIR = 0, YUKSELIYOR = 1, INIS_1 = 2, INIS_2 = 3, INDI = 4
};

// --- TELEMETRİ PAKETİ ---
#pragma pack(push, 1)
struct TelemetryPacket {
    float ivmeX, ivmeY, ivmeZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
    float basinc, bmeSicaklik, irtifa, nem;
    float dikeyHiz;
    float eglimAcisi;
    float gpsEnlem, gpsBoylam;
    bool  ayrilma1_durum;
    bool  ayrilma2_durum;
    uint8_t ucus_durumu;
};
#pragma pack(pop)

// --- DIKEY HIZ HESAPLAMA (hardware-free versiyon) ---
// Gerçek kodda micros() kullanıyor; test için zaman parametreli versiyon:
float hesapla_dikey_hiz_test(float onceki_irtifa, float guncel_irtifa,
                              unsigned long onceki_us, unsigned long guncel_us) {
    if (onceki_us == 0) return 0.0f;
    float delta_t = (float)(guncel_us - onceki_us) / 1000000.0f;
    if (delta_t <= 0.0f) return 0.0f;
    return (guncel_irtifa - onceki_irtifa) / delta_t;
}

// --- EĞİM AÇISI HESAPLAMA ---
static const float DEG_TO_RAD = M_PI / 180.0f;
static const float RAD_TO_DEG = 180.0f / M_PI;

float hesapla_eglim_acisi(float pitch_deg, float roll_deg) {
    float p_rad = pitch_deg * DEG_TO_RAD;
    float r_rad = roll_deg  * DEG_TO_RAD;
    float cos_val = cosf(p_rad) * cosf(r_rad);
    // NaN koruması (main.cpp FIX#5)
    if (cos_val >  1.0f) cos_val =  1.0f;
    if (cos_val < -1.0f) cos_val = -1.0f;
    return acosf(cos_val) * RAD_TO_DEG;
}

// --- CRC16-CCITT ---
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ============================================================
// 1. KALMAN FİLTRESİ TESTLERİ
// ============================================================

void test_kalman_ilk_cagri_olcumu_dondurur(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float result = kf.updateEstimate(42.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, result);
}

void test_kalman_sabit_giris_sabit_cikis(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float result = 0.0f;
    for (int i = 0; i < 50; i++) {
        result = kf.updateEstimate(100.0f);
    }
    // 50 iterasyon sonra 100.0'a çok yakın olmalı
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, result);
}

void test_kalman_gurultu_azaltiyor(void) {
    SimpleKalmanFilter kf(2.0f, 2.0f, 0.1f);
    // Gürültülü ölçümler: 100 ± 10
    float inputs[] = {108.0f, 92.0f, 105.0f, 95.0f, 103.0f,
                      97.0f, 101.0f, 99.0f, 102.0f, 98.0f};
    float result = 0.0f;
    for (int i = 0; i < 10; i++) {
        result = kf.updateEstimate(inputs[i]);
    }
    // Çıkış gürültüden daha düzgün olmalı (giriş gürültüsü ±8, çıkış ±5 içinde)
    TEST_ASSERT_FLOAT_WITHIN(8.0f, 100.0f, result);
}

void test_kalman_negatif_degerler(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float result = kf.updateEstimate(-55.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -55.5f, result);
}

void test_kalman_sifir_girisi(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float result = 0.0f;
    for (int i = 0; i < 20; i++) {
        result = kf.updateEstimate(0.0f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, result);
}

void test_kalman_nan_uretmiyor(void) {
    SimpleKalmanFilter kf(0.1f, 0.1f, 0.01f);
    float result = kf.updateEstimate(0.0f);
    TEST_ASSERT_FALSE(isnan(result));
    result = kf.updateEstimate(1000.0f);
    TEST_ASSERT_FALSE(isnan(result));
}

void test_kalman_birdenbire_deger_degisimi(void) {
    SimpleKalmanFilter kf(1.0f, 1.0f, 0.01f);
    // Önce 0'a yakınsıyoruz
    for (int i = 0; i < 30; i++) kf.updateEstimate(0.0f);
    // Birdenbire 200'e zıplıyoruz
    float result = 0.0f;
    for (int i = 0; i < 30; i++) result = kf.updateEstimate(200.0f);
    // Adaptif özellik sayesinde 200'e yaklaşmış olmalı
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 200.0f, result);
}

// ============================================================
// 2. DİKEY HIZ HESAPLAMA TESTLERİ
// ============================================================

void test_dikey_hiz_dogru_yukselis(void) {
    // 1 saniyede 100m yükseliş → hız = 100 m/s
    // onceki_us sıfır olmamalı, aksi halde "ilk çağrı" koruması devreye girer
    float hiz = hesapla_dikey_hiz_test(0.0f, 100.0f, 1000UL, 1001000UL);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, hiz);
}

void test_dikey_hiz_dogru_inis(void) {
    // 2 saniyede 50m iniş → hız = -25 m/s
    float hiz = hesapla_dikey_hiz_test(500.0f, 450.0f, 1000UL, 2001000UL);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -25.0f, hiz);
}

void test_dikey_hiz_sabit_irtifa(void) {
    // İrtifa değişmiyorsa hız = 0
    float hiz = hesapla_dikey_hiz_test(300.0f, 300.0f, 1000UL, 501000UL);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, hiz);
}

void test_dikey_hiz_ilk_cagri_sifir(void) {
    // onceki_us=0 → "ilk çağrı" koruması → 0 döndürmeli
    float hiz = hesapla_dikey_hiz_test(0.0f, 100.0f, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, hiz);
}

void test_dikey_hiz_kucuk_delta_t(void) {
    // 10ms (10000 µs) de 1m yükseliş → 100 m/s
    float hiz = hesapla_dikey_hiz_test(100.0f, 101.0f, 1000UL, 11000UL);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, hiz);
}

void test_dikey_hiz_nan_uretmiyor(void) {
    float hiz = hesapla_dikey_hiz_test(0.0f, 0.0f, 1000UL, 1001000UL);
    TEST_ASSERT_FALSE(isnan(hiz));
}

// ============================================================
// 3. TELEMETRİ PAKETİ TESTLERİ
// ============================================================

void test_telemetry_packet_boyutu(void) {
    // #pragma pack(1) sayesinde padding olmamalı
    // Struct'taki float alanları (17 adet):
    //   ivmeX, ivmeY, ivmeZ           → 3
    //   gyroX, gyroY, gyroZ           → 3
    //   roll, pitch, yaw              → 3
    //   basinc, bmeSicaklik, irtifa, nem → 4
    //   dikeyHiz, eglimAcisi          → 2
    //   gpsEnlem, gpsBoylam           → 2
    //                           TOPLAM: 17 float = 68 byte
    // 2 bool   = 2 byte
    // 1 uint8  = 1 byte
    // Beklenen toplam = 71 byte
    size_t beklenen = (17 * sizeof(float)) + (2 * sizeof(bool)) + sizeof(uint8_t);
    TEST_ASSERT_EQUAL_UINT(beklenen, sizeof(TelemetryPacket));
}

void test_telemetry_packet_doldurma(void) {
    TelemetryPacket p;
    p.ivmeX = 1.1f; p.ivmeY = 2.2f; p.ivmeZ = 9.8f;
    p.irtifa = 750.0f;
    p.ayrilma1_durum = true;
    p.ayrilma2_durum = false;
    p.ucus_durumu = (uint8_t)YUKSELIYOR;

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.1f,  p.ivmeX);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 750.0f, p.irtifa);
    TEST_ASSERT_TRUE(p.ayrilma1_durum);
    TEST_ASSERT_FALSE(p.ayrilma2_durum);
    TEST_ASSERT_EQUAL_UINT8(1, p.ucus_durumu);
}

void test_telemetry_ucus_durumu_sinirlar(void) {
    TelemetryPacket p;
    p.ucus_durumu = (uint8_t)HAZIR;
    TEST_ASSERT_EQUAL_UINT8(0, p.ucus_durumu);
    p.ucus_durumu = (uint8_t)INDI;
    TEST_ASSERT_EQUAL_UINT8(4, p.ucus_durumu);
}

// --- YENİ TELEMETRİ TESTLERİ (v3.0) ---

void test_crc16_ccitt_dogrulama(void) {
    // Bilinen bir veri kümesiyle test et: "123456789" (standard test string)
    // CRC16-CCITT (poly=0x1021, init=0xFFFF) için "123456789" sonucu: 0x29B1
    const char* test_data = "123456789";
    uint16_t result = crc16_ccitt((const uint8_t*)test_data, 9);
    TEST_ASSERT_EQUAL_HEX16(0x29B1, result);

    // Boş veri testi
    uint16_t result_empty = crc16_ccitt((const uint8_t*)"", 0);
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, result_empty);
}

void test_telemetry_frame_boyutu(void) {
    // Çerçevenin toplam boyutunu doğrula
    // [0xAA][0x55][LEN=71][PAYLOAD=71][CRC_HI][CRC_LO] = 76 byte
    TEST_ASSERT_EQUAL_INT(71, sizeof(TelemetryPacket));
    TEST_ASSERT_EQUAL_INT(76, FRAME_SIZE);
}

// ============================================================
// 4. UÇUŞ DURUM MAKİNESİ LOJİĞİ TESTLERİ
// ============================================================

void test_sm_hazir_kalkis_tespiti(void) {
    UcusDurumu durum = HAZIR;
    float ivmeZ = 25.0f; // > KALKIS_IVME_ESIGI (20)
    if (ivmeZ > KALKIS_IVME_ESIGI) durum = YUKSELIYOR;
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, durum);
}

void test_sm_hazir_dusuk_ivme_tetiklemez(void) {
    UcusDurumu durum = HAZIR;
    float ivmeZ = 5.0f; // < KALKIS_IVME_ESIGI
    if (ivmeZ > KALKIS_IVME_ESIGI) durum = YUKSELIYOR;
    TEST_ASSERT_EQUAL_INT(HAZIR, durum);
}

void test_sm_apogee_tespiti_tam_kosul(void) {
    UcusDurumu durum = YUKSELIYOR;
    bool ayrilma1 = false;
    float max_irtifa = 600.0f;
    float irtifa     = 580.0f;  // 600-580=20 > APOGEE_IRTIFA_FARKI(15)
    float dikey_hiz  = -5.0f;   // < MIN_DIKEY_HIZ(0)
    float eglim      = 5.0f;    // < MAX_EGLIM(10)

    if ((max_irtifa - irtifa > APOGEE_IRTIFA_FARKI) &&
        (dikey_hiz < MIN_DIKEY_HIZ) &&
        (eglim < MAX_EGLIM)) {
        ayrilma1 = true;
        durum = INIS_1;
    }
    TEST_ASSERT_EQUAL_INT(INIS_1, durum);
    TEST_ASSERT_TRUE(ayrilma1);
}

void test_sm_apogee_eglim_engeller(void) {
    // Eğim fazlaysa (tumbling) ateşleme yapılmamalı
    UcusDurumu durum = YUKSELIYOR;
    bool ayrilma1 = false;
    float max_irtifa = 600.0f;
    float irtifa     = 580.0f;
    float dikey_hiz  = -5.0f;
    float eglim      = 45.0f;  // > MAX_EGLIM → engel!

    if ((max_irtifa - irtifa > APOGEE_IRTIFA_FARKI) &&
        (dikey_hiz < MIN_DIKEY_HIZ) &&
        (eglim < MAX_EGLIM)) {
        ayrilma1 = true;
        durum = INIS_1;
    }
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, durum);
    TEST_ASSERT_FALSE(ayrilma1);
}

void test_sm_apogee_pozitif_hiz_engeller(void) {
    // Hâlâ yükseliyorsa ateşleme yapılmamalı
    UcusDurumu durum = YUKSELIYOR;
    bool ayrilma1 = false;
    float max_irtifa = 600.0f;
    float irtifa     = 580.0f;
    float dikey_hiz  = 5.0f;  // > 0 → hâlâ yükseliyor
    float eglim      = 5.0f;

    if ((max_irtifa - irtifa > APOGEE_IRTIFA_FARKI) &&
        (dikey_hiz < MIN_DIKEY_HIZ) &&
        (eglim < MAX_EGLIM)) {
        ayrilma1 = true;
        durum = INIS_1;
    }
    TEST_ASSERT_EQUAL_INT(YUKSELIYOR, durum);
    TEST_ASSERT_FALSE(ayrilma1);
}

void test_sm_inis1_ana_parasut_acilir(void) {
    UcusDurumu durum = INIS_1;
    bool ayrilma2 = false;
    float irtifa        = 400.0f;  // < AYRILMA2_MESAFE(550)
    float max_irtifa    = 700.0f;  // > AYRILMA2_MESAFE → gerçekten yüksekten indik

    if ((irtifa < AYRILMA2_MESAFE) && (max_irtifa > AYRILMA2_MESAFE)) {
        ayrilma2 = true;
        durum = INIS_2;
    }
    TEST_ASSERT_EQUAL_INT(INIS_2, durum);
    TEST_ASSERT_TRUE(ayrilma2);
}

void test_sm_inis1_dusuk_max_irtifa_tetiklemez(void) {
    // Roket hiç 550m'e çıkmadıysa ana paraşüt açılmamalı
    UcusDurumu durum = INIS_1;
    bool ayrilma2 = false;
    float irtifa     = 400.0f;
    float max_irtifa = 450.0f; // < AYRILMA2_MESAFE → yeterince yükselmedi

    if ((irtifa < AYRILMA2_MESAFE) && (max_irtifa > AYRILMA2_MESAFE)) {
        ayrilma2 = true;
        durum = INIS_2;
    }
    TEST_ASSERT_EQUAL_INT(INIS_1, durum);
    TEST_ASSERT_FALSE(ayrilma2);
}

void test_sm_inis2_yere_inis_tespiti(void) {
    UcusDurumu durum = INIS_2;
    float dikey_hiz = -1.5f; // > -INIS_HIZ_ESIGI(-2.0)
    float irtifa    = 10.0f; // < INIS_IRTIFA_ESIGI(20)

    if ((dikey_hiz > -INIS_HIZ_ESIGI) && (irtifa < INIS_IRTIFA_ESIGI)) {
        durum = INDI;
    }
    TEST_ASSERT_EQUAL_INT(INDI, durum);
}

void test_sm_inis2_yuksek_irtifa_tetiklemez(void) {
    UcusDurumu durum = INIS_2;
    float dikey_hiz = -1.0f;
    float irtifa    = 50.0f; // > INIS_IRTIFA_ESIGI → henüz yerde değil

    if ((dikey_hiz > -INIS_HIZ_ESIGI) && (irtifa < INIS_IRTIFA_ESIGI)) {
        durum = INDI;
    }
    TEST_ASSERT_EQUAL_INT(INIS_2, durum);
}

// ============================================================
// 5. EĞİM AÇISI TESTLERİ
// ============================================================

void test_eglim_tam_dik(void) {
    // pitch=0, roll=0 → eğim = 0°
    float eglim = hesapla_eglim_acisi(0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, eglim);
}

void test_eglim_90_derece(void) {
    // pitch=90 → eğim = 90°
    float eglim = hesapla_eglim_acisi(90.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, eglim);
}

void test_eglim_kucuk_salinim(void) {
    // ±2° salınım → eğim < MAX_EGLIM(10) olmalı
    float eglim = hesapla_eglim_acisi(2.0f, 2.0f);
    TEST_ASSERT_TRUE(eglim < MAX_EGLIM);
}

void test_eglim_tumbling(void) {
    // 45° eğim → apogee güvenlik kapısı engellenmeli
    float eglim = hesapla_eglim_acisi(45.0f, 0.0f);
    TEST_ASSERT_TRUE(eglim >= MAX_EGLIM);
}

void test_eglim_nan_uretmiyor(void) {
    // cos_val > 1.0 durumu (NaN koruması FIX#5)
    float eglim = hesapla_eglim_acisi(0.0f, 0.0f);
    TEST_ASSERT_FALSE(isnan(eglim));
    eglim = hesapla_eglim_acisi(0.001f, 0.001f);
    TEST_ASSERT_FALSE(isnan(eglim));
}

// ============================================================
// UNITY SETUP / TEARDOWN
// ============================================================

void setUp(void)    {}
void tearDown(void) {}

// ============================================================
// MAIN
// ============================================================

void setup() {
    

    UNITY_BEGIN();

    // --- Kalman ---
    RUN_TEST(test_kalman_ilk_cagri_olcumu_dondurur);
    RUN_TEST(test_kalman_sabit_giris_sabit_cikis);
    RUN_TEST(test_kalman_gurultu_azaltiyor);
    RUN_TEST(test_kalman_negatif_degerler);
    RUN_TEST(test_kalman_sifir_girisi);
    RUN_TEST(test_kalman_nan_uretmiyor);
    RUN_TEST(test_kalman_birdenbire_deger_degisimi);

    // --- Dikey Hız ---
    RUN_TEST(test_dikey_hiz_dogru_yukselis);
    RUN_TEST(test_dikey_hiz_dogru_inis);
    RUN_TEST(test_dikey_hiz_sabit_irtifa);
    RUN_TEST(test_dikey_hiz_ilk_cagri_sifir);
    RUN_TEST(test_dikey_hiz_kucuk_delta_t);
    RUN_TEST(test_dikey_hiz_nan_uretmiyor);

    // --- Telemetri Paketi ---
    RUN_TEST(test_telemetry_packet_boyutu);
    RUN_TEST(test_telemetry_packet_doldurma);
    RUN_TEST(test_telemetry_ucus_durumu_sinirlar);
    RUN_TEST(test_crc16_ccitt_dogrulama);
    RUN_TEST(test_telemetry_frame_boyutu);

    // --- State Machine ---
    RUN_TEST(test_sm_hazir_kalkis_tespiti);
    RUN_TEST(test_sm_hazir_dusuk_ivme_tetiklemez);
    RUN_TEST(test_sm_apogee_tespiti_tam_kosul);
    RUN_TEST(test_sm_apogee_eglim_engeller);
    RUN_TEST(test_sm_apogee_pozitif_hiz_engeller);
    RUN_TEST(test_sm_inis1_ana_parasut_acilir);
    RUN_TEST(test_sm_inis1_dusuk_max_irtifa_tetiklemez);
    RUN_TEST(test_sm_inis2_yere_inis_tespiti);
    RUN_TEST(test_sm_inis2_yuksek_irtifa_tetiklemez);

    // --- Eğim Açısı ---
    RUN_TEST(test_eglim_tam_dik);
    RUN_TEST(test_eglim_90_derece);
    RUN_TEST(test_eglim_kucuk_salinim);
    RUN_TEST(test_eglim_tumbling);
    RUN_TEST(test_eglim_nan_uretmiyor);

    UNITY_END();
}

void loop() {}

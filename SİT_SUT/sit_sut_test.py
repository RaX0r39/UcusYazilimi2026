import serial
import struct
import time
import threading

# --- AYARLAR ---
# ESP32'nin bağlı olduğu portu buraya yaz (Örn: "COM3" veya "/dev/cu.usbserial-xxx")
SERIAL_PORT = "/dev/cu.usbserial-0001" 
BAUD_RATE = 9600

# --- KOMUT PROTOKOL SABİTLERİ ---
HEADER_CMD = 0xAA
HEADER_DATA = 0xAB
FOOTER1 = 0x0D
FOOTER2 = 0x0A
CHK_OFFSET = 0x6C

CMD_SIT_BASLAT = 0x20
CMD_SUT_BASLAT = 0x22
CMD_DURDUR = 0x24

class RocketTester:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"Bağlantı Başarılı: {port} @ {baud}")
        except Exception as e:
            print(f"Bağlantı Hatası: {e}")
            print("İpucu: Port adını kontrol et (ls /dev/cu.* yazarak bulabilirsin)")
            exit()
        
        self.running = True
        self.read_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.read_thread.start()

    def send_command(self, cmd_byte):
        chk = (cmd_byte + CHK_OFFSET) & 0xFF
        packet = struct.pack("BBBBB", HEADER_CMD, cmd_byte, chk, FOOTER1, FOOTER2)
        self.ser.write(packet)
        print(f">> Komut Gönderildi: {hex(cmd_byte)}")

    def send_sut_data(self, alt, press, ax, ay, az, roll, pitch, yaw):
        """36 byte SUT paketi gönderir (Tablo 3 Formatı)"""
        # 8 adet float32 (Little Endian)
        payload = struct.pack("<ffffffff", alt, press, ax, ay, az, roll, pitch, yaw)
        chk = sum(payload) & 0xFF
        packet = struct.pack("B", HEADER_DATA) + payload + struct.pack("BBB", chk, FOOTER1, FOOTER2)
        self.ser.write(packet)

    def receive_loop(self):
        while self.running:
            if self.ser.in_waiting >= 36:
                header = self.ser.read(1)
                if header == b'\xab': # SİT Paketi Geldi
                    data = self.ser.read(35)
                    if len(data) == 35 and data[-2:] == b'\r\n':
                        vals = struct.unpack("<ffffffff", data[0:32])
                        print(f"\r[SİT VERİSİ] Irtifa: {vals[0]:.1f}m | Basınç: {vals[1]:.1f}hPa | İvmeZ: {vals[4]:.2f}", end="")
                elif header == b'\xaa': # Sistem Mesajı Geldi
                    line = self.ser.readline().decode('ascii', errors='ignore').strip()
                    print(f"\n[ROKET] {line}")
            time.sleep(0.01)

def main():
    tester = RocketTester(SERIAL_PORT, BAUD_RATE)
    
    print("\n================================================")
    print("   TRAKYA ROKET 2026 - SİT/SUT TEST PANELİ")
    print("================================================")
    print("1: SIT Başlat (Sensör Verilerini İzle)")
    print("2: SUT Başlat (Simülasyon Verisi Gönderimini Aç)")
    print("3: Durdur (Testi Bitir)")
    print("4: SUT Senaryosu (Uçuş Simülasyonu - 100m -> 2000m)")
    print("Q: Çıkış")

    try:
        while True:
            choice = input("\nSeçiminiz: ").upper()
            if choice == '1':
                tester.send_command(CMD_SIT_BASLAT)
            elif choice == '2':
                tester.send_command(CMD_SUT_BASLAT)
            elif choice == '3':
                tester.send_command(CMD_DURDUR)
            elif choice == '4':
                print(">> Simülasyon Senaryosu Koşuluyor...")
                tester.send_command(CMD_SUT_BASLAT)
                # Basit bir yükseliş simülasyonu
                for h in range(100, 2000, 25):
                    # h: irtifa, basınç, ivmeX, ivmeY, ivmeZ, roll, pitch, yaw
                    tester.send_sut_data(float(h), 1013.25 - (h/8.5), 0.0, 0.0, 20.0, 0.0, 0.0, 0.0)
                    time.sleep(0.05) 
                print("\n>> Senaryo Tamamlandı.")
            elif choice == 'Q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        tester.running = False
        print("\nBağlantı kapatıldı.")

if __name__ == "__main__":
    main()

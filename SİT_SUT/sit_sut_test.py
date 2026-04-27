import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys

# --- SABİT AYARLAR ---
BAUD_RATE = 9600
HEADER_CMD = 0xAA
HEADER_DATA = 0xAB
FOOTER1 = 0x0D
FOOTER2 = 0x0A
CHK_OFFSET = 0x6C

CMD_SIT_BASLAT = 0x20
CMD_SUT_BASLAT = 0x22
CMD_DURDUR = 0x24

def select_serial_port():
    """Sistemdeki aktif seri portları listeler ve kullanıcıya seçtirir."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("\n[!] HATA: Hiçbir seri port bulunamadı! ESP32'nin bağlı olduğundan emin olun.")
        sys.exit()

    print("\n--- MEVCUT SERİ PORTLAR ---")
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    
    while True:
        try:
            choice = input("\nPort Numarası Seç (Çıkış için Enter): ")
            if choice == "": # Düz enter basıldıysa kapat
                sys.exit()
            
            index = int(choice)
            if 0 <= index < len(ports):
                return ports[index].device
            else:
                print(f"Hata: 0 ile {len(ports)-1} arasında bir numara girin.")
        except ValueError:
            print("Hata: Lütfen geçerli bir sayı girin.")

class RocketTester:
    def __init__(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"\n[+] Bağlantı Başarılı: {port} @ {baud}")
        except Exception as e:
            print(f"\n[!] Bağlantı Hatası: {e}")
            sys.exit()
        
        self.running = True
        self.read_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.read_thread.start()

    def send_command(self, cmd_byte):
        chk = (cmd_byte + CHK_OFFSET) & 0xFF
        packet = struct.pack("BBBBB", HEADER_CMD, cmd_byte, chk, FOOTER1, FOOTER2)
        self.ser.write(packet)
        print(f"\n>> Komut Gönderildi: {hex(cmd_byte)}")

    def send_sut_data(self, alt, press, ax, ay, az, roll, pitch, yaw):
        payload = struct.pack("<ffffffff", alt, press, ax, ay, az, roll, pitch, yaw)
        chk = sum(payload) & 0xFF
        packet = struct.pack("B", HEADER_DATA) + payload + struct.pack("BBB", chk, FOOTER1, FOOTER2)
        self.ser.write(packet)

    def receive_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting >= 36:
                    header = self.ser.read(1)
                    if header == b'\xab': 
                        data = self.ser.read(35)
                        if len(data) == 35 and data[-2:] == b'\r\n':
                            vals = struct.unpack("<ffffffff", data[0:32])
                            print(f"\r[SİT VERİSİ] Irtifa: {vals[0]:.1f}m | Basınç: {vals[1]:.1f}hPa | İvmeZ: {vals[4]:.2f}", end="")
                    elif header == b'\xaa': 
                        line = self.ser.readline().decode('ascii', errors='ignore').strip()
                        print(f"\n[ROKET] {line}")
            except:
                break
            time.sleep(0.01)

def main():
    print("================================================")
    print("   TRAKYA ROKET 2026 - SİT/SUT TEST PANELİ")
    print("================================================")
    
    selected_port = select_serial_port()
    tester = RocketTester(selected_port, BAUD_RATE)
    
    print("\n--- KONTROL MENÜSÜ ---")
    print("1: SIT Başlat (Sensör Verilerini İzle)")
    print("2: SUT Başlat (Yapay Veri Girişi Hazırla)")
    print("3: Durdur (Testi Bitir)")
    print("4: SUT Senaryosu (Uçuş Simülasyonu - 100m -> 2000m)")
    print("Q: Çıkış (veya Enter)")

    try:
        while True:
            choice = input("\nSeçiminiz: ").upper()
            
            # Düz enter veya 'Q' basıldıysa çıkış yap
            if choice == "" or choice == "Q":
                break
                
            if choice == '1':
                tester.send_command(CMD_SIT_BASLAT)
            elif choice == '2':
                tester.send_command(CMD_SUT_BASLAT)
            elif choice == '3':
                tester.send_command(CMD_DURDUR)
            elif choice == '4':
                print(">> Simülasyon Senaryosu Koşuluyor...")
                tester.send_command(CMD_SUT_BASLAT)
                for h in range(100, 2000, 25):
                    tester.send_sut_data(float(h), 1013.25 - (h/8.5), 0.0, 0.0, 20.0, 0.0, 0.0, 0.0)
                    time.sleep(0.05) 
                print("\n>> Senaryo Tamamlandı.")
    except KeyboardInterrupt:
        pass
    finally:
        tester.running = False
        print("\nBağlantı kapatıldı, uygulama sonlandırıldı.")

if __name__ == "__main__":
    main()

#include <Servo.h>

Servo myServo;
int pirPin = 2;        // Pin untuk sensor PIR
int servoPin = 9;      // Pin untuk servo
int pirState = LOW;    // Status awal PIR

void setup() {
  pinMode(pirPin, INPUT);
  myServo.attach(servoPin);
  myServo.write(0);    // Posisi pintu tertutup
  Serial.begin(9600);  // Mulai komunikasi serial dengan Python
}

void loop() {
  int pirValue = digitalRead(pirPin);

  // Jika PIR mendeteksi gerakan
  if (pirValue == HIGH && pirState == LOW) {
    Serial.println("Gerakan terdeteksi, membuka pintu");
    myServo.write(90);  // Buka pintu
    delay(5000);        // Tunggu beberapa detik
    myServo.write(0);   // Tutup pintu
    pirState = HIGH;    // Ubah status PIR
  } else if (pirValue == LOW && pirState == HIGH) {
    Serial.println("Gerakan hilang, menunggu deteksi berikutnya");
    pirState = LOW;
  }

  delay(500); // Jeda untuk stabilitas sensor
}

import serial
import time

# Hubungkan ke port serial Arduino (ubah sesuai dengan port yang digunakan)
arduino_port = 'COM3'  # Untuk Windows (misalnya 'COM3')
# arduino_port = '/dev/ttyUSB0'  # Untuk Linux (misalnya '/dev/ttyUSB0')

baud_rate = 9600  # Baud rate sesuai dengan yang diatur di Arduino
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)  # Tunggu beberapa saat untuk memastikan koneksi serial siap

print("Memulai pemantauan pintu otomatis kandang kucing...")

try:
    while True:
        # Baca data dari serial
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print("Status:", data)
            
            # Contoh tambahan: Tambahkan logika berdasarkan data dari Arduino
            if "membuka pintu" in data:
                print("Pintu kandang terbuka!")
            elif "menunggu deteksi berikutnya" in data:
                print("Pintu kandang tertutup!")
                
except KeyboardInterrupt:
    print("Pemantauan dihentikan oleh pengguna.")

finally:
    ser.close()  # Tutup koneksi serial jika selesai

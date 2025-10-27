#include<ESP32Servo.h> // Library untuk mengendalikan servo di ESP32
#include<Adafruit_MPU6050.h> // Library untuk membaca sensor MPU6050
#include<Wire.h> // Library untuk komunikasi I2C
#include<Adafruit_Sensor.h> // Library untuk membantu memahami library Adafruit_MPU6050

Adafruit_MPU6050 mpu; // membuat objek mpu
Servo myservo1; // membuat objek servo 1
Servo myservo2; // membuat objek servo 2
Servo myservo3; // membuat objek servo 3
Servo myservo4; // membuat objek servo 4
Servo myservo5; // membuat objek servo 5

const int pinPIRSensor=23; // pin yang terhubung ke PIR motion sensor
const int pinServo1=5; // pin yang terhubung ke servo 1
const int pinServo2=16; // pin yang terhubung ke servo 2
const int pinServo3=19; // pin yang terhubung ke servo 3
const int pinServo4=18; // pin yang terhubung ke servo 4
const int pinServo5=13; // pin yang terhubung ke servo 5

bool yawberputar = false;  // menandai servo 5 berputar atau tidak

void setup () {
  Serial.begin(115200); // mengaktifkan serial monitor dengan baudrate 115200, untuk debugging
  if (!mpu.begin()) { // mengecek sensor MPU jika gagal sambung
  Serial.println("Gagal baca MPU6050!");
  while (1) delay(10); } // untuk keamanan, program berhenti
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // mengatur sensitivitas mpu agar pas dengan hitungan map

  pinMode (pinPIRSensor, INPUT); // pin PIRSensor sebagai input
  myservo1.attach(pinServo1); // untuk mengatur pin servo 1 sebagai output
  myservo2.attach(pinServo2); // untuk mengatur pin servo 2 sebagai output
  myservo3.attach(pinServo3); // untuk mengatur pin servo 3 sebagai output
  myservo4.attach(pinServo4); // untuk mengatur pin servo 4 sebagai output
  myservo5.attach(pinServo5); // untuk mengatur pin servo 5 sebagai output

  myservo1.write(90); // Set semua servo kembali posisi netral (90 derajat)
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
  myservo5.write(90);
}
void loop(){
  bool statusPIR = digitalRead(pinPIRSensor); // untuk membaca gerakan pada sensor
  sensors_event_t a, g, temp;  // menyiapkan ruang untuk menampung data
  mpu.getEvent(&a, &g, &temp); // mengisi ruang tsb
  float nilaiRoll = g.gyro.x; // untuk memasukkan data roll
  float nilaiPitch = g.gyro.y; // untuk memasukkan data pitch
  float nilaiYaw = g.gyro.z; // untuk memasukkan data yaw
  if (statusPIR==HIGH) { // mengecek gerakan 
    myservo1.write(45); // set semua servo menggerakkan ke posisi 45 derajat
    myservo2.write(45);
    myservo3.write(45);
    myservo4.write(45);
    myservo5.write(45);
    delay(1000); // diam sejenak
    myservo1.write(90); // Set semua servo kembali posisi netral (90 derajat)
    myservo2.write(90);
    myservo3.write(90);
    myservo4.write(90);
    myservo5.write(90);
    delay(500); // jeda
  } else { // jika sensor aman, memberi perintah jalan
    int sudutRoll = map(nilaiRoll, -250, 250, 180, 0); // menerjemahkan data roll melawan
    myservo1.write(sudutRoll); // menggerakan servo
    myservo2.write(sudutRoll);
    int sudutPitch = map(nilaiPitch, -250, 250, 0, 180); // menerjemahkan data pitch searah
    myservo3.write(sudutPitch); // menggerakkan servo
    myservo4.write(sudutPitch);
    float ambangBatasYaw = 5.0; // membuat batasan
    if (nilaiYaw > ambangBatasYaw || nilaiYaw < -ambangBatasYaw){ // jika sensor yaw sedang berputar cepat 
      int sudutYaw = map(nilaiYaw, -250, 250, 0, 180); // menerjemahkan data yaw searah
      myservo5.write(sudutYaw); // 
      yawberputar = true;
    } else { // jika sensor yaw diam
      if (yawberputar==true) {
        delay (1000); // diam sejenak
        myservo5.write(90); // servo kembali ke posisi netral (90 derajat)
        yawberputar = false; // untuk menghapus ingatan agar tidak mengulang-ulang
      }
    }
  }
    delay (50); // jeda
}


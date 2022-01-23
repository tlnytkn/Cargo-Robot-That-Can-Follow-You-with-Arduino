#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include "./TinyGPS.h"                
#include "./RobotTanimlamalari.h"
#define BLYNK_USE_DIRECT_CONNECT

#define echoPin 2
#define trigPin 3 

long surec; 
int uzaklik; 

TinyGPS gps;

bool durum = true;
SoftwareSerial Bluetooth_Baglantisi(BLUETOOTH_TX_PIN, BLUETOOTH_RX_PIN);
SoftwareSerial GPS_TX(GPS_TX_PIN, 255);


Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

Cografi_Lokasyon GPS_Kontrol() {
  bool yeniVeri = false;
  unsigned long baslangic = millis();
  while (millis() - baslangic < GPS_UPDATE_INTERVAL) {
    if (GPS_Besle())
      yeniVeri = true;
  }
  if (yeniVeri) {
    return gpsdump(gps);
  }

  Cografi_Lokasyon robotLokasyon;
  robotLokasyon.lat = 0.0;
  robotLokasyon.lon = 0.0;
  
  return robotLokasyon;
}

Cografi_Lokasyon gpsdump(TinyGPS &gps) {
  float f1, f2;
  unsigned long f3;
  gps.f_get_position(&f1, &f2, &f3);
  Cografi_Lokasyon robotLokasyon;
  robotLokasyon.lat = f1;
  robotLokasyon.lon = f2;
  return robotLokasyon;
}

bool GPS_Besle() {
  while (GPS_TX.available()) {
    if (gps.encode(GPS_TX.read()))
      return true;
  }
  return false;
}

BLYNK_WRITE(V1) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  digitalWrite(MOTOR_A_EN_PIN, HIGH);
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  digitalWrite(MOTOR_B_EN_PIN, HIGH);
  yakinlikKontrol();
}

BLYNK_WRITE(V2) {
  GpsParam gps(param);
  Cografi_Lokasyon telefonKonumu;
  telefonKonumu.lat = gps.getLat();
  telefonKonumu.lon = gps.getLon();
  Hedefe_Git(telefonKonumu, GPS_STREAM_TIMEOUT);
}


BLYNK_WRITE(V3) {
  digitalWrite(24,HIGH);
  digitalWrite(25,LOW);
  yakinlikKontrol();
}

BLYNK_WRITE(V4) {
  digitalWrite(24,LOW);
  digitalWrite(25,HIGH);
  yakinlikKontrol();
}

BLYNK_WRITE(V5) {
  digitalWrite(24,LOW);
  digitalWrite(25,LOW);
  yakinlikKontrol();
}

BLYNK_WRITE(V6) {
  int deger = param.asInt();
  analogWrite(4, deger);
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  yakinlikKontrol();
}

BLYNK_WRITE(V7) {
  int deger = param.asInt();
  analogWrite(9, deger);
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  yakinlikKontrol();
}

BLYNK_WRITE(V8) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);
  yakinlikKontrol();
}


#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

float Cografi_Rulman(struct Cografi_Lokasyon &lokasyon, struct Cografi_Lokasyon &lokasyon_1) {
  float y = sin(lokasyon_1.lon-lokasyon.lon) * cos(lokasyon_1.lat);
  float x = cos(lokasyon.lat)*sin(lokasyon_1.lat) - sin(lokasyon.lat)*cos(lokasyon_1.lat)*cos(lokasyon_1.lon-lokasyon.lon);
  return atan2(y, x) * RADTODEG;
}

float Cografi_Uzaklik(struct Cografi_Lokasyon &lokasyon_1, struct Cografi_Lokasyon &lokasyon_2) {
  const float R = 6371000;
  float p1 = lokasyon_1.lat * DEGTORAD;
  float p2 = lokasyon_2.lat * DEGTORAD;
  float dp = (lokasyon_2.lat-lokasyon_1.lat) * DEGTORAD;
  float dl = (lokasyon_2.lon-lokasyon_1.lon) * DEGTORAD;

  float x = sin(dp/2) * sin(dp/2) + cos(p1) * cos(p2) * sin(dl/2) * sin(dl/2);
  float y = 2 * atan2(sqrt(x), sqrt(1-x));

  return R * y;
}

float Cografi_Rota() { 
  sensors_event_t olayIsleyici; 
  mag.getEvent(&olayIsleyici);
  float rota = atan2(olayIsleyici.magnetic.y, olayIsleyici.magnetic.x);

  rota -= DECLINATION_ANGLE;
  rota -= COMPASS_OFFSET;

  if(rota < 0)
    rota += 2*PI;
    
  if(rota > 2*PI)
    rota -= 2*PI;
   
  float rotaAcisi = rota * 180/M_PI; 

  while (rotaAcisi < -180) rotaAcisi += 360;
  while (rotaAcisi >  180) rotaAcisi -= 360;

  return rotaAcisi;
}

void Motor_A_Calistir(int hiz) {
  digitalWrite(MOTOR_A_IN_1_PIN, LOW);
  digitalWrite(MOTOR_A_IN_2_PIN, HIGH);
  analogWrite(MOTOR_A_EN_PIN, hiz + MOTOR_A_OFFSET);
}

void Motor_B_Calistir(int hiz) {
  digitalWrite(MOTOR_B_IN_1_PIN, LOW);
  digitalWrite(MOTOR_B_IN_2_PIN, HIGH);
  analogWrite(MOTOR_B_EN_PIN, hiz + MOTOR_B_OFFSET);
}

void Hiz_Ayarla(int hiz)
{
  Motor_A_Calistir(hiz);
  Motor_B_Calistir(hiz);
}

void Dur() {
  digitalWrite(MOTOR_A_IN_1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN_2_PIN, LOW);  
  digitalWrite(MOTOR_B_IN_1_PIN, HIGH);
  digitalWrite(MOTOR_B_IN_2_PIN, LOW);
}

void Arac_Sur(int mesafe, float yon) {
  int fullHiz = 230;
  int durHizi = 0;

  int s = fullHiz;
  if ( mesafe < 8 ) {
    int istenilenHiz = s - durHizi;
    istenilenHiz *= mesafe / 8.0f;
    s = durHizi + istenilenHiz; 
  }
  
  int autoGaz = constrain(s, durHizi, fullHiz);
  autoGaz = 230;

  float t = yon;
  while (t < -180) t += 360;
  while (t >  180) t -= 360;
  
  float gerekliYon = (180.0 - abs(t)) / 180.0;
  float Motor_A_Hiz = 1;
  float Motor_B_Hiz = 1;

  if (t < 0) {
    Motor_B_Hiz = gerekliYon;
  } else if (t > 0){
    Motor_A_Hiz = gerekliYon;
  }

  int Hiz_A = (int) (((float) autoGaz) * Motor_A_Hiz);
  int Hiz_B = (int) (((float) autoGaz) * Motor_B_Hiz);
  
  Motor_A_Calistir(Hiz_A);
  Motor_B_Calistir(Hiz_B);
}

void Hedefe_Git(struct Cografi_Lokasyon &lokasyon, int zamanAsimi) {
  GPS_TX.listen();
  Cografi_Lokasyon robotLokasyonu = GPS_Kontrol();
  Bluetooth_Baglantisi.listen();

  if (robotLokasyonu.lat != 0 && robotLokasyonu.lon != 0 && durum) {
    float d = 0;
    do {
      GPS_TX.listen();
      robotLokasyonu = GPS_Kontrol();
      Bluetooth_Baglantisi.listen();
      d = Cografi_Uzaklik(robotLokasyonu, lokasyon);
      float t = Cografi_Rulman(robotLokasyonu, lokasyon) - Cografi_Rota();
      Arac_Sur(d, t);
      zamanAsimi -= 1;
    } while (d > 3.0 && durum && zamanAsimi>0);
    Dur();
  }
}


void setup()
{

  pinMode(trigPin, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(echoPin, INPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); 

  pinMode(MOTOR_A_EN_PIN, OUTPUT);
  pinMode(MOTOR_B_EN_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN_2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN_2_PIN, OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(25,OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  GPS_TX.begin(9600);
  Bluetooth_Baglantisi.begin(9600);
  Blynk.begin(Bluetooth_Baglantisi, auth);
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  surec = pulseIn(echoPin, HIGH);
  uzaklik = surec * 0.034 / 2; 
  delay(1000);
  Blynk.run();
}

void yakinlikKontrol(){
  
  digitalWrite(41, LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  surec = pulseIn(echoPin, HIGH);
  uzaklik = surec * 0.034 / 2; 

  if(uzaklik<15){
    digitalWrite(41, HIGH);
    digitalWrite(40, HIGH);
    delay(100);
    digitalWrite(40, LOW);
    digitalWrite(41, LOW);
    delay(100);
    digitalWrite(41, HIGH);
    digitalWrite(40, HIGH);
    delay(100);
    digitalWrite(40, LOW);
    digitalWrite(41, LOW);
    delay(100);
  }
  delay(200);
  
}

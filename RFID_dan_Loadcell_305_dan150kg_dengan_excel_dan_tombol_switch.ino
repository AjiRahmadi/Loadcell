#include "DHT.h"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
#include <SPI.h> //inisialisasi serial

#include <HX711_ADC.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

DHT dht(DHTPIN, DHTTYPE);

HX711_ADC LoadCell(4, 5); // dt pin, sck pin
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD HEX address 0x27
int taree = 6;
int a = 0;
float b = 0;
int val = 0;
int SWITCHCHANGE = 7;
//////////ultra//////////
int trig = 8; // membuat varibel trig di set ke-pin 8
int echo = 9; // membuat variabel echo di set ke-pin 9
long durasi, jarak; // membuat variabel durasi dan jarak

////////////////////////////////////////////////////

#include <rdm6300.h>
#define RDM6300_RX_PIN 3 // read the SoftwareSerial doc above! may need to change this pin to 10...
#define READ_LED_PIN 12
#define buzzerstik 10

Rdm6300 rdm6300;

void setup()
{
  Serial.begin(9600);
  dht.begin(); //pin dht
  ////////ultra////
  pinMode(trig, OUTPUT); // set pin trig menjadi OUTPUT
  pinMode(echo, INPUT); // set pin echo menjadi INPUT

  pinMode(SWITCHCHANGE, INPUT);
  int var = digitalRead(SWITCHCHANGE);
  ////////////////
  Serial.println("CLEARSHEET"); //membersihkan sheet
  Serial.println("LABEL, ID, BOBOT, TINGGI, SUHU, KELEMBABAN, TANGGAL, JAM");

  pinMode (taree, INPUT_PULLUP);
  LoadCell.begin(); // start connection to HX711
  LoadCell.start(1000); // load cells gets 1000ms of time to stabilize

  /////////////////////////////////////
  //LoadCell.setCalFactor(375); // Calibarate your LOAD CELL with 100g weight, and change the value according to readings
  
  if((var) == LOW){
  LoadCell.setCalFactor(29134);//timbangan 150kh gantung 29134 
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Timbangan Gantung");
  }
    else {LoadCell.setCalFactor(12824);// TIMBANGAN 2 timbangan ORANG 180KG x 2 = 360KG= 12824
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Timbangan Duduk");
    }// TIMBANGAN 2 timbangan ORANG 180KG x 2 = 360KG= 12824
  Serial.println(var);
  /////////////////////////////////////

  lcd.init(); // begins connection to the LCD module
  lcd.clear();
  lcd.backlight(); // turns on the backlight
  lcd.setCursor(1, 0); // set cursor to first row
  lcd.print("Digital Scale "); // print out to LCD
  lcd.setCursor(0, 1); // set cursor to first row
  lcd.print(" 350KG MAX LOAD "); // print out to LCD
  delay(3000);
  lcd.clear();

  ///////////////////////////////////////rfid//////////////////

  pinMode(READ_LED_PIN, OUTPUT);
  digitalWrite(READ_LED_PIN, LOW);
  pinMode(buzzerstik, OUTPUT);
  rdm6300.begin(RDM6300_RX_PIN);
  Serial.println("\nPlace RFID tag near the rdm6300...");
}

void loop()
{ float tag;
  float i = LoadCell.getData(); // get output value
  float h= i;
  //float h = i / 1000;  //konversi gram ke kilogram
  /////////ultra///////
  digitalWrite(trig, LOW);
  delayMicroseconds(8);
  digitalWrite(trig, HIGH);
  delayMicroseconds(8);
  digitalWrite(trig, LOW);
  delayMicroseconds(8);
  durasi = pulseIn(echo, HIGH); // menerima suara ultrasonic
  jarak = (durasi / 2) / 29.1; // mengubah durasi menjadi jarak (cm)
  //Serial.println(jarak); // menampilkan jarak pada Serial Monitor
  /////////dht////////
  float hum = dht.readHumidity();
  float temp = dht.readTemperature();

  if (isnan(hum) || isnan(temp) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  /* if non-zero tag_id, update() returns true- a new tag is near! */
  if (rdm6300.update()) {
    tag = float(rdm6300.get_tag_id());
    digitalWrite(READ_LED_PIN, rdm6300.is_tag_near());
    digitalWrite(buzzerstik, 1);
    Serial.print("DATA,");
    Serial.print(tag);
    Serial.print(",");
    Serial.print(h);
    Serial.print("Kg");
    Serial.print(",");
    Serial.print(jarak);
    Serial.print("Cm");
    Serial.print(",");
    Serial.print(temp);
    Serial.print("C");
    Serial.print(",");
    Serial.print(hum);
    Serial.print("%");
    Serial.print(",");
    Serial.print("DATE");
    Serial.print(",");
    Serial.print("TIME");
    Serial.println("");
    delay(3000);
    rdm6300.update();
  }
  else {
    lcd.setCursor(1, 0); // set cursor to first row
    lcd.print("Digital Scale "); // print out to LCD
    LoadCell.update(); // retrieves data from the load cell
    float i = LoadCell.getData(); // get output value
    if (i < 0)
    {
      i = i * (-1);
      lcd.setCursor(1, 1);
      lcd.print("-");
      lcd.setCursor(9, 1);
      lcd.print("-");
    }
    else
    {
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(8, 1);
      lcd.print(" ");
    }

    lcd.setCursor(1, 1); // set cursor to secon row
    lcd.print(h, 1); // print out the retrieved value to the second row
    lcd.print("kg ");
    float z = i / 28.3495;
    lcd.setCursor(9, 1);
    lcd.print(z, 2);
    lcd.print("oz ");
    float h = i / 1000;

    if (i >= 350)
    {
      i = 0;
      lcd.setCursor(0, 0); // set cursor to secon row
      lcd.print("  Over Loaded   ");
      delay(200);
    }
    if (digitalRead (taree) == LOW)
    {
      lcd.setCursor(0, 1); // set cursor to secon row
      lcd.print("   Taring...    ");
      LoadCell.start(1000);
      lcd.setCursor(0, 1);
      lcd.print("                ");
    }

    //////////rfid///////////
    digitalWrite(buzzerstik, 0);
    //Serial.println("ready");
  }
}

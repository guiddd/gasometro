#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>

Adafruit_BMP085 bmp;

LiquidCrystal_I2C lcd(0x27, 16, 2); 

unsigned long now1 = 0;
unsigned long lastMeasure = 0;
#define longInterval 5000

unsigned long now2 = 0;
unsigned long lastMeasure2 = 0;
#define shortInterval 100

//BMP180
float Temperatura = 0;
float Presion = 0;
float Altura = 0;

//MQ
#define MQ7pin 35 //monoxido carbono
float sensorMq7Value;

#define MQ5pin 34 //concentracion gas
float sensorMq5Value;

#define Advertencia 1230 //valor de advertencia


//TEMT 6000
#define LIGHTSENSORPIN 32
float NivelLuz;

//Buzzer
#define pinBuzzer 4

#define ON 1
#define OFF 0

boolean state = false;

void setup()
{
  Serial.begin(115200);

  bmp.begin();

  pinMode(LIGHTSENSORPIN, INPUT);

  pinMode (pinBuzzer, OUTPUT);
  
  lcd.init();                     
  lcd.backlight();
  lcd.clear();
}

void loop()
{
  now1 = millis();
  now2 = millis();
  DisplayFun();
  Alarma();

  if ((now1 - lastMeasure) >= longInterval) {
    lastMeasure1 = now1;
    BMP180();
    MQRead();
    TEMT6000();
  }
  //si se superó el tiempo, volver a calcular todo
}

void DisplayFun() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.println(Temperatura);
  lcd.print("Alt: ");
  lcd.println(Altura);
  lcd.print("Pres: ");
  lcd.println(Presion);
  lcd.println("");
  lcd.print("Gas nat: ");
  lcd.println(sensorMq5Value);
  lcd.print("Humo: ");
  lcd.println(sensorMq7Value);
  lcd.println("");
  lcd.print("Luz: ");
  lcd.println(NivelLuz);
  if (sensorMq7Value > Advertencia || sensorMq5Value > Advertencia) {
    display.print("Precaución");
  }
}

void BMP180() {
  Temperatura = bmp.readTemperature();
  Presion = bmp.readPressure();
  Altura = bmp.readAltitude();
}

void MQRead() {
  sensorMq7Value = analogRead(MQ7pin);
  sensorMq5Value = analogRead(MQ5pin);
}

void TEMT6000() {
  NivelLuz = analogRead(LIGHTSENSORPIN);
  float square_ratio = NivelLuz / 4095.0; //porcentaje del valor máximo(1023)
  square_ratio = pow(square_ratio, 2.0);
}

void Alarma() {
  if (sensorMq7Value > Advertencia || sensorMq5Value > Advertencia) {
    switch (state) {
      case OFF:
        if ((now2 - lastMeasure2) >= shortInterval) {
          digitalWrite(pinBuzzer, 150);
          lastMeasure2 = now2;
          state = ON;
        }
        //si el timer supera el intervalo, se prende la alarma
        break;

      case ON:
        if ((now2 - lastMeasure2) >= shortInterval) {
          digitalWrite(pinBuzzer, LOW);
          lastMeasure2 = now2;
          state = OFF;
        }
        //si el timer supera el intervalo, se apaga la alarma
        break;
    }
  }
}

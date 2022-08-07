#include <MQUnifiedsensor.h>

#define Voltage_Resolution 5
#define pin A0
#define type "MQ-7" //MQ7
#define RatioMQ7CleanAir 27.5 //RS / R0 = 27.5 ppm 
#define PWMPin 5 // Pin connected to mosfet

//Sensor
MQUnifiedsensor MQ7(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
unsigned long oldTime = 0;

void setup() {
  /*
    BMP180
    ----------MQ7
    ----------MQ5
    Buzzer
    Temt6000
    Display Oled SPI
  */
  Serial.begin(9600); //Init serial port
  pinMode(PWMPin, OUTPUT);

  //math model to calculate PPM concentration
  MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ7.setA(99.042); MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value

  MQ7.init();
  /* Si RL value no es 10K tengo que reasignarlo con:
    MQ7.setRL(10);
  */

  //--Calibración del sens--
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i ++)
  {
    MQ7.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
    Serial.print(".");
  }
  MQ7.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }
  //--calibración terminada--

  MQ7.serialDebug(true);
}

void loop() {
  oldTime = millis();
  while (millis() - oldTime <= (60 * 1000))
  {
    analogWrite(5, 255); // 255 a 5v
    MQ7.update(); // Update data
    MQ7.readSensor();
    MQ7.serialDebug(); //print
    delay(500);
  }

  // 90s cycle
  oldTime = millis();
  while (millis() - oldTime <= (90 * 1000))
  {
    analogWrite(5, 20); // 255 is 100%, 20.4 is aprox 8% of Duty cycle for 90s
    MQ7.update(); // Update data
    MQ7.readSensor();
    MQ7.serialDebug(); // print
    delay(500);
  }
  // Total: 60 + 90s =  2.5 minutes.
}

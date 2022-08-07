#include "EasyBuzzer.h"

unsigned int frequency = 1000;
unsigned int beeps = 10;

void setup() {
  /*
    BMP180
    MQ7
    MQ5
    ----------Buzzer
    Temt6000
    Display Oled SPI
  */
  Serial.begin(115200);

  // suena en una frecuencia indefinidamente
  // EasyBuzzer.beep(frequency);

  // suena en una frecuencia x cantidad de beeps
  // EasyBuzzer.beep(frequency, beeps);
  
  // func como anterior, pero con callback
  // EasyBuzzer.beep(frequency, beeps, done);

  //func para parar beep
  // EasyBuzzer.stopBeep();
}

void loop() {
  /* SIEMPRE llamar esta func para que funcione*/
  EasyBuzzer.update();
}

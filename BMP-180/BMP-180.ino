#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 pressure; // obj
#define ALTITUDE 1655.0

void setup() {
  /*
    ----------BMP180
    https://github.com/sparkfun/BMP180_Breakout/blob/master/Libraries/Arduino/examples/SFE_BMP180_example/SFE_BMP180_example.ino
    MQ7
    MQ5
    Buzzer
    Temt6000
    Display Oled SPI
  */
  Serial.begin(9600);
  Serial.println("REBOOT");
  if (pressure.begin())
    Serial.println("BMP180 init success");
}

void loop() {
  char status;
  double T, P, p0, a;
  //t:temp, p: pression, a: altitud, p0: sealevel

  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE, 0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE * 3.28084, 0);
  Serial.println(" feet");

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // arranca a sensar y pasa a obtener el dato

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T, 2);
      Serial.print(" deg C, ");
      Serial.print((9.0 / 5.0)*T + 32.0, 2);
      Serial.println(" deg F");

      // una vez que tiene la temp, pasa a la presión

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // arranca a sensar y pasa a obtener el dato

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P, 2);
          Serial.print(" mb, ");
          Serial.print(P * 0.0295333727, 2);
          Serial.println(" inHg");

          /* La presión, al ser absoluta, varái con la altitud.
             Si queremos corregirla, usamos el sealevel function y la altitud.

            p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
            Serial.print("relative (sea-level) pressure: ");
            Serial.print(p0, 2);
            Serial.print(" mb, ");
            Serial.print(p0 * 0.0295333727, 2);
            Serial.println(" inHg");*/

          // nos permite obtener la altitud DSPS de la presión y temp
          a = pressure.altitude(P, p0);
          Serial.print("computed altitude: ");
          Serial.print(a, 0);
          Serial.print(" meters, ");
          Serial.print(a * 3.28084, 0);
          Serial.println(" feet");
        }
        else Serial.println("error obteniendo presión\n");
      }
      else Serial.println("error comenzando presión\n");
    }
    else Serial.println("error obteniendo temp\n");
  }
  else Serial.println("error comenzando temp\n");

  //delay(1000)

}

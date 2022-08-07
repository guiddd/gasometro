#define LEDPIN 11         //LED brightness (PWM) writing
#define LIGHTSENSORPIN A0 //Ambient light sensor reading 

void setup() {
  /*
    BMP180
    MQ7
    MQ5
    Buzzer
    ----------Temt6000
    Display Oled SPI
  */
  pinMode(LIGHTSENSORPIN,  INPUT);  
  pinMode(LEDPIN, OUTPUT);  
  Serial.begin(9600);
}

void loop() {
  float reading = analogRead(LIGHTSENSORPIN); //Read light level

  //En caso de querer pasar el valor a un led
  /*
  float square_ratio = reading / 1023.0;      //Get percent of maximum value (1023)
  analogWrite(LEDPIN, 255.0 * square_ratio); 
  */
  
  Serial.println(reading);
}

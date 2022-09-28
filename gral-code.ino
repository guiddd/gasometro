#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include "AsyncMqttClient.h"
#include "time.h"

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

//////wifi 
const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$";

const char name_device = 16;  ////device numero de grupo 5A 1x siendo x el numero de grupo
                             ///                        5B 2x siendo x el numero de grupo   

const unsigned long interval_envio = 30000;//Intervalo de envio de datos mqtt
const unsigned long interval_leeo =  60000;//Intervalo de lectura de datos y guardado en la cola 
int i = 0;

///time
long unsigned int timestamp ;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

///variables ingresar a la cola struct
int indice_entra=0;
int indice_saca=0;
bool flag_vacio=1;

/////mqqtt
#define MQTT_HOST IPAddress(10, 162, 24, 31)
#define MQTT_PORT 1883
#define MQTT_USERNAME "esp32"
#define MQTT_PASSWORD "mirko15"
char mqtt_payload[150] ;  /////
// Test MQTT Topic
#define MQTT_PUB "/esp32/datos_sensores"
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

typedef struct
 {
     long time;
     float T1;///tempe
     float G5;///gas 5
     float G7;// gas 7
     float Presion; //presion
     float luz;
     bool Alarma;
     float ruido;
 }estructura ;
/////////////////
const int valor_max_struct=1000;///valor vector de struct 
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar 
estructura aux2 ;

/////*********************************************************************/////
////////////////////////////setup wifi/////////////////////////////////////////
/////*********************************************************************/////
void setupmqtt()
{
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);
  connectToWifi();
}
////////////////////////////Envio de datos mqtt//////////////////////////////////////////
////////Funcion que envia valores cuando la estructura no este vacia /////////////////// 
///////////////////////////////////////////////////////////////////////////////////////
void fun_envio_mqtt ()
{
    fun_saca ();////veo si hay valores nuevos 
    if (flag_vacio==0)////si hay los envio 
    {
      Serial.print("enviando");
      ////genero el string a enviar 
      snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%u",name_device,aux2.time,aux2.T1,aux2.G5,aux2.G7,aux2.Presion,aux2.luz,aux2.ruido,aux2.Alarma); //random(10,50)
      aux2.time =0;///limpio valores
      aux2.T1 = 0;
      aux2.G5 = 0;
      aux2.G7 = 0;
      aux2.Presion = 0;
      aux2.luz = 0;
      aux2.Alarma = 0;
      aux2.ruido = 0;

      Serial.print("Publish message: ");
      Serial.println(mqtt_payload);
      // Publishes Temperature and Humidity values
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB, 1, true, mqtt_payload);                            
    }
    else
    {
      Serial.println("no hay valores nuevos"); 
    }
}///////////////////////////////////////////////////

///////////////////////////////////////////////////
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}///////////////////////////////////////////////////
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}///////////////////////////////////////////////////

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}///////////////////////////////////////////////////
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}///////////////////////////////////////////////////

////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
////////////////NO TOCAR FUNCIONES MQTT///////////////////////////////////
void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}///////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
/////////////Funcion que saca un valor de la estructura para enviar //////
///////////////////////////////////////////////////////////////////////
void fun_saca (){
  if (indice_saca != indice_entra)
  {
    aux2.time = datos_struct[indice_saca].time;
    aux2.T1 = datos_struct[indice_saca].T1;
    aux2.G5 = datos_struct[indice_saca].G5;
    aux2.G7 = datos_struct[indice_saca].G7;
    aux2.Presion = datos_struct[indice_saca].Presion;
    aux2.luz = datos_struct[indice_saca].luz;
    aux2.Alarma = datos_struct[indice_saca].Alarma;
    aux2.ruido = datos_struct[indice_saca].ruido;
 
    flag_vacio = 0; 

    Serial.println(indice_saca);
    if (indice_saca>=(valor_max_struct-1))
      {
      indice_saca=0;
      }
    else
      {
      indice_saca++;
      }
    Serial.print("saco valores de la struct isaca:");
    Serial.println(indice_saca);
  }
  else 
  {
    flag_vacio=1;   ///// no hay datos
  }
  return ;
}
/////////////////////////////////////////////////////////////////////
/////////////funcion que ingresa valores a la cola struct///////////
///////////////////////////////////////////////////////////////////
void fun_entra (void)
{
  if (indice_entra>=valor_max_struct)
  {
    indice_entra=0; ///si llego al maximo de la cola se vuelve a cero 
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:"); 
  timestamp =  time(NULL);
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora 
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time=timestamp;
  datos_struct[indice_entra].T1=20; /// leeo los datos 
  datos_struct[indice_entra].G5=30; //// se puede pasar por un parametro
  datos_struct[indice_entra].G7=50;
  datos_struct[indice_entra].luz=30;
  datos_struct[indice_entra].ruido=1;  ///// valores motor
  datos_struct[indice_entra].Presion=0;
  datos_struct[indice_entra].Alarma=1;
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}

void setup()
{
  Serial.begin(115200);
  setupmqtt();
 //Setup de time
 configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

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
  
  /*now = millis();
  if (now - lastMeasure1 > interval_envio) {    ////envio el doble de lectura por si falla algun envio 
    lastMeasure1 = now;/// cargo el valor actual de millis
    fun_envio_mqtt();///envio los valores por mqtt
  } 
  if (now - lastMeasure2 > interval_leeo) {
    lastMeasure2 = now;/// cargo el valor actual de millis
    fun_entra(); ///ingreso los valores a la cola struct
  }*/
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

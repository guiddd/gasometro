#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include "AsyncMqttClient.h"
#include "time.h"
#include "Arduino.h"

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define I2C_SDA 26
#define I2C_SCL 27

//TEMT
#define LIGHTSENSORPIN 34
float NivelLuz;

//MQ
#define MQ7pin 33
float sensorMq7Value;

#define MQ5pin 32
float sensorMq5Value;

#define Peligro 30

//INTERVALOS
unsigned long now = 0;
unsigned long lastMeasure1 = 0;
#define interval1 5000

unsigned long lastMeasure2 = 0;
#define interval2 100

//Buzzer
#define pinBuzzer 4

#define ON 1
#define OFF 0

boolean state = false;


//MQTT
//////wifi
const char* ssid = "ORT-IoT";
const char* password = "OrtIOTnew22$2";

const char name_device = 15;  ////device numero de grupo 5A 1x siendo x el numero de grupo
///                        5B 2x siendo x el numero de grupo

// Timers auxiliar variables//////////////////////////
unsigned long lastMeasure3 = 0; ///variable para contar el tiempo actual
unsigned long lastMeasure4 = 0; ///variable para contar el tiempo actual

const unsigned long interval_envio = 30000;//Intervalo de envio de datos mqtt
const unsigned long interval_leeo =  60000;//Intervalo de lectura de datos y guardado en la cola
int i = 0;

///time
long unsigned int timestamp ;  // hora
const char* ntpServer = "south-america.pool.ntp.org";
const long  gmtOffset_sec = -10800;
const int   daylightOffset_sec = 0;

///variables ingresar a la cola struct
int indice_entra = 0;
int indice_saca = 0;
bool flag_vacio = 1;

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
} estructura ;
/////////////////
const int valor_max_struct = 1000; ///valor vector de struct
estructura datos_struct [valor_max_struct];///Guardo valores hasta que lo pueda enviar
estructura aux2 ;


void setup() {
Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();
Serial.begin(115200);

   pinMode (pinBuzzer, OUTPUT);

  setupmqtt();
  //Setup de time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void loop() {

  now = millis();
  Alarma();
   if ((now - lastMeasure1) >= interval1) {
    lastMeasure1 = now;
    // TEMT
    NivelLuz = analogRead(LIGHTSENSORPIN);
    float square_ratio = NivelLuz / 4095.0; //Get percent of maximum value (1023)
    square_ratio = pow(square_ratio, 2.0);

    //MQ
    sensorMq7Value = analogRead(MQ7pin) *100 / 4095;
    sensorMq5Value = analogRead(MQ5pin)*100 / 4095;

    //PRINTEO
    lcd.print("Luz: ");
    lcd.print(NivelLuz);
    delay(1000);
    lcd.clear();

    lcd.print("Humo: ");
    lcd.print(sensorMq7Value);
    delay(1000);
    lcd.clear();

    lcd.print("Gas nat: ");
    lcd.print(sensorMq5Value);
    delay(1000);
    lcd.clear();

    if (sensorMq7Value > Peligro || sensorMq5Value > Peligro) {

    Serial.println("Precaución");
  }
     
   }
     if (now - lastMeasure3 > interval_envio) {    ////envio el doble de lectura por si falla algun envio
    lastMeasure3 = now;/// cargo el valor actual de millis
    fun_envio_mqtt();///envio los valores por mqtt
  }
  if (now - lastMeasure4 > interval_leeo) {
    lastMeasure4 = now;/// cargo el valor actual de millis
    fun_entra(); ///ingreso los valores a la cola struct
  }
   
}
void Alarma() {
  if (sensorMq7Value > Peligro || sensorMq5Value > Peligro) {
    switch (state) {
      case OFF:
        if ((now - lastMeasure2) >= interval2) {
          digitalWrite(pinBuzzer, HIGH);
          lastMeasure2 = now;
          state = ON;
        }
        break;

      case ON:
        if ((now - lastMeasure2) >= interval2) {
          digitalWrite(pinBuzzer, LOW);
          lastMeasure2 = now;
          state = OFF;
        }
        break;
    }
  }
}

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

void fun_envio_mqtt ()
{
  fun_saca ();////veo si hay valores nuevos
  if (flag_vacio == 0) ////si hay los envio
  {
    Serial.print("enviando");
    ////genero el string a enviar
    snprintf (mqtt_payload, 150, "%u&%ld&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%u", name_device, aux2.time, aux2.T1, aux2.G5, aux2.G7, aux2.Presion, aux2.luz, aux2.ruido, aux2.Alarma); //random(10,50)
    aux2.time = 0; ///limpio valores
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
  switch (event) {
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
void fun_saca () {
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
    if (indice_saca >= (valor_max_struct - 1))
    {
      indice_saca = 0;
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
    flag_vacio = 1; ///// no hay datos
  }
  return ;
}
/////////////////////////////////////////////////////////////////////
/////////////funcion que ingresa valores a la cola struct///////////
///////////////////////////////////////////////////////////////////
void fun_entra (void)
{
  if (indice_entra >= valor_max_struct)
  {
    indice_entra = 0; ///si llego al maximo de la cola se vuelve a cero
  }
  //////////// timestamp/////// consigo la hora
  Serial.print("> NTP Time:");
  timestamp =  time(NULL);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;  //// si no puede conseguir la hora
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  ///////////////////////// fin de consigo la hora
  datos_struct[indice_entra].time = timestamp;
  datos_struct[indice_entra].G5 = sensorMq5Value; //// se puede pasar por un parametro
  datos_struct[indice_entra].G7 = sensorMq7Value;
  datos_struct[indice_entra].luz = NivelLuz;
  datos_struct[indice_entra].ruido = 0; ///// valores motor
  datos_struct[indice_entra].Alarma = state;
  indice_entra++;
  Serial.print("saco valores de la struct ientra");
  Serial.println(indice_entra);
}

#include <ESP8266WiFi.h>        // Biblioteca para generar la conexión a internet a través de WiFi
#include <PubSubClient.h>       // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>        // Biblioteca para manejar Json en Arduino
#include "DHT.h"                // Biblioteca para trabajar con DHT 11 (Sensor de temperatura y humedad)
#include "arduinoFFT.h"         // FFT para microfono
#include "ESP.h"
/*========= CONSTANTES =========*/ //----------------------------------------------------------------------------------------------------------------------



// Credenciales de la red WiFi
const char* ssid = "Ale R";
const char* password = "aler1234";


// Host de ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;

// Token del dispositivo en ThingsBoard
const char* token = "x6mGC1E5lNZbIfE8EOjH";

// Tipo de sensor
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int digitalPinMic = 0;
#define DHTTYPE DHT11  // DHT 11
#define DHT_PIN A0  // Conexión en PIN D3
#define pinMotor 8

// FFT
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 9500; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;


/*========= VARIABLES =========*/ //--------------------------------------------------------------------------------------------------------------------

// Objetos de conexión
WiFiClient espClient;  // Objeto de conexión WiFi
PubSubClient client(espClient);  // Objeto de conexión MQTT

// Objetos de Sensores o Actuadores
DHT dht(DHT_PIN, DHTTYPE);

// Declaración de variables para los datos a manipular
unsigned long lastMsg = 0;  // Control de tiempo de reporte
int msgPeriod = 1000;  // Actualizar los datos cada 0.2 segundos

float valorAnalog = 0;
int valorDigital = 0;

bool flagAttribute = false;
double frecuencia = 0.0;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

boolean led_state = false;

// Mensajes y buffers
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

// Objeto Json para recibir mensajes desde el servidor
DynamicJsonDocument incoming_message(256);


/*========= FUNCIONES =========*/ //----------------------------------------------------------------------------------------------------------------------------

// Inicializar la conexión WiFi -----------------------------------------------------------------------------------------------------------------------------
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);  // Declarar la ESP como STATION
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("¡Conectado!");
  Serial.print("Dirección IP asignada: ");
  Serial.println(WiFi.localIP());
}

// Función de callback para recepción de mensajes MQTT (Tópicos a los que está suscrita la placa)
// Se llama cada vez que arriba un mensaje entrante (En este ejemplo la placa se suscribirá al tópico: v1/devices/me/rpc/request/+)

void callback(char* topic, byte* payload, unsigned int length) { //-----------------------------------------------------------------------------------------
  // Log en Monitor Serie
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // En el nombre del tópico agrega un identificador del mensaje que queremos extraer para responder solicitudes
  String _topic = String(topic);
  
  // Detectar de qué tópico viene el "mensaje"
  if (_topic.startsWith("v1/devices/me/rpc/request/")) {  // El servidor "me pide que haga algo" (RPC)
    // Obtener el número de solicitud (request number)
    String _request_id = _topic.substring(26);

    // Leer el objeto JSON (Utilizando ArduinoJson)
    deserializeJson(incoming_message, payload);  // Interpretar el cuerpo del mensaje como Json
    String metodo = incoming_message["method"];  // Obtener del objeto Json, el método RPC solicitado

    // Ejecutar una acción de acuerdo al método solicitado
    if (metodo == "checkStatus") {  // Chequear el estado del dispositivo. Se debe responder utilizando el mismo request_number

      char outTopic[128];
      ("v1/devices/me/rpc/response/" + _request_id).toCharArray(outTopic, 128);

      DynamicJsonDocument resp(256);
      resp["status"] = true;
      char buffer[256];
      serializeJson(resp, buffer);
      client.publish(outTopic, buffer);

    } else if (metodo == "setLedStatus") {  // Establecer el estado del led y reflejar en el atributo relacionado
      boolean estado = incoming_message["params"];  // Leer los parámetros del método
      flagAttribute = !flagAttribute;
      if(flagAttribute){
        digitalWrite(pinMotor, HIGH);
        Serial.print("______________________________ON________________________________");
      } else {
        digitalWrite(pinMotor, LOW);
        Serial.print("______________________________OFF________________________________");
      }
      

      // Actualizar el atributo relacionado
      DynamicJsonDocument resp(256);
      resp["estado"] = flagAttribute;
      char buffer[256];
      serializeJson(resp, buffer);
      client.publish("v1/devices/me/attributes", buffer);  //Topico para actualizar atributos
      Serial.print("Publish message [attribute]: ");
      Serial.println(buffer);
    }
    else{
      Serial.print("----------------------------------------------------------------------");
      Serial.print(metodo);
      Serial.print(WiFi.localIP());
      Serial.print("----------------------------------------------------------------------");
    }
  }
}


// Establecer y mantener la conexión con el servidor MQTT (En este caso de ThingsBoard) ----------------------------------------------------------------------
void reconnect() {
  
  // Bucle hasta lograr la conexión
  while (!client.connected()) {
    Serial.print("Intentando conectar MQTT...");
    if (client.connect("ESP8266", token, token)) {  //Nombre del Device y Token para conectarse
      Serial.println("¡Conectado!");

      // Una vez conectado, suscribirse al tópico para recibir solicitudes RPC
      client.subscribe("v1/devices/me/rpc/request/+");

    } else {
      Serial.print("Error, rc = ");
      Serial.print(client.state());

      // Esperar 5 segundos antes de reintentar
      Serial.println("Reintenar en 5 segundos...");
      delay(5000);
    }
  }
}









/*========= SETUP =========*/ //------------------------------------------------------------------------------------------------------------------------------
void setup() {
  delay(0);
  ESP.wdtDisable();
  // Conectividad
  Serial.begin(115200);  // Inicializar conexión Serie para utilizar el Monitor
  setup_wifi();  // Establecer la conexión WiFi
  client.setServer(mqtt_server, mqtt_port);  // Establecer los datos para la conexión MQTT
  client.setCallback(callback);  // Establecer la función del callback para la llegada de mensajes en tópicos suscriptos
Serial.println("El primero");
ESP.wdtFeed();
  pinMode(analogInPin, INPUT);
  ESP.wdtFeed();
  pinMode(digitalPinMic, INPUT);
  ESP.wdtFeed();
  pinMode(pinMotor, OUTPUT);
ESP.wdtFeed();
  // Sensores y actuadores
  pinMode(LED_BUILTIN, OUTPUT);  // Inicializar el LED como salida
  pinMode(DHT_PIN, INPUT);  // Inicializar el DHT como entrada
  Serial.println("Antes del dht");
  dht.begin();  // Iniciar el sensor DHT}
ESP.wdtFeed();
Serial.println("El segundo");
  // FFT
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
}


/*========= BUCLE PRINCIPAL =========*/ //---------------------------------------------------------------------------------------------------------------------

void loop() {
  
  // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected()) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();  // Y recuperarla en caso de desconexión
  }
  client.loop();  // Controlar si hay mensajes entrantes o para enviar al servidor
Serial.println("Antes del analog");
  // === Realizar las tareas asignadas al dispositivo ===

  // En este caso se medirá temperatura y humedad para reportar periódicamente
  // El control de tiempo se hace con millis para que no sea bloqueante y en "paralelo" completar
  // ciclos del bucle principal
  unsigned long now = millis();

  if (now - lastMsg > msgPeriod) {
    lastMsg = now;

    /* ====== LECTURA DE SENSORES ====== */
    frecuencia = lecturaFFT();
    valorAnalog = analogRead(analogInPin);
    Serial.println(valorAnalog);
    //valorDigital = digitalRead(digitalPinMic);
    //Serial.println(valorDigital);

    /* ====== LECTURA DE SENSORES ====== */

     // Actualizar el atributo relacionado ????????????????????????????????????????????????????????????????????????
    char buffer[256];
    DynamicJsonDocument resp(256);
    resp["estado"] = flagAttribute;
    //char buffer[256];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/attributes", buffer);  //Topico para actualizar atributos
    //Serial.print("Publish message [attribute]: ");
    //Serial.println(buffer);
    // ????????????????????????????????????????????????????????????????????????????????????????????????????????????
    // Publicar los datos en el tópico de telemetría para que el servidor los reciba
    //DynamicJsonDocument resp(256);

    resp["analog"] = valorAnalog;
    //resp["digital"] = valorDigital;
    resp["frecuencia"] = frecuencia;

    //char buffer[256];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/telemetry", buffer);  // Publica el mensaje de telemetría
    //Serial.print("Publicar mensaje [telemetry]: ");
    //Serial.println(buffer);
  }
}

double lecturaFFT(){
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(analogInPin);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  /* Print the results of the sampling according to time */
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //PrintVector(vReal, samples, SCL_INDEX);
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //Serial.println(x, 6); //Print out what frequency is the most dominant.
  return x;
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

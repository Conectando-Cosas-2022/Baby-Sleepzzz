#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>       // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>        // Biblioteca para manejar Json en Arduino
#include "app_httpd.cpp"

extern void stopCameraServer();
extern void startCameraServer();

// Objetos de conexión
WiFiClient espClient;  // Objeto de conexión WiFi
PubSubClient client(espClient);  // Objeto de conexión MQTT
/*========= CONSTANTES =========*/ //----------------------------------------------------------------------------------------------------------------------

// Host de ThingsBoard
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;

// Token del dispositivo en ThingsBoard
const char* token = "4jGTQEvpwozCHwME91Mu";

// Tipo de sensor
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
bool conex_things = false;
bool camera_state = false;



/*========= VARIABLES =========*/ //--------------------------------------------------------------------------------------------------------------------

// Declaración de variables para los datos a manipular
unsigned long lastMsg = 0;  // Control de tiempo de reporte
int msgPeriod = 2000;  // Actualizar los datos cada 2 segundos

// Mensajes y buffers
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
char msg2[MSG_BUFFER_SIZE];

// Objeto Json para recibir mensajes desde el servidor
DynamicJsonDocument incoming_message(256);

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"
//const char* ssid = "HUAWEI-IoT";
//const char* password = "ORTWiFiIoT";
const char* ssid = "Nico Valdes";
const char* password = "11111111";



// THINGSBOARD ------------------------------------------------------------------------------------------------------------------------------------------

void setupThings() {
  if(conex_things == false){
    conex_things = true;
    Serial.println("estoy en setupThings");
    // Conectividad
    client.setServer(mqtt_server, mqtt_port);  // Establecer los datos para la conexión MQTT
    client.setCallback(callback);  // Establecer la función del callback para la llegada de mensajes en tópicos suscriptos
  }
  reconnect();

}

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

    } else if(metodo == "controlCamera"){
      bool camera_state = incoming_message["params"];
      Serial.println(camera_state);

      // Actualizar el atributo relacionado
      DynamicJsonDocument resp(256);
      resp["camera_state"] = camera_state;
      char buffer[256];
      serializeJson(resp, buffer);
      client.publish("v1/devices/me/attributes", buffer);  //Topico para actualizar atributos
      Serial.print("Publish message [attribute]: ");

      if(camera_state){
        arrancaCamara();
      } else {
        frenaCamara();
      }

      
      Serial.println(buffer); 

    }
  }
}

void reportarIP(String ip){
  // Publicar los datos en el tópico de telemetría para que el servidor los reciba
    DynamicJsonDocument resp(1024);

    if(ip == "OFF"){
      resp["camera_state"] = false;
    } else {
      resp["camera_state"] = true;
    }
    
    resp["ip"] = ip;

    char buffer[1024];
    serializeJson(resp, buffer);
    client.publish("v1/devices/me/attributes", buffer);  // Publica el mensaje de telemetría
    Serial.print("Publicar mensaje [attributes]: ");
    Serial.println(buffer);
}


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

// CAMARA ---------------------------------------------------------------------------------------------------

void arrancaCamara(){
  Serial.println("1");
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  String ip = "http://" + WiFi.localIP().toString();
  reportarIP(ip);
}

void frenaCamara(){
  reportarIP("OFF");
  Serial.println("freno server");
  delay(1000);
  ESP.restart();
}






void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  setupThings();
  reportarIP("OFF");
}

void loop() {
  // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected() && camera_state) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();  // Y recuperarla en caso de desconexión
  }
  client.loop();  // Controlar si hay mensajes entrantes o para enviar al servidor
}

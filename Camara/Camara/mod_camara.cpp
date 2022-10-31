//#include <ESP8266WiFi.h>        // Biblioteca para generar la conexión a internet a través de WiFi
#include <PubSubClient.h>       // Biblioteca para generar la conexión MQTT con un servidor (Ej.: ThingsBoard)
#include <ArduinoJson.h>        // Biblioteca para manejar Json en Arduino





/*========= FUNCIONES =========*/ //----------------------------------------------------------------------------------------------------------------------------

// Inicializar la conexión WiFi -----------------------------------------------------------------------------------------------------------------------------
//void setup_wifi() {
//  delay(10);
//  Serial.println();
//  Serial.print("Conectando a: ");
//  Serial.println(ssid);

  //WiFi.mode(WIFI_STA);  // Declarar la ESP como STATION
  //WiFi.begin(ssid, password);

  //while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    //Serial.print(".");
 // }

  //randomSeed(micros());

  //Serial.println("");
  //Serial.println("¡Conectado!");
  //Serial.print("Dirección IP asignada: ");
  //Serial.println(WiFi.localIP());
//}

// Función de callback para recepción de mensajes MQTT (Tópicos a los que está suscrita la placa)
// Se llama cada vez que arriba un mensaje entrante (En este ejemplo la placa se suscribirá al tópico: v1/devices/me/rpc/request/+)




// Establecer y mantener la conexión con el servidor MQTT (En este caso de ThingsBoard) ----------------------------------------------------------------------


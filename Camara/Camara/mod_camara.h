/*========= CONSTANTES =========*/ //----------------------------------------------------------------------------------------------------------------------







void loop(){
    // === Conexión e intercambio de mensajes MQTT ===
  if (!client.connected()) {  // Controlar en cada ciclo la conexión con el servidor
    reconnect();  // Y recuperarla en caso de desconexión
  }
  client.loop();  // Controlar si hay mensajes entrantes o para enviar al servidor
}


void dht11_setup() {
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
  dht.setup(5, DHTesp::DHT11); // Connect DHT sensor to GPIO 5
}

void mqtt_setup() {
  mqtt.enableDebuggingMessages(); // Enable debugging messages sent to serial output
}

void send_dht_measurement() {

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  mqtt.publish("sensors/enviro1/humidity", String(humidity));
  mqtt.publish("sensors/enviro1/temperature", String(dht.toFahrenheit(temperature)));
  mqtt.publish("sensors/enviro1/heatindex", String(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true)));
  char strBuf[50];
  
  Udp.beginPacket("192.168.2.103", statsdUdpPort);
  sprintf(strBuf,"environment.bedroom.temperature:%f|g\n",  dht.toFahrenheit(temperature));
  Udp.write(strBuf);
  Udp.endPacket();
  
  Udp.beginPacket("192.168.2.103", statsdUdpPort);
  sprintf(strBuf,"environment.bedroom.heatindex:%f|g\n",  dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true));
  Udp.write(strBuf);
  Udp.endPacket();
  
  Udp.beginPacket("192.168.2.103", statsdUdpPort);
  sprintf(strBuf,"environment.bedroom.humidity:%f|g",  humidity);
  Udp.write(strBuf);
  Udp.endPacket();
}

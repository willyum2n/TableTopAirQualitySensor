#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
unsigned int statsdUdpPort = 8125; //statsd

WiFiUDP Udp;

#include <EspMQTTClient.h>
EspMQTTClient mqtt(
  "Three",
  "xenoglossophobia42",
  "192.168.2.103",  // MQTT Broker server ip
  "",   // Can be omitted if not needed
  "",   // Can be omitted if not needed
  "envirodevice",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

#include "DHTesp.h"
#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif
DHTesp dht;

#include <SoftwareSerial.h>
#include <Sds011.h>
#define SDS_PIN_RX D2
#define SDS_PIN_TX D3
#ifdef ESP32
HardwareSerial& serialSDS(Serial2);
Sds011Async< HardwareSerial > sds011(serialSDS);
#else
SoftwareSerial serialSDS;
Sds011Async< SoftwareSerial > sds011(serialSDS);
#endif

// The example stops the sensor for 10s, then runs it for 30s, then repeats.
// At tablesizes 19 and below, the tables get filled during duty cycle
// and measurement completes.
// At tablesizes 20 and above, the tables do not get completely filled
// and the rampup / 4 timeout trips, completing measurement at whatever
// number of measurements were recorded in the tables.
constexpr int pm_tablesize = 20;
int pm25_table[pm_tablesize];
int pm10_table[pm_tablesize];
bool is_SDS_running = true;


void start_SDS() {
    Serial.println("Start wakeup SDS011");

    if (sds011.set_sleep(false)) { is_SDS_running = true; }

    Serial.println("End wakeup SDS011");
}

void stop_SDS() {
    Serial.println("Start sleep SDS011");

    if (sds011.set_sleep(true)) { is_SDS_running = false; }

    Serial.println("End sleep SDS011");
}

void sds011_setup()
{

#ifdef ESP32
    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);
#else
    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif

    Serial.println("SDS011 start/stop and reporting sample");

    Sds011::Report_mode report_mode;
    if (!sds011.get_data_reporting_mode(report_mode)) {
        Serial.println("Sds011::get_data_reporting_mode() failed");
    }
    if (Sds011::REPORT_ACTIVE != report_mode) {
        Serial.println("Turning on Sds011::REPORT_ACTIVE reporting mode");
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
            Serial.println("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed");
        }
    }
}

void dht11_setup()
{
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

void onConnectionEstablished() {
  
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

void send_sds_measurement() {
   constexpr uint32_t down_s = 10;

    stop_SDS();
    Serial.print("stopped SDS011 (is running = ");
    Serial.print(is_SDS_running);
    Serial.println(")");

    uint32_t deadline = millis() + down_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.print(static_cast<int32_t>(deadline - millis()) / 1000);
        Serial.print(" ");
        sds011.perform_work();
	    mqtt.loop();
    }

    constexpr uint32_t duty_s = 30;

    start_SDS();
    Serial.print("started SDS011 (is running = ");
    Serial.print(is_SDS_running);
    Serial.println(")");

    sds011.on_query_data_auto_completed([](int n) {
        Serial.println("Begin Handling SDS011 query data");
        int pm25;
        int pm10;
        Serial.print("n = "); Serial.println(n);
        if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
            !isnan(pm10) && !isnan(pm25)) {
            Serial.print("PM10: ");
            Serial.println(float(pm10) / 10);
            Serial.print("PM2.5: ");
            Serial.println(float(pm25) / 10);
			mqtt.publish("sensors/enviro1/pm10", String(float(pm10) / 10));
			mqtt.publish("sensors/enviro1/pm25", String(float(pm25) / 10));
            char strBuf[50];

		    Udp.beginPacket("192.168.2.103", statsdUdpPort);
		    sprintf(strBuf,"environment.bedroom.pm10:%f|g\n",  float(pm10)/10);
		    Udp.write(strBuf);
		    Udp.endPacket();

		    Udp.beginPacket("192.168.2.103", statsdUdpPort);
		    sprintf(strBuf,"environment.bedroom.pm25:%f|g\n",  float(pm25)/10);
		    Udp.write(strBuf);
		    Udp.endPacket();
        }
        Serial.println("End Handling SDS011 query data");
        });

    if (!sds011.query_data_auto_async(pm_tablesize, pm25_table, pm10_table)) {
        Serial.println("measurement capture start failed");
    }

    deadline = millis() + duty_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.print(static_cast<int32_t>(deadline - millis()) / 1000);
        Serial.print(" ");
        sds011.perform_work();
	    mqtt.loop();
    }
	Serial.println();
}


void setup() {
  Serial.begin(115200);
  Serial.println(">>>>>>>>>>>MQTT");
  mqtt_setup();
  Serial.println(">>>>>>>>>>>DHT");
  dht11_setup();
  Serial.println(">>>>>>>>>>>SDS");
  sds011_setup();
}

void loop() {
  delay(dht.getMinimumSamplingPeriod());
  send_dht_measurement();
  mqtt.loop();
  send_sds_measurement();
}

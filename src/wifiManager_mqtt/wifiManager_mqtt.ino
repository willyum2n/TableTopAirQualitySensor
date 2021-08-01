
// WiFi MQTT Manager -------------------------------------------------------------------
#include <WiFiMQTTManager.h>
#include "secrets.h"

// Button that will put device into Access Point mode to allow for re-entering WiFi and MQTT settings
#define WIFI_RESET_PIN 4
WiFiMQTTManager wmm(WIFI_RESET_PIN, CAPTIVE_AP_PASSWORD);  // CAPTIVE_AP_PASSWORD is defined in the secrets.h file

// Serial Communications ---------------------------------------------------------------
#include <SoftwareSerial.h>
SoftwareSerial serialSDS;
Sds011Async<SoftwareSerial> sds011(serialSDS);

// DHT11 (Temp & Humidity) -------------------------------------------------------------
#include "DHTesp.h"
#define DHT_DATA_PIN 5;
DHTesp dht;

// SDS011 (Particulate Sensor) ---------------------------------------------------------
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

// Platform Check -----------------------------------------------------------------------
#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif


/**********************************************************************************************************************
 **********                                      Da Program                                                  **********
 **********************************************************************************************************************/ 
void setup() {
  Serial.begin(115200);
  Serial.println(F("WiFiMQTTManager Basic Example"));
  // set debug to true to get verbose logging
  // wm.wm.setDebugOutput(true);
  // most likely need to format FS but only on first use
  // wmm.formatFS = true;
  // optional - define the function that will subscribe to topics if needed
  wmm.subscribeTo = subscribeTo;
  // required - allow WiFiMQTTManager to do it's setup
  wmm.setup(__SKETCH_NAME__);
  // optional - define a callback to handle incoming messages from MQTT
  wmm.client->setCallback(subscriptionCallback);

    Serial.println(">>>>>>>>>>>DHT");
  dht11_setup();
  Serial.println(">>>>>>>>>>>SDS");
  sds011_setup();
}

// ---------------------------------------  DHT11 ---------------------------------------
void dht11_setup()
{
  Serial.println();
  Serial.println("DHT11 Setup: Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard = ARDUINO_BOARD;
  Serial.println(thisBoard);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
  dht.setup(DHT_DATA_PIN, DHTesp::DHT11); // Connected to DHT sensor
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

}

// ---------------------------------------  SDS011 ---------------------------------------
void sds011_setup() {

    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
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
void loop() {
  // required - allow WiFiMQTTManager to check for new MQTT messages, 
  // check for reset button push, and reconnect to MQTT if necessary 
  wmm.loop();

  

  delay(dht.getMinimumSamplingPeriod());
  send_dht_measurement();
  mqtt.loop();
  send_sds_measurement();
}

// optional function to subscribe to MQTT topics
void subscribeTo() {
  Serial.println("subscribing to some topics...");  
  // subscribe to some topic(s)
  char topic[100];
  snprintf(topic, sizeof(topic), "%s%s%s", "switch/", wmm.deviceId, "/led1/output");
  wmm.client->subscribe(topic);
}

// optional function to process MQTT subscribed to topics coming in
void subscriptionCallback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  //if (String(topic) == "switch/esp1234/led1/output") {
  //  Serial.print("Changing led1 output to ");
  //}
}

/*
   ESPJagaBooster - boost your radiator
   by Bas Vermulst

   == some remarks ==
   Temperature sensor connections are as follows:
    CN0 = ambient, leave floating just below the radiator
    CN1 = inlet, mount tightly to inlet water piping
    CN2 = outlet, mount tightly to outlet water piping

   Mounting the module against metal radiator plating reduces WiFi reception!
   If reception is poor (RSSI < -80 dB), consider relocating the module.
   
   
   === configuring ===
   Using the #define parameters in the CONFIGURATION section, do the following:
   1) configure WiFi via WIFI_SSID and WIFI_PASSWORD
   2) configure MQTT broker via MQTTSERVER
   3) set unique node name via NODE_NAME
   4) enable or disable slave mode via SLAVE_MODE and MASTER_NODE_NAME

   Slave mode makes the board copy the fan speed of another board.
   
   
   === compiling ===
   1) Add D1 mini to the board manager:
   File --> Preferences, add to board manager URLs: http://arduino.esp8266.com/stable/package_esp8266com_index.json
   
   Then:
   Tools -> Board --> Boards manager --> Install "esp8266" board library.

   2) Install required software libraries:
   - Adafruit ADS1X15 (by Adafruit)
   - MQTT (by Joel Gaehwiler)
   - ArduinoOTA (by Juraj Andrassy/Arduino)
   - ArduinoJson (by Benoit Blanchon)
   Install via "Manage Libraries..." under Tools.

   3) Configure board:
   Select Tools --> Board --> "ESP8266 boards" --> LOLIN(WEMOS) D1 R2 & mini

   4) Configure programming port:
   Connect board via USB cable.
   Select appropriate programming port via Tools --> Port

   === usage ===
   you can send the following commands to the board via MQTT:
      - NODE_NAME/fan-boostmode-ref: 0 = silent mode, 1 = boost mode
      - NODE_NAME/fan-controlmode-ref: 0 = automatic fan speed, 1 = manual fan speed
      - NODE_NAME/fan-speed-ref: value between 0 - 100 to control fan speed (only in manual fan speed)

   the board sends the following status messages via MQTT:
      - NODE_NAME/ip: ip-address
      - NODE_NAME/ssid: WiFi SSID
      - NODE_NAME/rssi: WiFi signal strength

      - NODE_NAME/interval: MQTT update interval
      - NODE_NAME/runtime: time the board has been powered on
      - NODE_NAME/reconnects: reconnection attempts since power on/reset

      - NODE_NAME/fan-boostmode: fan boost mode (0 = normal mode, 1 = boost mode)
      - NODE_NAME/fan-controlmode: current control mode (0 = auto, 1 = manual)
      - NODE_NAME/fan-enabled: fan power status (0 = power off, 1 = power on)
      - NODE_NAME/fan-speed: current fan speed (0 - 100)

      - NODE_NAME/temp-inlet: inlet temperature in deg C
      - NODE_NAME/temp-outlet: outet temperature in deg C
      - NODE_NAME/temp-delta-io: delta between inlet and outlet in deg C
      - NODE_NAME/temp-ambient: ambient temperature in deg C
*/

///////// CONFIGURATION ///////// 

// Node
#define NODE_NAME "radiator-wk-voor" // name of this module - only use small letters and hyphens ('-')
#define STANDALONE_MODE 0 // if set to 1, the board runs without WiFi and MQTT
#define SLAVE_MODE 0 // if set to 1, use control input from other node (only works if STANDALONE_MODE=0), module does not need temperature sensors in this mode
#define MASTER_NODE_NAME "radiator-wk-voor" // when in slave mode, this is the node name of the master

// WiFi
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// MQTT
#define MQTTSERVER "192.168.1.11"
#define MQTT_USERNAME "" // leave empty if no credentials are needed
#define MQTT_PASSWORD "" 
#define ENABLE_HOME_ASSISTANT_AUTODISCOVERY 1 // send node autodiscovery messages (0=disabled, 1=enabled)

// fan speed controller tuning
// (heating)
#define HEATING_LOWER_TEMPERATURE 27.5 // degrees C inlet temperature (start of control range, duty cycle = 0 %)
#define HEATING_UPPER_TEMPERATURE 45.0 // degrees C inlet temperature (end of control range, duty cycle = HEATING_SPEED_LIMIT %)
#define HEATING_SPEED_LIMIT 60 // max fan speed in normal mode
#define HEATING_SPEED_BOOST_LIMIT 100 // max fan speed in boost mode

#define HEATING_FAN_ENABLE_TEMPERATURE 27.5 // temperature above which to enable fans
#define HEATING_FAN_ENABLE_DELTA_T_ON 5.0 // temperature difference between inlet and ambient to turn on fans
#define HEATING_FAN_ENABLE_DELTA_T_OFF 4.0 // temperature difference between inlet and ambient to turn off fans

// fan speed controller tuning
// (cooling)
#define COOLING_SPEED 55 // fan speed in cooling mode (cooling uses a constant fan speed)
#define COOLING_SPEED_BOOST 100 // fan speed in boost cooling mode (cooling uses a constant fan speed)

#define COOLING_FAN_ENABLE_DELTA_T_ON 2.5 // temperature difference between inlet and ambient to turn on fans for cooling
#define COOLING_FAN_ENABLE_DELTA_T_OFF 1.5 // temperature difference between inlet and ambient to turn off fans for cooling

#define FAN_OFF_DELAY 600 // delay before fans are switched off in seconds (heating & cooling)





////////// DO NOT TOUCH ////////// 

// includes
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> // for OTA
#include <MQTT.h> // for MQTT
#include <WiFiClient.h> // for MQTT
#include <ArduinoOTA.h> // for OTA
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ArduinoJson.h> // for JSON used with autodiscovery

#define FW_VERSION "1.0.1"

// mqtt
#define MQTT_INTERVAL 5

// NTC
#define NTC_BETA 3950

// peripherals and IO
#define FAN_PIN D0
#define PWR_SW_PIN D5
#define V_BUS 3.3 // NTC resistor divider input voltage

// filtering
#define TEMPERATURE_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz
#define SUPPLY_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz

// pwm
#define FAN_PWM_FREQUENCY 25000 // 25 kHz PWM according to 4-pin fan standard

// initialization
WiFiClient wificlient_mqtt;
MQTTClient mqttclient(2048);

Adafruit_ADS1115 ads;

float battery_voltage = 0;
int fan_speed = 0;
int fan_controlmode = 0; // 0 = automatic, 1 = manual
int fan_enabled = 0;
int fan_boostmode = 0; // 0 = normal, 1 = boost

float water_in_temperature = 0;
float water_out_temperature = 0;
float T0, T1, T2, voltage_comp;

int mqtt_reconnects = 0;



void setup() {
  Serial.begin(115200);

  if(STANDALONE_MODE==0){
    // Start networking
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // optional: fixed IP address, it is recommended to assign a fixed IP via the DHCP server instead
    // IPAddress ip(192,168,1,31); IPAddress gateway(192,168,1,1); IPAddress subnet(255,255,0,0); WiFi.config(ip, gateway, subnet);
    Serial.print("Attempting to connect to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(1000);
    } Serial.println("");
    Serial.println("Connected");
    Serial.println("IP address: " + IPAddressString(WiFi.localIP()));
  
  
    // Init OTA updates
    ArduinoOTA.setPort(8266);    // Port defaults to 8266
    ArduinoOTA.setHostname(NODE_NAME);  // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setPassword("admin");
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_FS
        type = "filesystem";
      }
  
      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
    ArduinoOTA.begin();
  
  
    // Start MQTT
    mqttclient.begin(MQTTSERVER, wificlient_mqtt);
    mqttclient.onMessage(handleMQTTreceive);
    MQTTconnect();
  }

  
  // init PWM
  analogWriteRange(100);
  analogWriteFreq(FAN_PWM_FREQUENCY);
  analogWrite(FAN_PIN, 0);

  // start ADS ADC
  ads.setGain(GAIN_ONE);
  ads.begin();

  // enable fans
  pinMode(PWR_SW_PIN, OUTPUT);
  digitalWrite(PWR_SW_PIN, LOW);
}


// === Main stuff ====
void loop() {

  if(STANDALONE_MODE==0){
    ArduinoOTA.handle();
    handleMQTT();
  }
  
  handleTemperature();
  handleControl();
}

// handle temperature
void handleTemperature(void) {
  static unsigned long prev_millis = 0;

  if (InterruptPending(&prev_millis, 250, 1)) {
    int16_t adc0, adc1, adc2, adc3;
    float volts0, volts1, volts2; static float volts3=0.0;
    float R0, R1, R2;
    float steinhart;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    volts0 = ads.computeVolts(adc0);
    volts1 = ads.computeVolts(adc1);
    volts2 = ads.computeVolts(adc2);

    volts3 = (1 - SUPPLY_FILTER_COEFF) * volts3 + SUPPLY_FILTER_COEFF * (2 * ads.computeVolts(adc3)); // supply voltage (filtered)
    
    // R_ntc= R * (V_o/(V_i-V_o))
    R0 = 10.0e3 * (volts0 / (volts3 - volts0));
    R1 = 10.0e3 * (volts1 / (volts3 - volts1));
    R2 = 10.0e3 * (volts2 / (volts3 - volts2));


    // T = 1 / ( (1/T_0) + log(R / R_0)*1/beta)

    // AMBIENT
    if (R0 > 0) {
      steinhart = R0 / 10e3;
      steinhart = log(steinhart);
      steinhart /= NTC_BETA;
      steinhart += 1.0 / (25 + 273.15);
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;
      T0 = (1 - TEMPERATURE_FILTER_COEFF) * T0 + TEMPERATURE_FILTER_COEFF * steinhart;
    } else {
      T0 = 0;
    }


    // INLET
    if (R1 > 0) {
      steinhart = R1 / 10e3;
      steinhart = log(steinhart);
      steinhart /= NTC_BETA;
      steinhart += 1.0 / (25 + 273.15);
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;
      T1 = (1 - TEMPERATURE_FILTER_COEFF) * T1 + TEMPERATURE_FILTER_COEFF * steinhart;
    } else {
      T1 = 0;
    }

    // OUTLET
    if (R2 > 0) {
      steinhart = R2 / 10e3;
      steinhart = log(steinhart);
      steinhart /= NTC_BETA;
      steinhart += 1.0 / (25 + 273.15);
      steinhart = 1.0 / steinhart;
      steinhart -= 273.15;
      T2 = (1 - TEMPERATURE_FILTER_COEFF) * T2 + TEMPERATURE_FILTER_COEFF * steinhart;
    } else {
      T2 = 0;
    }
  }
}

void handleControl(void) {
  static unsigned long prev_millis = 0, fan_enabled_prev_millis = 0;

  if (InterruptPending(&prev_millis, 1000, 1)) {
    
    if (((SLAVE_MODE == 0)&&(fan_controlmode == 0)) || (STANDALONE_MODE == 1)){
      // automatic fan speed

      if(T0 > T1){ // we could be cooling
      
        if (T0 - T1 > COOLING_FAN_ENABLE_DELTA_T_ON) { // inlet lower than ambient, enable fans for cooling
          fan_enabled = 1;
          
          if(fan_boostmode){
            fan_speed = COOLING_SPEED_BOOST;
          }else{
            fan_speed = COOLING_SPEED;
          }
          
          fan_enabled_prev_millis=millis();
          
        } else if ( (T0 - T1 < COOLING_FAN_ENABLE_DELTA_T_OFF) && 
                    (InterruptPending(&fan_enabled_prev_millis, FAN_OFF_DELAY*1000, 1))
                    ){ // inlet higher than ambient, disable fans for cooling cooling
          fan_enabled = 0;
          fan_speed = 0;
          
        }
        
      }else{ // we could be heating

        if (T1 - T0 > HEATING_FAN_ENABLE_DELTA_T_ON) { // inlet higher than ambient, enable fans
          fan_enabled = 1;
          fan_speed = HEATING_SPEED_LIMIT * (T1 - HEATING_LOWER_TEMPERATURE) / (HEATING_UPPER_TEMPERATURE - HEATING_LOWER_TEMPERATURE);
          fan_enabled_prev_millis=millis();
          
          switch (fan_boostmode) {
            default:
            case 0:
              if (fan_speed > HEATING_SPEED_LIMIT)
                fan_speed = HEATING_SPEED_LIMIT;
              break;
            case 1:
              if (fan_speed > HEATING_SPEED_BOOST_LIMIT)
                fan_speed = HEATING_SPEED_BOOST_LIMIT;
              break;
          }
  
        } else if ( ((T1 - T0 < HEATING_FAN_ENABLE_DELTA_T_OFF) || (T1 < HEATING_FAN_ENABLE_TEMPERATURE)) &&
                    (InterruptPending(&fan_enabled_prev_millis, FAN_OFF_DELAY*1000, 1))
                    ){ // inlet temp is too low, disable fans
          fan_enabled = 0;
          fan_speed = 0;
        }
        
      }
      
    } else if(fan_controlmode == 1){
      // manual fan speed
      if(fan_speed > 0){
        fan_enabled = 1;
        fan_enabled_prev_millis=millis() - FAN_OFF_DELAY*1000; // we don't care about the turn-off delay in manual mode
      }else{
        fan_enabled = 0;
      }
    }


    if (fan_speed > 100)
      fan_speed = 100;
    if (fan_speed < 0)
      fan_speed = 0;

    analogWrite(FAN_PIN, 100-fan_speed);
    digitalWrite(PWR_SW_PIN, fan_enabled);
  }
}

// ===== Handles for MQTT =====
// handle connection and send messages at intervals
void handleMQTT(void) {
  static unsigned long prev_millis_mqtt = 0, prev_millis_autodiscovery = 0;
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi is not available. Waiting for WiFi connection.");
    delay(1000);
  }

  if (!mqttclient.connected()) {
    MQTTconnect();
  } else {
    mqttclient.loop();
    delay(10); // needed according to MQTT library documentation
    
    if (InterruptPending(&prev_millis_mqtt, MQTT_INTERVAL * 1000, 1)) {
      Serial.println("Sending MQTT update");
      mqttclient.publish(GetTopic("ip"), IPAddressString(WiFi.localIP()));
      mqttclient.publish(GetTopic("ssid"), WiFi.SSID());
      mqttclient.publish(GetTopic("rssi"), String(WiFi.RSSI()));

      mqttclient.publish(GetTopic("interval"), String(MQTT_INTERVAL));
      mqttclient.publish(GetTopic("firmware"), String(FW_VERSION));
      mqttclient.publish(GetTopic("runtime"), String(millis() / 1000));
      mqttclient.publish(GetTopic("reconnects"), String(mqtt_reconnects));

      mqttclient.publish(GetTopic("fan-speed"), String(fan_speed));
      mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
      mqttclient.publish(GetTopic("fan-boostmode"), String(fan_boostmode));
      mqttclient.publish(GetTopic("fan-enabled"), String(fan_enabled));

      mqttclient.publish(GetTopic("temp-inlet"), String(T1, 2));
      mqttclient.publish(GetTopic("temp-outlet"), String(T2, 2));
      mqttclient.publish(GetTopic("temp-delta-io"), String(T1 - T2, 2));

      mqttclient.publish(GetTopic("temp-ambient"), String(T0, 2));

      if(ENABLE_HOME_ASSISTANT_AUTODISCOVERY==1){
        sendHomeAssistantAutodiscoveryMessages();
      }
      
    }
  }
}


// connect and subscribe
void MQTTconnect(void) {
  if ((!mqttclient.connected())) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    int retVal;
    if(strcmp(MQTT_USERNAME, "") == 0){
      retVal=mqttclient.connect(NODE_NAME);
    }else{
      retVal=mqttclient.connect(NODE_NAME,MQTT_USERNAME,MQTT_PASSWORD);
    }

    if (retVal) {
      Serial.println("connected");
      mqttclient.publish(GetTopic("ip"), IPAddressString(WiFi.localIP()));
      
      mqttclient.subscribe(GetTopic("fan-speed-ref"));
      mqttclient.subscribe(GetTopic("fan-controlmode-ref"));
      mqttclient.subscribe(GetTopic("fan-boostmode-ref")); 
              
      if(SLAVE_MODE == 1){
        mqttclient.subscribe(GetTopicMaster("fan-speed"));
        mqttclient.subscribe(GetTopicMaster("fan-boostmode"));
        mqttclient.subscribe(GetTopicMaster("fan-enabled"));
      }

      mqtt_reconnects++;
    } else {
      Serial.print("failed");
    }
  }
}

// handle received MQTT messages
void handleMQTTreceive(String &topic, String &payload) {

  if (topic.indexOf(GetTopic("fan-speed-ref")) >= 0) {
    fan_speed = payload.toInt();
    mqttclient.publish(GetTopic("fan-speed"), String(fan_speed));
  }

  if (topic.indexOf(GetTopic("fan-controlmode-ref")) >= 0) {
    fan_controlmode = payload.toInt();
    mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
  }

  if (topic.indexOf(GetTopic("fan-boostmode-ref")) >= 0) {
    fan_boostmode = payload.toInt();
    mqttclient.publish(GetTopic("fan-boostmode"), String(fan_boostmode));
  }

    
  if((SLAVE_MODE == 1)&&(fan_controlmode == 0)){ // when using automatic control mode, listen to the master
    if (topic.indexOf(GetTopicMaster("fan-speed")) >= 0) {
      fan_speed = payload.toInt();
      mqttclient.publish(GetTopic("fan-speed"), String(fan_speed));
    }
  
    if (topic.indexOf(GetTopicMaster("fan-boostmode")) >= 0) {
      fan_boostmode = payload.toInt();
      mqttclient.publish(GetTopic("fan-boostmode"), String(fan_boostmode));
    }    

    if (topic.indexOf(GetTopicMaster("fan-enabled")) >= 0) {
      fan_enabled = payload.toInt();
      mqttclient.publish(GetTopic("fan-enabled"), String(fan_enabled));
    }   
  }
}


// send HA autodiscovery message
void sendHomeAssistantAutodiscoveryMessages(void){
  homeassistantAddSwitch("fan-boostmode", "Boost mode", true, "mdi:fan-plus");
  homeassistantAddSwitch("fan-controlmode", "Manual control", true, "mdi:controller-classic");

  homeassistantAddSensor("fan-speed", "%", "Fan speed", true, "mdi:fan");
  homeassistantAddSwitch("fan-enabled", "Fan status", false, "mdi:fan-chevron-down");
  
  homeassistantAddSensor("temp-inlet", "°C", "Water inlet temperature", false, "mdi:thermometer-lines");
  homeassistantAddSensor("temp-outlet", "°C", "Water outlet temperature", false, "mdi:thermometer-lines");
  homeassistantAddSensor("temp-delta-io", "°C", "Water temperature difference", false, "mdi:thermometer");
  
  homeassistantAddSensor("temp-ambient", "°C", "Ambient temperature", false, "mdi:home-thermometer-outline");

  homeassistantAddSensor("rssi", "dB", "WiFi signal strength", false, "mdi:wifi");
}

void homeassistantAddSensor(String sensor_name, String unit_measurement, String name_measurement, bool is_controllable, String icon){
  DynamicJsonDocument doc(1024);
  
  String topic = String(NODE_NAME) + String("/") + sensor_name;
  String HA_topic;

  if(is_controllable==true){
    HA_topic = "homeassistant/number/" + topic + String("/config");    
  }else{
    HA_topic = "homeassistant/sensor/" + topic + String("/config");    
  }


  doc["name"] = name_measurement;
  doc["unique_id"] = String(NODE_NAME) + String("_") + sensor_name;
  doc["object_id"] = String(NODE_NAME) + String("_") + sensor_name;
  doc["icon"] = icon;

  doc["state_topic"]= String(NODE_NAME) + String("/") + sensor_name;
  doc["unit_of_measurement"] = unit_measurement;

  if(is_controllable==true){
    doc["command_topic"] = String(NODE_NAME) + String("/") + sensor_name + String("-ref");
  }

  doc["device"]["name"] = String("ESPJagaBooster - ") + String(NODE_NAME);
  doc["device"]["model"] = String("ESPJagaBooster");
  doc["device"]["manufacturer"] = String("Bas Vermulst");
  doc["device"]["identifiers"] = String(NODE_NAME);
  doc["device"]["sw_version"] = String(FW_VERSION);
  
  
  String HA_payload;
  serializeJson(doc, HA_payload);
  mqttclient.publish(HA_topic, HA_payload);
}


void homeassistantAddSwitch(String sensor_name, String name_measurement, bool is_controllable, String icon){
  DynamicJsonDocument doc(1024);
  
  String topic = String(NODE_NAME) + String("/") + sensor_name;
  String HA_topic;
  
  if(is_controllable==true){
    HA_topic = String("homeassistant/switch/") + topic + String("/config");
  }else{
    HA_topic = String("homeassistant/binary_sensor/") + topic + String("/config");    
  }
  
  doc["name"] = name_measurement;
  doc["unique_id"] = String(NODE_NAME) + String("_") + sensor_name;
  doc["object_id"] = String(NODE_NAME) + String("_") + sensor_name;
  doc["icon"] = icon;

  doc["state_topic"]= String(NODE_NAME) + String("/") + sensor_name;
  
  if(is_controllable==true){
    doc["command_topic"] = String(NODE_NAME) + String("/") + sensor_name + String("-ref");
  
    doc["state_on"] = 1;
    doc["state_off"] = 0;
  }

  doc["payload_on"] = 1;
  doc["payload_off"] = 0;

  doc["device"]["name"] = String("ESPJagaBooster - ") + String(NODE_NAME);
  doc["device"]["model"] = String("ESPJagaBooster");
  doc["device"]["manufacturer"] = String("Bas Vermulst");
  doc["device"]["identifiers"] = String(NODE_NAME);
  doc["device"]["sw_version"] = String(FW_VERSION);
  
  
  String HA_payload;
  serializeJson(doc, HA_payload);
  mqttclient.publish(HA_topic, HA_payload);
}



// ===== Helper functions =====
String IPAddressString(IPAddress address) {
  return String(address[0]) + "." + String(address[1]) + "." + String(address[2]) + "." + String(address[3]);
}

bool InterruptPending(unsigned long *prev_millis, unsigned int period, int mode) {
  // mode = 0: approximate mode without catch-up
  // mode = 1: exact mode without catch-up
  // mode = 2: exact mode with catch-up
  // note: int overflow is handled properly

  if ( (millis() - (*prev_millis) > period) || (millis() - (*prev_millis) < 0)) {
    // trigger detected
    switch (mode) {
      default:
      case 0:
        // approximate mode without catch-up
        *prev_millis = millis();
        break;

      case 1:
        // exact mode without catch-up
        while (millis() - (*prev_millis) > period) { // unwind
          *prev_millis = *prev_millis + period;
        }
        break;

      case 2:
        // exact mode with catch-up
        *prev_millis = *prev_millis + period;
        break;
    }

    return true;
  } else {
    return false;
  }
}

String GetTopic(String topic) {
  return String(NODE_NAME) + String("/") + String(topic);
}

String GetTopicMaster(String topic) {
  return String(MASTER_NODE_NAME) + String("/") + String(topic);
}

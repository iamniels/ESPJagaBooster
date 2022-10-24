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
   NOTE: instructions for Arduino IDE version 1.8.x
   
   1) Install required software libraries:
   
      Select Tools --> "Manage Libraries...", then install the following items one by one:     
       - Adafruit ADS1X15 (by Adafruit)
       - MQTT (by Joel Gaehwiler) - you need to scroll way down the library list to find this one
       - ArduinoOTA (by Juraj Andrassy/Arduino)
       - ArduinoJson (by Benoit Blanchon) 


   2) Add WiFi module to the board manager as follows:
     
         File --> Preferences, add to board manager URLs: http://arduino.esp8266.com/stable/package_esp8266com_index.json
          Then:
         Select Tools --> Board --> Boards manager --> Install "esp8266" board library.
          And:
         Select Tools --> Board --> "ESP8266 Boards" --> LOLIN(WEMOS) D1 R2 & mini
  


   3) Configure programming port:
     
      First connect the +12V power supply to the module to power it. Then connect the board to your computer via USB cable. 
      After connecting, select the appropriate programming port via Tools --> Port (it should read something like COM..)
      
      NOTE: If you are unsure which COM-port to use, unplug the USB cable from the module, check the list of COM ports, plug the module back in, check the list of COM ports again.
            Select the COM port that appears after plugging in the USB cable.
  
      NOTE: After the first firmware upload, you can update the firmware over the air via WiFi by selecting the node-name port
            in the Port menu (only if you have enabled WiFi in the module configuration below).


   4) Click Sketch --> Upload. It will compile the source code and upload the firmware to the module.
      Wait for the programming to finish ("Hard resetting via RTS pin" means it's done).

   
   5) Done! Module is now active. Have a look at what it's doing by clicking Tools --> Serial Monitor. Set the port to 115200 baud (bottom right of window).
      If the module is connected via USB, you can see your status messages here.
     
   
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
      - NODE_NAME/temp-outlet: outlet temperature in deg C
      - NODE_NAME/temp-delta-io: delta between inlet and outlet in deg C
      - NODE_NAME/temp-ambient: ambient temperature in deg C
*/

///////// CONFIGURATION ///////// 

// Node
#define NODE_NAME "radiator-wk-voor" // name of this module - only use small letters and hyphens ('-'), no spaces allowed!
#define STANDALONE_MODE 0 // 0=mqtt enabled, 1=mqtt disabled. Note: WiFi and over-the-air updates are still active.
#define SLAVE_MODE 0 // 0=use own temperature sensors & control, 1=use control input from other node (only works if STANDALONE_MODE=0 and WIFI_ENABLE=1), module does not need temperature sensors in slave mode
#define MASTER_NODE_NAME "radiator-wk-voor" // when in slave mode, this is the node name of the master

// WiFi
#define WIFI_ENABLE 1 // 0=wifi disabled, 1=wifi enabled (disabled = no OTA updates, no MQTT, no Home Assistant)
#define WIFI_SSID "***REMOVED***"
#define WIFI_PASSWORD "***REMOVED***"

// MQTT
#define MQTTSERVER "192.168.1.11"
#define MQTT_USERNAME "" // leave empty if no credentials are needed
#define MQTT_PASSWORD "" 
#define ENABLE_HOME_ASSISTANT_AUTODISCOVERY 1 // send node autodiscovery messages over MQTT (0=disabled, 1=enabled)

// fan speed controller tuning
// (heating)
#define HEATING_LOWER_TEMPERATURE 27.5 // degrees C inlet temperature @ start of control range, duty cycle = 0 %
#define HEATING_UPPER_TEMPERATURE 45.0 // degrees C inlet temperature @ end of control range, duty cycle = HEATING_SPEED_LIMIT %
#define HEATING_SPEED_LIMIT 75 // fan speed in normal mode @ HEATING_UPPER_TEMPERATURE
#define HEATING_SPEED_BOOST_LIMIT 120 // fan speed in boost mode @ HEATING_UPPER_TEMPERATURE (you may enter a number > 100 here to increase the boost effect)

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

#define FW_VERSION "1.1.0"

// mqtt
#define MQTT_INTERVAL 10

// IO
#define V_BUS 3.3 // NTC resistor divider input voltage

// filtering
#define TEMPERATURE_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz
#define SUPPLY_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz


// PWM
#define FAN_PWM_FREQUENCY 10000 // Note: ESP8266 has interrupt based PWM, so we can't meet the 25 kHz PWM required by 4-pin fan standard (we overflow the interrupt stack if we try)
#define FAN_PIN D0
#define FAN_POWER_PIN D5


// initialization
WiFiClient wificlient_mqtt;
MQTTClient mqttclient(2048);

Adafruit_ADS1115 ads;

float battery_voltage = 0;
int fan_speed = 0;
int fan_controlmode = 0; // 0 = automatic, 1 = manual
int fan_enabled = 0;
int fan_boostmode = 0; // 0 = normal, 1 = boost

int status_led = 0; // 0 = off, 1 = on, 2 = blink slow, 3 = blink fast

float T0=-1e3, T1=-1e3, T2=-1e3;

int mqtt_reconnects = 0;

String node_hostname = "";


void setup() {
   
  status_led=3; // status led blinks fast
    
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESPJagaBooster starting...");
  
  Serial.print("FW version ");
  Serial.println(String(FW_VERSION));
  
  
  
  if(WIFI_ENABLE == 1){    
    // Start networking
    initWiFi();
  
    // Init OTA updates
    initOTA();

    if(STANDALONE_MODE==0){
      // Start MQTT
      mqttclient.begin(MQTTSERVER, wificlient_mqtt);
      mqttclient.onMessage(handleMQTTreceive);
      MQTTconnect();
    }  
  }
  
  initFan();

  // start ADS ADC
  ads.setGain(GAIN_ONE);
  ads.begin();

  // enable fans
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, LOW);
  
  Serial.println("Done. Fan control algorithm operational.");  

  status_led=1; // status led on
  
}



// =====================================
// ============ Main stuff =============
// =====================================
void loop() {
  if(WIFI_ENABLE==1){
    ArduinoOTA.handle();
    
    if(STANDALONE_MODE==0){
      handleMQTT();
    }
  }
  
  handleTemperature();
  handleControl();

}



// handle temperature
void handleTemperature(void) {
  static unsigned long prev_millis = 0;

  if (InterruptPending(&prev_millis, 250, 1)) {
    int16_t adc0, adc1, adc2, adc3;
    static float v_dc_3v3=0.0;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);

    if(v_dc_3v3 < 0.1){ // detect reboot
      v_dc_3v3 = (2 * ads.computeVolts(adc3)); // initialize value
    }else{ // value already present, update
      v_dc_3v3 = (1 - SUPPLY_FILTER_COEFF) * v_dc_3v3 + SUPPLY_FILTER_COEFF * (2 * ads.computeVolts(adc3)); // supply voltage (filtered)
    }

    calculateTemperature(&T0, ads.computeVolts(adc0), v_dc_3v3);
    calculateTemperature(&T1, ads.computeVolts(adc1), v_dc_3v3);
    calculateTemperature(&T2, ads.computeVolts(adc2), v_dc_3v3);

  }
}

void calculateTemperature(float* T, float v_out, float v_in){
  // calculate temperature with 10k NTC with 10k pull-up from measured voltages v_out and v_in of resistor divider
  
  // R_ntc = R_pullup * (V_o/(V_i-V_o))
  // T_ntc = 1/((log(R_ntc/R_ntc_nominal)/NTC_BETA)+1/(25.0+273.15))-273.15
  
  #define NTC_BETA 3950 // NTC beta
  #define NTC_R_NOMINAL 10.0e3 // NTC resistance at 25 deg C
  #define NTC_R_PULLUP 10.0e3 // pullup resistance

  float R, steinhart;
  
  if((v_out < 0.91*v_in)&&(v_out > 0.08*v_in)){ // sensor has valid range (between -20 deg C and +90 deg C)
    
    R = NTC_R_PULLUP * (v_out / (v_in - v_out));
    
    steinhart = R / NTC_R_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= NTC_BETA;
    steinhart += 1.0 / (25.0 + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;

    if(steinhart > (*T + 100)){ // detect re-connection of temperature sensor, initialize
      *T = steinhart;
    }else{
      *T = (1 - TEMPERATURE_FILTER_COEFF) * (*T) + TEMPERATURE_FILTER_COEFF * steinhart;
    }
    
  } else {
    *T = -1e3; // sensor has invalid reading
  }
}


void handleControl(void) {
  static unsigned long prev_millis = 0, fan_enabled_prev_millis = 0;

  if (InterruptPending(&prev_millis, 1000, 1)) {
    
    if (((SLAVE_MODE == 0)&&(fan_controlmode == 0)) || (STANDALONE_MODE == 1)){
      // automatic fan speed
      
      if(T0 < -100 || T1 < -100){
        // ambient and inlet temperature sensors are not connected, we can't use the automatic control mode
        fan_enabled = 0;
        fan_speed = 0;
        
        writeFanSpeed();
    
        Serial.println("Temperature sensor missing - automatic fan speed control disabled.");   
        return;
      }
      
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
          fan_enabled_prev_millis=millis();
          
          if(T1 > HEATING_LOWER_TEMPERATURE){
            switch (fan_boostmode) {
              default:
              case 0:
                fan_speed = HEATING_SPEED_LIMIT * (T1 - HEATING_LOWER_TEMPERATURE) / (HEATING_UPPER_TEMPERATURE - HEATING_LOWER_TEMPERATURE);
                if (fan_speed > HEATING_SPEED_LIMIT)
                  fan_speed = HEATING_SPEED_LIMIT;
                break;
                
              case 1:
                fan_speed = HEATING_SPEED_BOOST_LIMIT * (T1 - HEATING_LOWER_TEMPERATURE) / (HEATING_UPPER_TEMPERATURE - HEATING_LOWER_TEMPERATURE);
                if (fan_speed > HEATING_SPEED_BOOST_LIMIT)
                  fan_speed = HEATING_SPEED_BOOST_LIMIT;
                break;
            }
            
            if(fan_speed > 100)
              fan_speed=100;
              
          }else{
            fan_speed=0;
          }
  
        } else if ( (T1 - T0 < HEATING_FAN_ENABLE_DELTA_T_OFF) &&
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


    writeFanSpeed();
    
    Serial.println("Fan speed updated.");      
  }
}



// =====================================
// =========== WiFi stuff ==============
// =====================================
// WiFi connect
void initWiFi(void){
  WiFi.mode(WIFI_STA);
  
  String mac_string=String(WiFi.macAddress());
  mac_string.replace(":","");
  node_hostname = String(NODE_NAME).substring(0,23)+String("-")+mac_string.substring(6);
  
  WiFi.hostname(node_hostname.c_str());
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      
  // optional: fixed IP address, it is recommended to assign a fixed IP via the DHCP server instead
  // IPAddress ip(192,168,1,31); IPAddress gateway(192,168,1,1); IPAddress subnet(255,255,0,0); WiFi.config(ip, gateway, subnet);
  Serial.print("Attempting to connect to WiFi...");
  int seconds=0;
  while ((WiFi.status() != WL_CONNECTED)&&(seconds < 10)) {
    Serial.print(".");        
    delay(1000);
    seconds++;    
  }Serial.println();
  
  if(WiFi.status() != WL_CONNECTED) { 
    WiFireconnect(); // blocking function that continues to attempt connecting
  }else{
    
    Serial.println("Connected");
    Serial.println("IP address: " + IPAddressString(WiFi.localIP()));
  }
}

// WiFi reconnect
void WiFireconnect(void){
  while (WiFi.status() != WL_CONNECTED) {
    status_led = 3; // status led blinks fast
    Serial.print("WiFi is not available. Attempting to reconnect...");
    
    WiFi.disconnect();
    WiFi.reconnect();
    
    int seconds=0;
    while ((WiFi.status() != WL_CONNECTED)&&(seconds < 10)) {
      Serial.print(".");        
      delay(1000);   
      seconds++; 
    }Serial.println();
  }
  
  Serial.println("Connected");
  Serial.println("IP address: " + IPAddressString(WiFi.localIP()));

  status_led = 1; // we're connected again, set status led on
}



// =====================================
// ============= MQTT stuff ============
// =====================================
// connect and subscribe
void MQTTconnect(void) {
  if ((!mqttclient.connected())) {
    status_led = 3; // status led blinks fast
    Serial.println("Attempting MQTT connection...");

    // Attempt to connect
    int retVal;
    if(strcmp(MQTT_USERNAME, "") == 0){
      retVal=mqttclient.connect(NODE_NAME);
    }else{
      retVal=mqttclient.connect(NODE_NAME,MQTT_USERNAME,MQTT_PASSWORD);
    }

    if (retVal) {
      status_led = 1; // status led on
      Serial.println("Connected");
      mqttclient.publish(GetTopic("ip"), IPAddressString(WiFi.localIP()));
      
      mqttclient.subscribe(GetTopic("fan-speed-ref"));
      mqttclient.subscribe(GetTopic("fan-controlmode-ref"));
      mqttclient.subscribe(GetTopic("fan-boostmode-ref")); 
              
      if((SLAVE_MODE == 1)&&(strcmp(NODE_NAME,MASTER_NODE_NAME) !=0 )){
        mqttclient.subscribe(GetTopicMaster("fan-speed"));
        mqttclient.subscribe(GetTopicMaster("fan-enabled"));
      }

      mqtt_reconnects++;
    } else {
      Serial.println("Failed");
    }
  }
}

// handle connection and send messages at intervals
void handleMQTT(void) {
  static unsigned long prev_millis_mqtt = 0, prev_millis_autodiscovery = 0;

  if((WiFi.status() != WL_CONNECTED)){
    WiFireconnect();
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

      if(T1 > -100.0){
        mqttclient.publish(GetTopic("temp-inlet"), String(T1, 2));
      }else{
        mqttclient.publish(GetTopic("temp-inlet"), String("-"));        
      }

      if(T2 > -100.0){
        mqttclient.publish(GetTopic("temp-outlet"), String(T2, 2));
      }else{
        mqttclient.publish(GetTopic("temp-outlet"), String("-"));        
      }
      
      if((T1 > -100.0)&&(T2 > -100.0)){
        mqttclient.publish(GetTopic("temp-delta-io"), String(T1 - T2, 2));
      }else{
        mqttclient.publish(GetTopic("temp-delta-io"), String("-"));
      }        
      
      if(T0 > -100.0){
        mqttclient.publish(GetTopic("temp-ambient"), String(T0, 2));
      }else{
        mqttclient.publish(GetTopic("temp-ambient"), String("-"));        
      }

      if(ENABLE_HOME_ASSISTANT_AUTODISCOVERY==1){
        sendHomeAssistantAutodiscoveryMessages();
      }
      
    }
  }
}

// handle received MQTT messages
void handleMQTTreceive(String &topic, String &payload) {
  if (topic.indexOf(GetTopic("fan-boostmode-ref")) >= 0) {
    fan_boostmode = payload.toInt();
    mqttclient.publish(GetTopic("fan-boostmode"), String(fan_boostmode));
  }

  if (topic.indexOf(GetTopic("fan-controlmode-ref")) >= 0) {
    fan_controlmode = payload.toInt();
    mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
  }

  if(fan_controlmode == 1){ // manual mode, accept updates of setpoints
    if (topic.indexOf(GetTopic("fan-speed-ref")) >= 0) {
      fan_speed = payload.toInt();
      writeFanSpeed();
      mqttclient.publish(GetTopic("fan-speed"), String(fan_speed));
    }    
  
  }else if((SLAVE_MODE == 1)&&(fan_controlmode == 0)){ // automatic control mode, listen to the master

    if (topic.indexOf(GetTopicMaster("fan-speed")) >= 0) {
      fan_speed = payload.toInt();
      writeFanSpeed();
      mqttclient.publish(GetTopic("fan-speed"), String(fan_speed));
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
  homeassistantAddSensor("ip", "", "IP address", false, "mdi:map-marker");
}

void homeassistantAddSensor(String sensor_name, String unit_measurement, String name_measurement, bool is_controllable, String icon){
  DynamicJsonDocument doc(2048);
  
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

  JsonDocAddDevice(&doc);
  
  String HA_payload;
  serializeJson(doc, HA_payload);
  mqttclient.publish(HA_topic, HA_payload);
}


void homeassistantAddSwitch(String sensor_name, String name_measurement, bool is_controllable, String icon){
  DynamicJsonDocument doc(2048);
  
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

  JsonDocAddDevice(&doc);
  
  String HA_payload;
  serializeJson(doc, HA_payload);
  mqttclient.publish(HA_topic, HA_payload);
}

void JsonDocAddDevice(DynamicJsonDocument * doc){
  // add device parameters to JsonDocument
  (*doc)["device"]["name"] = String("ESPJagaBooster - ") + String(NODE_NAME);
  (*doc)["device"]["model"] = String("ESPJagaBooster");
  (*doc)["device"]["manufacturer"] = String("Bas Vermulst");
  (*doc)["device"]["identifiers"] = String(NODE_NAME);
  (*doc)["device"]["sw_version"] = String(FW_VERSION);
}



// =====================================
// ===== Over-the-air update stuff =====
// =====================================
void initOTA(void){
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(node_hostname.c_str());
  // If a programming password is desired, uncomment line below to set the OTA password (password is "1234" in this case):
  // ArduinoOTA.setPassword("1234");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    
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
}


// =====================================
// =========== Fan control =============
// =====================================
void initFan(void){
  // init PWM ESP8266 (interrupt-based)
  analogWriteRange(100);
  analogWriteFreq(FAN_PWM_FREQUENCY);
  analogWrite(FAN_PIN, 0);    
}

void writeFanSpeed(void){
  if(fan_speed > 100)
    fan_speed = 100;
  if(fan_speed < 0)
    fan_speed = 0;

  if(fan_enabled){
    int dutycycle=(int) ((float) fan_speed * 0.8 + (float) 20.0); // rescale, the PWM standard dictates 20% dutycycle = 0% speed
    analogWrite(FAN_PIN, 100-dutycycle);
  }else{    
    analogWrite(FAN_PIN, 100); // no PWM
  }
  digitalWrite(FAN_POWER_PIN, fan_enabled);
}



// =====================================
// ========= Helper functions ==========
// =====================================
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

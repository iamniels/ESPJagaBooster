/*
   Software for controlling fan-boosted radiators.
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
   Install via "Manage Libraries..." under Tools.

   3) Configure board:
   Select Tools --> Board --> "ESP8266 boards" --> LOLIN(WEMOS) D1 R2 & mini

   4) Configure programming port:
   Connect board via USB cable.
   Select appropriate programming port via Tools --> Port

   === usage ===
   you can send the following commands to the board via MQTT:
      - NODE_NAME/fan-speedmode-ref: 0 = silent mode, 1 = boost mode
      - NODE_NAME/fan-controlmode-ref: 0 = automatic fan speed, 1 = manual fan speed
      - NODE_NAME/fan-dutycycle-ref: value between 0 - 100 to control fan speed (only in manual fan speed)

   the board sends the following status messages via MQTT:
      - NODE_NAME/ip: ip-address
      - NODE_NAME/ssid: WiFi SSID
      - NODE_NAME/rssi: WiFi signal strength

      - NODE_NAME/interval: MQTT update interval
      - NODE_NAME/runtime: time the board has been powered on
      - NODE_NAME/reconnects: reconnection attempts since power on/reset

      - NODE_NAME/fan-dutycycle: current fan speed (0 - 100)
      - NODE_NAME/fan-controlmode: current control mode (0 = auto, 1 = manual)
      - NODE_NAME/fan-speedmode: current speed mode (0 = silent, 1 = boost)
      - NODE_NAME/fan-enable: fan power status (0 = power off)

      - NODE_NAME/temp-inlet: inlet temperature in deg C
      - NODE_NAME/temp-outlet: outet temperature in deg C
      - NODE_NAME/temp-delta-io: delta between inlet and outlet in deg C
      - NODE_NAME/temp-ambient: ambient temperature in deg C
*/

///////// CONFIGURATION ///////// 

// Node
#define NODE_NAME "radiator-wk-voor"
#define STANDALONE_MODE 0 // if set to 1, the board runs without WiFi and MQTT
#define SLAVE_MODE 0 // use control input from other node (only works if STANDALONE_MODE == 0)
#define MASTER_NODE_NAME "radiator-wk-voor" // node name of master (for slave mode)

// WiFi
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

// MQTT
#define MQTTSERVER "192.168.1.11"
#define MQTT_USERNAME "" // leave empty if no credentials are needed
#define MQTT_PASSWORD "" 

// fan speed controller tuning
// (heating)
#define FAN_CONTROL_START 27.5 // degrees C inlet temperature (start of control range, duty cycle = 0 %)
#define FAN_CONTROL_FULL 45.0 // degrees C inlet temperature (end of control range, duty cycle = limit %)
#define FAN_DUTYCYCLE_LIMIT 60 // max fan speed in normal mode
#define FAN_DUTYCYCLE_BOOST_LIMIT 100 // max fan speed in boost mode

#define FAN_ENABLE_T_OFF 27.5 // temperature below which to disable fans
#define FAN_ENABLE_DELTA_T_ON 5.0 // delta inlet-ambient to turn on fans
#define FAN_ENABLE_DELTA_T_OFF 4.0 // delta inlet-ambient to turn off fans

// fan speed controller tuning
// (cooling)
#define FAN_DUTYCYCLE_COOLING 55 // fan speed in cooling mode (cooling uses a constant fan speed)
#define FAN_ENABLE_DELTA_T_ON_COOLING 2.5 // delta inlet-ambient to turn on fans for cooling
#define FAN_ENABLE_DELTA_T_OFF_COOLING 1.5 // delta inlet-ambient to turn off fans for cooling

#define FAN_OFF_DELAY 10*60 // delay before fans are switched off (heating & cooling)


// NTC
#define NTC_BETA 3950



////////// DO NOT TOUCH ////////// 

// includes
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h> // for OTA
#include <MQTT.h> // for MQTT
#include <WiFiClient.h> // for MQTT
#include <ArduinoOTA.h> // for OTA
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// peripherals and IO
#define FAN_PIN D0
#define PWR_SW_PIN D5
#define V_BUS 3.3 // NTC resistor divider input voltage

#define TEMPERATURE_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz
#define SUPPLY_FILTER_COEFF 0.1 // 1st order iir filter @ fs = 4 Hz

// initialization
WiFiClient wificlient_mqtt;
MQTTClient mqttclient;
Adafruit_ADS1115 ads;

float battery_voltage = 0;
int fan_dutycycle = 0;
int fan_controlmode = 0; // 0 = automatic, 1 = manual
int fan_enable = 0;
int fan_speedmode = 0; // 0 = normal, 1 = boost

float water_in_temperature = 0;
float water_out_temperature = 0;
float T0, T1, T2, voltage_comp;

int mqtt_reconnects = 0;
int mqtt_interval = 5; //mqtt publishing interval in seconds



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
  analogWriteFreq(8000);
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
  static unsigned long prev_millis = 0;
  
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
    
    if (SLAVE_MODE == 0 || STANDALONE_MODE == 1){
      if (fan_controlmode == 0){
        // automatic fan speed and automatic enable/disable

        if(T0 > T1){ // we could be cooling
        
          if (T0 - T1 > FAN_ENABLE_DELTA_T_ON_COOLING) { // inlet lower than ambient, enable fans for cooling
            fan_enable = 1;
            fan_dutycycle = FAN_DUTYCYCLE_COOLING;
            fan_enabled_prev_millis=millis();
            
          } else if ( (T0 - T1 < FAN_ENABLE_DELTA_T_OFF_COOLING) && 
                      (InterruptPending(&fan_enabled_prev_millis, FAN_OFF_DELAY*1000, 1))
                      ){ // inlet higher than ambient, disable fans for cooling cooling
            fan_enable = 0;
            fan_dutycycle = 0;
            
          }
          
        }else{ // we could be heating

          if (T1 - T0 > FAN_ENABLE_DELTA_T_ON) { // inlet higher than ambient, enable fans
            fan_enable = 1;
            fan_dutycycle = FAN_DUTYCYCLE_LIMIT * (T1 - FAN_CONTROL_START) / (FAN_CONTROL_FULL - FAN_CONTROL_START);
            fan_enabled_prev_millis=millis();
            
            switch (fan_speedmode) {
              default:
              case 0:
                if (fan_dutycycle > FAN_DUTYCYCLE_LIMIT)
                  fan_dutycycle = FAN_DUTYCYCLE_LIMIT;
                break;
              case 1:
                if (fan_dutycycle > FAN_DUTYCYCLE_BOOST_LIMIT)
                  fan_dutycycle = FAN_DUTYCYCLE_BOOST_LIMIT;
                break;
            }
    
          } else if ( ((T1 - T0 < FAN_ENABLE_DELTA_T_OFF) || (T1 < FAN_ENABLE_T_OFF)) &&
                      (InterruptPending(&fan_enabled_prev_millis, FAN_OFF_DELAY*1000, 1))
                      ){ // inlet temp is too low, disable fans
            fan_enable = 0;
            fan_dutycycle = 0;
          }
        }
        
      } else {
        // manual fan speed control
        if(fan_dutycycle > 0){
          fan_enable = 1;
        }else{
          fan_enable = 0;
        }
      }
    }


    if (fan_dutycycle > 100)
      fan_dutycycle = 100;
    if (fan_dutycycle < 0)
      fan_dutycycle = 0;

    analogWrite(FAN_PIN, 100-fan_dutycycle);
    digitalWrite(PWR_SW_PIN, fan_enable);
  }
}

// ===== Handles for MQTT =====
// handle connection and send messages at intervals
void handleMQTT(void) {
  static unsigned long prev_millis = 0;
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi is not available. Waiting for WiFi connection.");
    delay(1000);
  }

  if (!mqttclient.connected()) {
    MQTTconnect();
  } else {
    mqttclient.loop();

    if (InterruptPending(&prev_millis, mqtt_interval * 1000, 1)) {
      Serial.println("Sending MQTT update");
      mqttclient.publish(GetTopic("ip"), IPAddressString(WiFi.localIP()));
      mqttclient.publish(GetTopic("ssid"), WiFi.SSID());
      mqttclient.publish(GetTopic("rssi"), String(WiFi.RSSI()));

      mqttclient.publish(GetTopic("interval"), String(mqtt_interval));
      mqttclient.publish(GetTopic("runtime"), String(millis() / 1000));
      mqttclient.publish(GetTopic("reconnects"), String(mqtt_reconnects));

      mqttclient.publish(GetTopic("fan-dutycycle"), String(fan_dutycycle));
      mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
      mqttclient.publish(GetTopic("fan-speedmode"), String(fan_speedmode));
      mqttclient.publish(GetTopic("fan-enable"), String(fan_enable));

      mqttclient.publish(GetTopic("temp-inlet"), String(T1, 2));
      mqttclient.publish(GetTopic("temp-outlet"), String(T2, 2));
      mqttclient.publish(GetTopic("temp-delta-io"), String(T1 - T2, 2));

      mqttclient.publish(GetTopic("temp-ambient"), String(T0, 2));
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

      if(SLAVE_MODE == 0){
        mqttclient.subscribe(GetTopic("fan-dutycycle-ref"));
        mqttclient.subscribe(GetTopic("fan-controlmode-ref"));
        mqttclient.subscribe(GetTopic("fan-speedmode-ref")); 
              
      }else if(SLAVE_MODE == 1){
        mqttclient.subscribe(GetTopicMaster("fan-dutycycle"));
        mqttclient.subscribe(GetTopicMaster("fan-controlmode"));
        mqttclient.subscribe(GetTopicMaster("fan-speedmode"));
        mqttclient.subscribe(GetTopicMaster("fan-enable"));
      }

      mqtt_reconnects++;
    } else {
      Serial.print("failed");
    }
  }
}

// handle received MQTT messages
void handleMQTTreceive(String &topic, String &payload) {

  if(SLAVE_MODE == 0){
    if (topic.indexOf(GetTopic("fan-dutycycle-ref")) >= 0) {
      fan_dutycycle = payload.toInt();
      mqttclient.publish(GetTopic("fan-dutycycle"), String(fan_dutycycle));
    }
  
    if (topic.indexOf(GetTopic("fan-controlmode-ref")) >= 0) {
      fan_controlmode = payload.toInt();
      mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
    }
  
    if (topic.indexOf(GetTopic("fan-speedmode-ref")) >= 0) {
      fan_speedmode = payload.toInt();
      mqttclient.publish(GetTopic("fan-speedmode"), String(fan_speedmode));
    }

    
  }else if(SLAVE_MODE == 1){
    if (topic.indexOf(GetTopicMaster("fan-dutycycle")) >= 0) {
      fan_dutycycle = payload.toInt();
      mqttclient.publish(GetTopic("fan-dutycycle"), String(fan_dutycycle));
    }
  
    if (topic.indexOf(GetTopicMaster("fan-controlmode")) >= 0) {
      fan_controlmode = payload.toInt();
      mqttclient.publish(GetTopic("fan-controlmode"), String(fan_controlmode));
    }
  
    if (topic.indexOf(GetTopicMaster("fan-speedmode")) >= 0) {
      fan_speedmode = payload.toInt();
      mqttclient.publish(GetTopic("fan-speedmode"), String(fan_speedmode));
    }    

    if (topic.indexOf(GetTopicMaster("fan-enable")) >= 0) {
      fan_enable = payload.toInt();
      mqttclient.publish(GetTopic("fan-enable"), String(fan_enable));
    }   
  }
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


#include <FS.h>                   //this needs to be first, or it all crashes and burns...
//find code at
//https://github.com/CurlyWurly-1/ESP8266-WIFIMANAGER-MQTT/blob/master/MQTT_with_WiFiManager.ino
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>

#include <stdio.h>
#include <string.h>

#include <Wire.h>
#include "vl53l1_api.h"

//OTA START
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
//OTA END

#include <MedianFilter.h>
int medianwindow = 15;
MedianFilter distancereadings(medianwindow, 0);

VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
int status;
int distance = 0 ;
String page = "";
String text = "";


int updatemode = 0;

unsigned int raw=0;
float volt=0.0;

//flag for saving data
bool shouldSaveConfig = false;

//const long oneSecond = 1000;  // a second is a thousand milliseconds
const long oneSecond = 1000000;  // a second is a 1 million microseconds
const long oneMinute = oneSecond * 60;
const long oneHour   = oneMinute * 60;
const long oneDay    = oneHour * 24;

const int sleepTimeS = 3600; //3600 1 hour in seconds - 30 minutes = 1800
const int sleepTimeDemo = 1800; //seconds

String StatusofLED = "off";

String macaddress = "";

//rgb
//12 //13 //14
#define redPin 13
#define greenPin 12
#define bluePin 14
#define buzzerPin 0


//define your default values here, if there are different values in config.json, they are overwritten.
//char mqtt_server[40];
#define mqtt_server       "ipaddress_of_your_mqtt_server"
#define mqtt_port         "1883"
#define mqtt_user         "username"
#define mqtt_pass         "password"
#define mqtt_topic_prefix "/lidar/"

String composeClientID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String clientId;
  
  clientId += macToStr(mac);
  return clientId;
}

String subscribetopicred =  mqtt_topic_prefix + composeClientID() + "/red";
String subscribetopicgreen =  mqtt_topic_prefix + composeClientID() + "/green";
String subscribetopicblue =  mqtt_topic_prefix + composeClientID() + "/blue";
String subscribetopiconoff =  mqtt_topic_prefix + composeClientID() + "/onoff";
String subscribetopicupdate =  mqtt_topic_prefix + composeClientID() + "/update";

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    
  }
  return result;
}





String mac; 

WiFiClient espClient;
PubSubClient client(espClient);

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println();
  
  uint8_t byteData;
  uint16_t wordData;

  text = (String)distance;

  //adc battery voltage
  pinMode(A0, INPUT);

  //turn off laser light nd remember to turn on before reading as well as turn off again before sleep
  pinMode(2, OUTPUT);
  //turn off
  digitalWrite(2, LOW); 
  

  Wire.begin();
  Wire.setClock(400000);

  Dev->I2cDevAddr = 0x52; 
  //Dev->I2cDevAddr = 0x29;

  VL53L1_software_reset(Dev);

  VL53L1_RdByte(Dev, 0x010F, &byteData);
  Serial.print(F("VL53L1X Model_ID: "));
  Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  Serial.print(F("VL53L1X Module_Type: "));
  Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.print(F("VL53L1X: "));
  Serial.println(wordData, HEX);

  Serial.println(F("Autonomous Ranging Test"));
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 33); // reduced to 50 ms from 500 ms in ST example
  status = VL53L1_StartMeasurement(Dev);



  //clean FS for testing
  //  SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  //WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 20);
  //WiFiManagerParameter custom_mqtt_pass("pass", "mqtt pass", mqtt_pass, 20);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // Reset Wifi settings for testing
  //  wifiManager.resetSettings();

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //  wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  //wifiManager.addParameter(&custom_mqtt_user);
  //wifiManager.addParameter(&custom_mqtt_pass);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);
  wifiManager.setDebugOutput(true);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  String apname = "AutoConnect" + composeClientID();
   
  if (!wifiManager.autoConnect(String(apname).c_str(), "rftagit")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.deepSleep(sleepTimeDemo * 1000000);
    //ESP.deepSleep(sleepTimeS * 1000000);
    //todo make sure we record reading and then when we do get a connect send data with time stamp
    //ESP.reset();

    //this means AP only shows up withn 5 seconds
    //delay(5000);
  }

  
  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  //WiFi.macAddress(mac);
  //GET MAC ADDRESS START
  Serial.print("MAC: ");
  macaddress = WiFi.macAddress();
  macaddress.toLowerCase();
  macaddress.replace(":", "");
  Serial.println(macaddress); 
  //GET MAC ADDRESS END

  //check for latest software and update OTA
  //OTA START
  if(updatemode == 1)
  {
    for (uint8_t t = 5; t > 0; t--) 
    {
      Serial.printf("[SETUP] WAIT %d...\n", t);
      Serial.flush();
      delay(1000);
    }
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://rftagit.co.za/scripts/otaupdate.php?deviceidentifier=" + macaddress);
  
    delay(500);
      Serial.println(ret);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.println("[update] Update failed.");
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[update] Update no Update.");
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[update] Update ok."); // might not be called since we reboot the ESP
        break;
    }

    String topic =  mqtt_topic_prefix + composeClientID() + "/update";
    int output = 0;
    client.publish(String(topic).c_str() , String(output).c_str(), false);
    
    //OTA END
  }
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  //strcpy(mqtt_user, custom_mqtt_user.getValue());
  //strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  // strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  //  client.setServer(mqtt_server, 12025);
  //const uint16_t mqtt_port_x = 12025;
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {

    String mqttclient = "ESP8266Client" + composeClientID();
   
  
    if (client.connect(String(mqttclient).c_str(), mqtt_user, mqtt_pass)) {
      client.subscribe(subscribetopiconoff.c_str());
      Serial.println(subscribetopiconoff.c_str());
      
      client.subscribe(subscribetopicred.c_str());
      Serial.println(subscribetopicred.c_str());
      
      client.subscribe(subscribetopicgreen.c_str());
      Serial.println(subscribetopicgreen.c_str());
      
      client.subscribe(subscribetopicblue.c_str());
      Serial.println(subscribetopicblue.c_str());

      //update 
      client.subscribe(subscribetopicupdate.c_str());
      Serial.println(subscribetopicupdate.c_str());
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float diff = 1.0;

long duration; // Duration used to calculate distance

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  char string[50] = "";
  char mqtt_topic[40] = "";

  uint8_t byteData;
  uint16_t wordData;

digitalWrite(2, HIGH); 
    delay(100);

    //check for latest software and update OTA
  //OTA START
  if(updatemode == 1)
  {
    for (uint8_t t = 5; t > 0; t--) 
    {
      Serial.printf("[SETUP] WAIT %d...\n", t);
      Serial.flush();
      delay(1000);
    }
    t_httpUpdate_return ret = ESPhttpUpdate.update("http://rftagit.co.za/scripts/otaupdate.php?deviceidentifier=" + macaddress);
  
    delay(500);
      Serial.println(ret);
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.println("[update] Update failed.");
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("[update] Update no Update.");
        break;
      case HTTP_UPDATE_OK:
        Serial.println("[update] Update ok."); // might not be called since we reboot the ESP
        break;
    }

    String topic =  mqtt_topic_prefix + composeClientID() + "/update";
    int output = 0;
    client.publish(String(topic).c_str() , String(output).c_str(), false);
    //OTA END
    
  }
    

  //read battery voltage
  raw = analogRead(A0);
  Serial.print(F("raw: "));
  Serial.println(raw);
  volt = raw / 1023.0;
  volt = 6.7 * volt;
  
  String v=String(volt);// change float into string
  
  
  Serial.print(F("Voltage: "));
  Serial.println(v);

  text = (String)distance;

  Wire.begin();
  Wire.setClock(400000);

  Dev->I2cDevAddr = 0x52;
  //Dev->I2cDevAddr = 0x29;


  VL53L1_software_reset(Dev);

  VL53L1_RdByte(Dev, 0x010F, &byteData);
  Serial.print(F("VL53L1X Model_ID: "));
  Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  Serial.print(F("VL53L1X Module_Type: "));
  Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.print(F("VL53L1X: "));
  Serial.println(wordData, HEX);

  Serial.println(F("Autonomous Ranging Test"));
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 33); // reduced to 50 ms from 500 ms in ST example
  int averagecount = 0;
  for (int counter = 0; counter < 50; counter++) 
  {
    
    status = VL53L1_StartMeasurement(Dev);
  
    static VL53L1_RangingMeasurementData_t RangingData;
  
    status = VL53L1_WaitMeasurementDataReady(Dev);
    if(!status)
    {
      status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
      if(status==0)
      {
        Serial.print(RangingData.RangeStatus);
        Serial.print(F(","));
        Serial.print(RangingData.RangeMilliMeter);
        distance = RangingData.RangeMilliMeter;
        distancereadings.in(distance);
        averagecount = averagecount + distance;
        Serial.print(F(","));
        Serial.print(RangingData.SignalRateRtnMegaCps/65536.0);
        Serial.print(F(","));
        Serial.println(RangingData.AmbientRateRtnMegaCps/65336.0);
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
    }
    else
    {
      Serial.print(F("error waiting for data ready: "));
      Serial.println(status); 
    }
  }

  
  //use the median value to get distance
  distance = distancereadings.out();
  distance = averagecount / 50;
  String topic =  mqtt_topic_prefix + composeClientID();
  String output = String(distance) + "," + v;
  Serial.println("outputstring : " + String(output));
   client.publish(String(topic).c_str() , String(output).c_str(), false);
  //delay(oneHour); //if you use this remember that 1 second has been converted to microseconds and not milliseconds up above in the code
  //Serial.println("ESP8266 in sleep mode");
  delay(100);
  digitalWrite(2, LOW); 
  delay(100);
  client.loop();

    
   
  
  
   //ESP.deepSleep(sleepTimeS * 1000000);
   ESP.deepSleep(sleepTimeDemo * 1000000);

}


void callback(char* topic, byte* payload, unsigned int length) {
  int value = 0;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("");
  
  //client.publish(String(topic).c_str() , String(StatusofLED).c_str(), false);
 
  payload[length] = '\0';
  String s = String((char*)payload);
  value = s.toInt();
  
  if (strcmp(topic,subscribetopiconoff.c_str())==0){
    Serial.println("onoff mqtt message received");
    // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    analogWrite(redPin, 255); 
    analogWrite(greenPin, 255); 
    analogWrite(bluePin, 255); 
    digitalWrite(buzzerPin, 1); 
    StatusofLED = "on";
  } else {
    analogWrite(redPin, 0); 
    analogWrite(greenPin, 0); 
    analogWrite(bluePin, 0); 
    digitalWrite(buzzerPin, 0); 
     StatusofLED = "off";
  }
  }
 
  if (strcmp(topic,subscribetopicred.c_str())==0) {
    //Serial.println(value);
    analogWrite(redPin, value); 
     StatusofLED = "on";
  }
 
  if (strcmp(topic,subscribetopicblue.c_str())==0) {
    //Serial.println(value);
    analogWrite(bluePin, value); 
     StatusofLED = "on";
  }  
 
  if (strcmp(topic,subscribetopicgreen.c_str())==0) {
    //Serial.println(value);
    analogWrite(greenPin, value); 
     StatusofLED = "on";
  }  

  //update flag from mqtt - if 0 dont bother if 1 do update routine
  if (strcmp(topic,subscribetopicupdate.c_str())==0)
  {
     if(value == 0)
     {
        //no updates
        updatemode = 0;
        Serial.println("Dont update");
     }
     else if (value == 1)
     {
        //go into update mode now 
        updatemode = 1;
        Serial.println("OTA Update");
     }
  }
}

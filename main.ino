#include <ESPAsyncTCP.h>
#include <AsyncMqtt_Generic.h>
#include <Ticker.h>                     // Ticker Library for ESP8266
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <ESP8266WebServer.h>
#include <Bounce2.h>

#define WIFI_SSID "Ammad_C-25"
#define WIFI_PASSWORD "ammad175"

#define mqtt_HOST IPAddress(192, 168, 1, 102)
#define mqtt_PORT 1883

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

//flag for saving data
bool shouldSaveConfig = false;

//LED on ESP8266 GPIO2
const int outlet1 = D4;
const int outlet2 = D3;
const int outlet3 = D1;
const int outlet4 = 3;
const int outlet1Button = D0;
const int outlet2Button = D5;
const int outlet3Button = D6;
const int outlet4Button = D7;
const int phaseDetect = D2;

//create an instance of the bounce class
Bounce myoutlet1Button = Bounce();
Bounce myoutlet2Button = Bounce();
Bounce myoutlet3Button = Bounce();
Bounce myoutlet4Button = Bounce();

// static var to keep track of the intended current outlet1 state
static boolean button1State = false;
static boolean button2State = false;
static boolean button3State = false;
static boolean button4State = false;

// variables used to store state and brightness
Ticker TimeMS;
int brightness=10;
int brightnessInv;
int freqStep = 1;    // This is the delay-per-brightness step in microseconds.
volatile int i=0;               // Variable to use as a counter volatile as it is in an interrupt
volatile boolean zero_cross=0;  // Boolean to store a "switch" to tell us if we have crossed zero

// buffer used to send/receive data with mqtt
char m_msg_buffer[4];

// mqtt: topics
// state
const char* roomTopic1 = "/room2/set/outlet1";
const char* roomConfirmTopic1 PROGMEM = "/room2/status/outlet1";
const char* roomTopic2 PROGMEM = "/room2/set/outlet2";
const char* roomConfirmTopic2 PROGMEM = "/room2/status/outlet2";
const char* roomTopic3 PROGMEM = "/room2/set/outlet3";
const char* roomConfirmTopic3 PROGMEM = "/room2/status/outlet3";
const char* roomTopic4 PROGMEM = "/room2/set/outlet4";
const char* roomConfirmTopic4 PROGMEM = "/room2/status/outlet4";

// brightness
const char* roomBrightnessTopic PROGMEM = "/room2/brightness/set";
const char* roomBrightnessConfirmTopic PROGMEM = "/room2/brightness/status";

void connectToWifi() {
  //Serial.println("Connecting to Wi-Fi...");
  WiFi.disconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//  WiFi.setAutoReconnect(true);
//  WiFi.persistent(true);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi");
  connectTomqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to mqtt while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectTomqtt() {
  Serial.println("Connecting to mqtt...");
  mqttClient.connect();
}

void onmqttConnect(bool sessionPresent) {
  Serial.println("Connected to mqtt.");
  mqttClient.subscribe(roomTopic1, 0);
  mqttClient.subscribe(roomBrightnessTopic, 0);
  mqttClient.subscribe(roomTopic2, 0);
  mqttClient.subscribe(roomTopic3, 0);
  mqttClient.subscribe(roomTopic4, 0);
}

void onmqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from mqtt.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(10, connectTomqtt);
  }
}

void onmqttSubscribe(uint16_t packetId, uint8_t qos) {
}

void onmqttUnsubscribe(uint16_t packetId) {
}

void onmqttMessage(char* p_topic, char* p_payload, AsyncMqttClientMessageProperties properties, size_t p_length, size_t index, size_t total) {
  //convert topic to string to make it easier to work with
  String topic = p_topic;

  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  // handle message topic
  if (String(roomTopic1).equals(p_topic)) {
    if (payload.equals(String("a"))) {
      if(brightness>=9) {
        TimeMS.detach();
        detachInterrupt(phaseDetect);
        digitalWrite(outlet1, LOW);
        //Serial.println("outlet 1 turned on at full");
      }
      if (brightness>=0 && brightness<=8){
        TimeMS.attach_ms(freqStep, dim_check);
        attachInterrupt(phaseDetect, zero_crosss_int, RISING);
        //Serial.println("outlet 1 turned on at partial");  
      }
      publishBrightness();
      mqttClient.publish(roomConfirmTopic1, 0, false, "a");
      //button1State = true;
    }

    else if (payload.equals(String("b"))) {
      TimeMS.detach();
      detachInterrupt(phaseDetect);
      digitalWrite(outlet1, HIGH);
      //Serial.println("outlet 1 turned off");
      mqttClient.publish(roomConfirmTopic1, 0, false, "b");
      //button1State = false;
    }
  }
  
  if (String(roomTopic2).equals(p_topic)) {
    if (payload.equals(String("c"))) {
      digitalWrite(outlet2, HIGH);
      ////Serial.println("outlet 2 high");
      mqttClient.publish(roomConfirmTopic2, 0, false, "c");
      //button2State = true;
    }

    else if (payload.equals(String("d"))) {
      digitalWrite(outlet2, LOW);
      ////Serial.println("outlet 2 low");
      mqttClient.publish(roomConfirmTopic2, 0, false, "d");
      //button2State = false;
    }
  }
  
  if (String(roomTopic3).equals(p_topic)) {
    if (payload.equals(String("e"))) {
      digitalWrite(outlet3, HIGH);
      ////Serial.println("outlet 3 high");
      mqttClient.publish(roomConfirmTopic3, 0, false, "e");
      //button3State = true;
    }

    else if (payload.equals(String("f"))) {
      digitalWrite(outlet3, LOW);
      ////Serial.println("outlet 3 low");
      mqttClient.publish(roomConfirmTopic3, 0, false, "f");
      //button3State = false;
    }
  }
  if (String(roomTopic4).equals(p_topic)) {
    if (payload.equals(String("g"))) {
      digitalWrite(outlet4, HIGH);
      ////Serial.println("outlet 4 high");
      mqttClient.publish(roomConfirmTopic4, 0, false, "g");
      //button4State = true;
    }

    else if (payload.equals(String("h"))) {
      digitalWrite(outlet4, LOW);
      ////Serial.println("outlet 4 low");
      mqttClient.publish(roomConfirmTopic4, 0, false, "h");
      //button4State = false;
    }
  }

  if (String(roomBrightnessTopic).equals(p_topic)) {
    brightness = payload.toInt();
    brightnessInv = 10 - brightness;
    publishBrightness();
    //Serial.println(brightness);
  }
}

void onmqttPublish(uint16_t packetId) {
  //Serial.println("Publish acknowledged.");
  //Serial.print("  packetId: ");
  //Serial.println(packetId);
}

void mqttsetup(){
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onmqttConnect);
  mqttClient.onDisconnect(onmqttDisconnect);
  mqttClient.onSubscribe(onmqttSubscribe);
  mqttClient.onUnsubscribe(onmqttUnsubscribe);
  mqttClient.onMessage(onmqttMessage);
  mqttClient.onPublish(onmqttPublish);
  mqttClient.setServer(mqtt_HOST, mqtt_PORT);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  WiFi.mode(WIFI_STA);
  connectToWifi();
  mqttsetup();
  
  //initialize the outlet1 as an output and set to LOW (off)
  pinMode(outlet1, OUTPUT);
  pinMode(outlet2, OUTPUT);
  pinMode(outlet3, OUTPUT);
  pinMode(outlet4, OUTPUT);

  pinMode(outlet1Button, INPUT_PULLUP);
  pinMode(outlet2Button, INPUT_PULLUP);
  pinMode(outlet3Button, INPUT_PULLUP);
  pinMode(outlet4Button, INPUT_PULLUP);

  myoutlet1Button.attach(outlet1Button);
  myoutlet1Button.interval(250);
  myoutlet2Button.attach(outlet2Button);
  myoutlet2Button.interval(250);
  myoutlet3Button.attach(outlet3Button);
  myoutlet3Button.interval(250);
  myoutlet4Button.attach(outlet4Button);
  myoutlet4Button.interval(250);
  
  if(digitalRead(outlet1Button)==0) {
    //mqttClient.publish(roomBrightnessConfirmTopic, 0, false, "10");
    //mqttClient.publish(roomConfirmTopic1, 0, false, "a");
    digitalWrite(outlet1, LOW); 
  }
  else if(digitalRead(outlet1Button)==1) {
    //mqttClient.publish(roomConfirmTopic1, 0, false, "b");
    digitalWrite(outlet1, HIGH);
  }  
  
  if(digitalRead(outlet2Button)==0) {
    //mqttClient.publish(roomConfirmTopic2, 0, false,  "c");
    digitalWrite(outlet2, HIGH);
  }
  else if(digitalRead(outlet2Button)==1) {
    //mqttClient.publish(roomConfirmTopic2, 0, false,  "d");
    digitalWrite(outlet2, LOW);
  }
   
  if(digitalRead(outlet3Button)==0) {
    //mqttClient.publish(roomConfirmTopic3, 0, false,  "e");
    digitalWrite(outlet3, HIGH);
  } 
  else if(digitalRead(outlet3Button)==1) {
    //mqttClient.publish(roomConfirmTopic3, 0, false,  "f"); 
    digitalWrite(outlet3, LOW);
  }
  
  if(digitalRead(outlet4Button)==0) {
    //mqttClient.publish(roomConfirmTopic4, 0, false,  "g");
    digitalWrite(outlet4, HIGH);
  }
  else if(digitalRead(outlet4Button)==1) {
    //mqttClient.publish(roomConfirmTopic4, 0, false,  "h");
    digitalWrite(outlet4, LOW);
  }
}

void loop() {
  //monitor the button
  checkoutlet1Button();
  checkoutlet2Button();
  checkoutlet3Button();
  checkoutlet4Button();
}


void publishBrightness() {
  sprintf(m_msg_buffer, "%d", brightness);
  mqttClient.publish(roomBrightnessConfirmTopic, 0, false, m_msg_buffer);
}

ICACHE_RAM_ATTR void zero_crosss_int() {    
  zero_cross = true;               
  i=0;
  digitalWrite(outlet1, HIGH);       // turn off TRIAC (and AC)
  //Serial.println("ZC Detected");
}                       

// Turn on the TRIAC at the appropriate time
void dim_check() {                   
  if(zero_cross == true) {              
    if(i>=brightnessInv) {
                          
      digitalWrite(outlet1, LOW); // turn on light       
      //i=0;  // reset time step counter                         
      zero_cross = false; //reset zero cross detection
    } 
    else {
      i++; // increment time step counter  
      //Serial.println(i);                   
    }                                
  }                                  
}

void checkoutlet1Button() {
  //static boolean isOn = false;  // static var to keep track of the intended current outlet1 state
  if (myoutlet1Button.update()) {
    if (myoutlet1Button.fell()) { //update the button and check for HIGH or LOW state
      if(brightness>=9) {
        TimeMS.detach();
        detachInterrupt(phaseDetect);
        digitalWrite(outlet1, LOW);
        //Serial.println("outlet 1 turned on at full");
      }
      if (brightness>=0 && brightness<=8){
        TimeMS.attach_ms(freqStep, dim_check);
        attachInterrupt(phaseDetect, zero_crosss_int, RISING);
        //Serial.println("outlet 1 turned on at partial");  
      }
      publishBrightness();
      mqttClient.publish(roomConfirmTopic1, 0, false, "a");
      //button1State = true;
    }

    //else (on true), the outlet1 is on so tell it to turn off and set the internal var to false
    else if (myoutlet1Button.rose()) {
      TimeMS.detach();
      detachInterrupt(phaseDetect);
      digitalWrite(outlet1, HIGH);
      //Serial.println("outlet 1 turned off");
      mqttClient.publish(roomConfirmTopic1, 0, false, "b");
      //button1State = false;
    }
  }
}

void checkoutlet2Button() {
  //static boolean isOn = false;  //static var to keep track of the intended current outlet1 state
  if (myoutlet2Button.update()) {
    if (myoutlet2Button.fell()) { //update the button and check for HIGH or LOW state
      mqttClient.publish(roomConfirmTopic2, 0, false, "c");
      digitalWrite(outlet2, HIGH);
      //Serial.println("outlet 2 turned on");
    }

    //else (on true), the outlet1 is on so tell it to turn off and set the internal var to false
    else if (myoutlet2Button.rose()) {
      mqttClient.publish(roomConfirmTopic2, 0, false, "d");
      digitalWrite(outlet2, LOW);
      //Serial.println("outlet 2 turned off");
    }
  }
}

void checkoutlet3Button() {
  //static boolean isOn = false;  //static var to keep track of the intended current outlet1 state
  if (myoutlet3Button.update()) {
    if (myoutlet3Button.fell()) { //update the button and check for HIGH or LOW state
      mqttClient.publish(roomConfirmTopic3, 0, false, "e");
      digitalWrite(outlet3, HIGH);
      //Serial.println("outlet 3 turned on");
    }

    //else (on true), the outlet1 is on so tell it to turn off and set the internal var to false
    else if (myoutlet3Button.rose()) {
      mqttClient.publish(roomConfirmTopic3, 0, false, "f");
      digitalWrite(outlet3, LOW);
      //Serial.println("outlet 4 turned off");
    }
  }
}

void checkoutlet4Button() {
  //static boolean isOn = false;  //static var to keep track of the intended current outlet1 state
  if (myoutlet4Button.update()) {
    if (myoutlet4Button.fell()) { //update the button and check for HIGH or LOW state
      mqttClient.publish(roomConfirmTopic4, 0, false, "g");
      digitalWrite(outlet4, HIGH);
      //Serial.println("outlet 4 turned on");
    }

    //else (on true), the outlet1 is on so tell it to turn off and set the internal var to false
    else if (myoutlet4Button.rose()) {
      mqttClient.publish(roomConfirmTopic4, 0, false,  "h");
      digitalWrite(outlet4, LOW);
      //Serial.println("outlet 4 turned off");
    }
  }
}

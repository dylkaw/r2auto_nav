//V3 worksyay :)
#include <ESP32Servo.h>

const int pushButtonPin_1 = 15;
const int pushButtonPin_2 = 16;
const int pushButtonPin_3 = 18;
const int pushButtonPin_4 = 21;
const int pushButtonPin_5 = 23;
const int pushButtonPin_6 = 14;
const int ledPin_1 = 4;
const int ledPin_2 = 17; 
const int ledPin_3 = 19; 
const int ledPin_4 = 22;
const int ledPin_5 = 13; 
const int ledPin_6 = 27; 
//const int ledPin_7 = 33;


#define SERVO_PIN 26 // ESP32 pin GIOP26 connected to servo motor

Servo servoMotor;

/************************MQTT INSERT START**/
/******************************************************************************************/

#include "PubSubClient.h"
#include "WiFi.h" 

// WiFi
const char* ssid = "YOUR_SSID";
const char* wifi_password = "YOUR_PASSORD";


// MQTT
const char* mqtt_server = "172.20.10.8"; // CHANGE TO YOUR MQTT BROKER IP
const char* TABLE_TOPIC = "table"; 
const char* clientID = "ros2mqtt"; // MQTT client ID

 // Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// 1883 is the listener port for the Broker
PubSubClient client(mqtt_server, 1883, wifiClient);

void connect_MQTT(){
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);

  // Wait until the connection is confirmed
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Debugging – Output the IP Address of the ESP8266
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Connect to MQTT Broker
  if (client.connect(clientID) {
    Serial.println("Connected to MQTT Broker (computer)!");
  }
  else {
    Serial.println("Connection to MQTT Broker (computer) failed…");
  }
}

void sendToCom(char* topic, char data){
  
  Serial.setTimeout(2000);

  //ESP32 gathers data to be sent to RPi
  const char h = data; 

  Serial.print(topic);
  Serial.print(": ");
  Serial.print(h); 

  // MQTT can only transmit strings
  String hs=String(h);

  if (client.publish(topic, String(h).c_str())) {
    Serial.println("Data sent!");
  }
  else {
    Serial.println("Data failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(clientID, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn’t clash with the client.connect call
    client.publish(topic, String(h).c_str());
  }
  //client.disconnect();  // disconnect from the MQTT broker
  delay(1000);
}

/************************MQTT INSERT END**/
/******************************************************************************************/

void setup() {
  pinMode(pushButtonPin_1, INPUT);
  pinMode(pushButtonPin_2, INPUT);
  pinMode(pushButtonPin_3, INPUT);
  pinMode(pushButtonPin_4, INPUT);
  pinMode(pushButtonPin_5, INPUT);
  pinMode(pushButtonPin_6, INPUT); 
  pinMode(ledPin_1, OUTPUT);
  pinMode(ledPin_2, OUTPUT);
  pinMode(ledPin_3, OUTPUT);
  pinMode(ledPin_4, OUTPUT);
  pinMode(ledPin_5, OUTPUT);
  pinMode(ledPin_6, OUTPUT); 

  servoMotor.attach(SERVO_PIN , 771 , 2740); // 2470 attaches the servo on ESP32 pin
  Serial.begin(115200);

  connect_MQTT();  /**MQTT**/ 
}

void loop() { 

    if(digitalRead(pushButtonPin_1) == HIGH){
      digitalWrite(ledPin_1, HIGH);
      servoMotor.write(0);
      delay(1000);
      servoMotor.write(175);
      delay(1000);
      sendToCom("table", '1');
      digitalWrite(ledPin_1, LOW); 
    }else if(digitalRead(pushButtonPin_2) == HIGH){
      digitalWrite(ledPin_2, HIGH);
      servoMotor.write(0);
      delay(1000);
      servoMotor.write(175);
      delay(1000);
      sendToCom("table", '2');
      digitalWrite(ledPin_2, LOW); 
    }else if(digitalRead(pushButtonPin_3) == HIGH){
        digitalWrite(ledPin_3, HIGH);
        servoMotor.write(0);
        delay(1000);
        servoMotor.write(175);
        delay(1000);
        sendToCom("table", '3');
        digitalWrite(ledPin_3, LOW); 
    }else if(digitalRead(pushButtonPin_4) == HIGH){
      digitalWrite(ledPin_4, HIGH);
      servoMotor.write(0);
      delay(1000);
      servoMotor.write(175);
      delay(1000);
      sendToCom("table", '4');
      digitalWrite(ledPin_4, LOW); 
    }else if(digitalRead(pushButtonPin_5) == HIGH){
      digitalWrite(ledPin_5, HIGH);
      servoMotor.write(0);
      delay(1000);
      servoMotor.write(175);
      delay(1000);
      sendToCom("table", '5');
      digitalWrite(ledPin_5, LOW); 
    }else if(digitalRead(pushButtonPin_6) == HIGH){
      digitalWrite(ledPin_6, HIGH);
      servoMotor.write(0);
      delay(1000);
      servoMotor.write(175);
      delay(1000);
      sendToCom("table", '6');
      digitalWrite(ledPin_6, LOW); 
    }

  
}

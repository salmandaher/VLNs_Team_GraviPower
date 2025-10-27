#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#define load1 16
#define load2 17

#define voltPin 33

#define isCar 23
#define high 22

#define btsPin 32

int bts_v =170;

// ---------- Wi-Fi + ROS ------------
const char* ssid     = "Salman";
const char* password = "00002222";

// ROS master (rosserial TCP server) IP and port
IPAddress server(192, 168, 43, 47);  // RPi IP
const uint16_t serverPort = 11411;




ros::NodeHandle nh;
std_msgs::Bool str_msg;

String mode, last;
int tmp=0 , t=-10000 , last_time=0, vol=0;

void messageCb(const std_msgs::String& msg){
mode = msg.data;
}
void btsCb(const std_msgs::Int32& msg) {
bts_v=msg.data;
}


ros::Publisher chatter("/car_passed", &str_msg);
ros::Subscriber<std_msgs::String> sub("mode", messageCb );
ros::Subscriber<std_msgs::Int32> btss("bts", &btsCb );
char msg_buffer[64];

// ---------- Lobot Servo ------------
#include "LobotSerialServoControl.h"

#define SERVO_SERIAL_RX    35
#define SERVO_SERIAL_TX    12
#define receiveEnablePin   13
#define transmitEnablePin  14

// Use UART2 on ESP32
HardwareSerial LobotUART(2);
LobotSerialServoControl BusServo(LobotUART, receiveEnablePin, transmitEnablePin);

// Servo parameters
const uint8_t SERVO_ID = 2;

// ---------- Motion control ----------
bool start_en = true;
unsigned long last_pub_ms = 0;
const uint32_t pub_interval_ms = 1000;

// Helper: read servo position safely
int readServoPosition(uint8_t id) {
  // Library function returns int; commonly -1 on failure
  int pos = BusServo.LobotSerialServoReadPosition(id);
  return pos;
}

void switch2bump() {
  BusServo.LobotSerialServoMove(SERVO_ID, 500, 200);
  delay(400);
  BusServo.LobotSerialServoStopMove(SERVO_ID);
}

void switch2motor() {
  BusServo.LobotSerialServoMove(SERVO_ID, 800, 200);
  delay(400);
  BusServo.LobotSerialServoStopMove(SERVO_ID);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // ----- Wi-Fi -----
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // ----- Servo bus init -----
  // Initialize Lobot control (sets direction pins)
  BusServo.OnInit();
  // Bring up UART2 for servo bus
  LobotUART.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
  delay(200);

  // Optionally move once as in original setup
  BusServo.LobotSerialServoMove(SERVO_ID, 700, 600);
  delay(2000);

  // ----- ROS (rosserial TCP) -----
  nh.getHardware()->setConnection(server, serverPort);
  Serial.println("Attempting ROS connection...");
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  nh.subscribe(btss);

  pinMode(load1,OUTPUT);
  pinMode(load2,OUTPUT);

  pinMode(voltPin,OUTPUT);

  pinMode(high,OUTPUT);

  pinMode(isCar,INPUT_PULLUP);


  pinMode(btsPin, OUTPUT);

  digitalWrite(load1, 1);
  digitalWrite(load2, 1);
  digitalWrite(high, 1);


  analogWrite(btsPin, bts_v);
  attachInterrupt(digitalPinToInterrupt(isCar),OnLow,FALLING);

}

void loop() {
analogWrite(btsPin, bts_v);


control();
  if(millis()-t>1000 && last_time==1){
    str_msg.data=0;
    last_time=0;
    chatter.publish( &str_msg );
  }
  delay(100);
  nh.spinOnce();
}


void control(){
if (mode != last){
  if (mode == "charge"){
    switch2bump();
    digitalWrite(2,1);
    digitalWrite(load1, 1);
    digitalWrite(load2, 1);
    delay(50);
    }
//
  else if (mode == "discharge"){
    switch2motor();
    digitalWrite(load1, 0);
    digitalWrite(load2, 0);
    delay(50);

    }
    else if (mode == "fast"){
    switch2motor();
    digitalWrite(load1, 1);
    digitalWrite(load2, 1);
    delay(50);

    }
    else if (mode == "maxCharge"){
    switch2bump();
    digitalWrite(load1, 1);
    digitalWrite(load2, 1);
    delay(50);

    }
    last = mode;
    }
    
  }
  
void OnLow()
{
  if((millis()-t>1000 && last_time==0)){
    t = millis();
    str_msg.data=1; 
    last_time=1;
    chatter.publish( &str_msg );

  }

  
}
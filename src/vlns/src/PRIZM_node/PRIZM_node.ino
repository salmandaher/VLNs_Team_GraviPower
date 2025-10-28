#include <PRIZM.h>          
#include <ros.h>
#include <std_msgs/String.h>


#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

const int isCar =2;
int t=10000, last_time=0;

PRIZM prizm; 
ros::NodeHandle  nh;
std_msgs::Float32 enc;
std_msgs::Bool str_msg;


String mode;
int tmp=0;

void messageCb(const std_msgs::String& msg){
mode = msg.data;
}

ros::Publisher encoder_pub("/energy_storage/encoder", &enc);
ros::Publisher chatter("/car_passed", &str_msg);
ros::Subscriber<std_msgs::String> sub("mode", messageCb );





void setup() {

  Serial.begin(57600);
  prizm.PrizmBegin();         
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_pub);
  nh.advertise(chatter);
  delay(300);
  prizm.setMotorPower(2,0);
  delay(300);
  enc.data = 0.0;
  encoder_pub.publish( &enc );
  pinMode(isCar, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(isCar), onLow, FALLING);
    
}

void control(){

  if (mode == "charge"){
    prizm.setMotorPower(2,0);
    prizm.setRedLED(LOW);
    prizm.setGreenLED(HIGH);
    delay(600);
    prizm.setGreenLED(LOW);
    delay(600);
      prizm.setServoPosition(1,90);  
    }

  else if (mode == "discharge"){
    prizm.setMotorPower(2,0);
    prizm.setRedLED(HIGH);
    prizm.setGreenLED(LOW);
    delay(600);
    prizm.setRedLED(LOW);
    delay(600);
    prizm.setServoPosition(1,30);  
    }
    else if (mode == "fast"){
      delay(200);
    prizm.setMotorPower(2,50);
    prizm.setGreenLED(LOW);
    prizm.setRedLED(HIGH);    
    prizm.setServoPosition(1,90);  
    }
    else if (mode == "maxCharge"){
    prizm.setMotorPower(2,0);
    prizm.setRedLED(LOW);
    prizm.setGreenLED(HIGH);  
    prizm.setServoPosition(1,30);  
  
    }
    tmp = prizm.readEncoderCount(2);
    enc.data = abs(tmp/1000.0);
    encoder_pub.publish( &enc );
    
  }

  
  

void loop() {
control();

if(millis()-t>1500 && last_time==1){
  str_msg.data=0;
  last_time=0;
  chatter.publish(&str_msg);
}
delay(100);
nh.spinOnce();
}

void onLow()
{
  if(millis()-t>1500 && last_time==0){
      t=millis();
      str_msg.data=1;
      last_time=1;
      chatter.publish(&str_msg);
  }
  
}

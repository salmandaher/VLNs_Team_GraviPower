#include <PRIZM.h>          
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>




PRIZM prizm; 
ros::NodeHandle  nh;
std_msgs::Float32 enc;

String mode;
int tmp=0;

void messageCb(const std_msgs::String& msg){
mode = msg.data;
}

ros::Publisher encoder_pub("/energy_storage/encoder", &enc);
ros::Subscriber<std_msgs::String> sub("mode", messageCb );





void setup() {
  Serial.begin(57600);
  prizm.PrizmBegin();         
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(encoder_pub);
  delay(300);
  prizm.setMotorPower(2,0);
  delay(300);
  enc.data = 0.0;
  encoder_pub.publish( &enc );
    
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
    prizm.setServoPosition(1,10);  
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
    prizm.setServoPosition(1,10);  
  
    }
    tmp = prizm.readEncoderCount(2);
    enc.data = abs(tmp/1000.0);
    encoder_pub.publish( &enc );
    
  }

  
  

void loop() {
control();
nh.spinOnce();
}

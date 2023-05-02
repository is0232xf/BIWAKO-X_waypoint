#include <Wire.h>
#include <Servo.h>
byte servoPin1 = 3;
byte servoPin2 = 5;
byte servoPin3 = 6;
byte servoPin4 = 9;

int rc_pin1 = 2;
int rc_pin2 = 4;
int rc_pin3 = 7;
int rc_pin4 = 8;
unsigned int rc_pin1_duration;
unsigned int rc_pin2_duration;
unsigned int rc_pin3_duration;
unsigned int rc_pin4_duration;

int mode = 0; // 0: controller, 1: manual

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// chnage motor rotation direction 0 or 1
int motor1_dir = 0;
int motor2_dir = 0;
int motor3_dir = 1;
int motor4_dir = 0;

void setup() {
    // put your setup code here, to run once:
    int SLAVE_ADDRESS = 0x04;
    Serial.begin(9600);
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(ReceiveMessage);

    servo1.attach(servoPin1);
    servo2.attach(servoPin2);
    servo3.attach(servoPin3);
    servo4.attach(servoPin4);
    servo1.writeMicroseconds(1500);
    servo2.writeMicroseconds(1500);
    servo3.writeMicroseconds(1500);
    servo4.writeMicroseconds(1500);

    Serial.print("Wait seven seconds....\n");
    delay(7000);
    Serial.print("Compleate thrusters set up \n");

}

void loop() {
    if(mode==0){
        rc_pin1_duration = duration_noise_cut(pulseIn(rc_pin1, HIGH));
        rc_pin2_duration = duration_noise_cut(pulseIn(rc_pin2, HIGH));
        rc_pin3_duration = duration_noise_cut(pulseIn(rc_pin3, HIGH));
        rc_pin4_duration = duration_noise_cut(pulseIn(rc_pin4, HIGH));
        if(motor1_dir == 1){
          rc_pin1_duration = 3000 - rc_pin1_duration;
        }
        else if(motor2_dir == 1){
          rc_pin2_duration = 3000 - rc_pin2_duration;
        }
        else if(motor3_dir == 1){
          rc_pin3_duration = 3000 - rc_pin3_duration;
        }
        else if(motor4_dir == 1){
          rc_pin4_duration = 3000 - rc_pin4_duration;
        }
        servo1.writeMicroseconds(rc_pin1_duration);
        servo2.writeMicroseconds(rc_pin2_duration);
        servo3.writeMicroseconds(rc_pin3_duration);
        servo4.writeMicroseconds(rc_pin4_duration);
    }
}

void ReceiveMessage(int n){
  int action = 0;
  int pwm = 0;
  mode = Wire.read();
  action = Wire.read();
  pwm = Wire.read();

  if(mode==0){
      return;
  }
  else if(mode==1){
      return;
  }
  else if(mode==2){
      thruster_control(action, pwm);
  }
}

int duration_noise_cut(int duration){
    if(1450 < duration && duration < 1550){
        duration = 1500;
    }
    return duration;
}

void thruster_control(int action, int pwm){
  int power = map(pwm, 0, 100, 1500, 1900);
  int rc_pin1_duration = power;
  int rc_pin2_duration = power;
  int rc_pin3_duration = power;
  int rc_pin4_duration = power;
  int neutral = 1500;

  if(motor1_dir == 1){
    rc_pin1_duration = 3000 - rc_pin1_duration;
  }
  else if(motor2_dir == 1){
    rc_pin2_duration = 3000 - rc_pin2_duration;
  }
  else if(motor3_dir == 1){
    rc_pin3_duration = 3000 - rc_pin3_duration;
  }
  else if(motor4_dir == 1){
    rc_pin4_duration = 3000 - rc_pin4_duration;
  }

  if(action==0){
    // Stop thrusters
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==1){
    // 4 thrusters drive mode (Positive)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==2){
    // 4 thrusters drive mode (Negative)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  else if(action==3){
    // 4 thrusters drive mode (Left)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==4){
    // 4 thrusters drive mode (Right)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  else if(action==5){
    // 4 thrusters drive mode (CW)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==6){
    // 4 thrusters drive mode (CCW)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  else if(action==7){
    // 2 thrusters drive mode (First quadrant)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==8){
    // 2 thrusters drive mode (Second quadrant)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==9){
    // 2 thrusters drive mode (Third quadrant)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==10){
    // 2 thrusters drive mode (Forth quadrant)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  else if(action==11){
    // 2 thrusters drive mode(Push) (Positive)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==12){
    // 2 thrusters drive mode(Push) (Negative)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==13){
    // 2 thrusters drive mode(Push) (Left)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(rc_pin2_duration);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(rc_pin4_duration);
  }
  else if(action==14){
    // 2 thrusters drive mode(Pull) (Right)
    servo1.writeMicroseconds(rc_pin1_duration);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(rc_pin3_duration);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==15){
    // 2 thrusters drive mode(Pull) (Positive)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==16){
    // 2 thrusters drive mode(Pull) (Negative)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  else if(action==17){
    // 2 thrusters drive mode(Pull) (Left)
    servo1.writeMicroseconds(3000-rc_pin1_duration);
    servo2.writeMicroseconds(neutral);
    servo3.writeMicroseconds(3000-rc_pin3_duration);
    servo4.writeMicroseconds(neutral);
  }
  else if(action==18){
    // 2 thrusters drive mode(Pull) (Right)
    servo1.writeMicroseconds(neutral);
    servo2.writeMicroseconds(3000-rc_pin2_duration);
    servo3.writeMicroseconds(neutral);
    servo4.writeMicroseconds(3000-rc_pin4_duration);
  }
  Serial.print("PWM: ");
  Serial.println(pwm);
  Serial.print("Duration: ");
  Serial.println(rc_pin1_duration);
}

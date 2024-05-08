#include <Servo.h>
#include <TimerOne.h>
#define speedPinR 5   // RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  12    //  Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  11    // Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6        //  Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  7    // Left Motor direction pin 1 to MODEL-X IN3
#define LeftDirectPin2  8   ///Left Motor direction pin 1 to MODEL-X IN4
#define LPT 2 // scan loop coumter

#define SERVO_PIN 9
#define Echo_PIN    4 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    10  // Ultrasonic Trig pin connect to D12

#define BUZZ_PIN     13
#define FAST_SPEED  250     //both sides of the motor speed
#define SPEED  120     //both sides of the motor speed
#define TURN_SPEED  200     //both sides of the motor speed
#define BACK_SPEED1  255     //back speed
#define BACK_SPEED2  90     //back speed
// Include the TimerOne Library from Paul Stoffregen

// Constants for Interrupt Pins
// Change values if not using Arduino Uno
const byte MOTOR1 = 2; // Motor 1 Interrupt Pin - INT 0 (Encoder)
const byte MOTOR2 = 3; // Motor 2 Interrupt Pin - INT 1 (Encoder)
// Integers for pulse counters (global variables)
unsigned int counter1 = 0;
unsigned int counter2 = 0;
unsigned int m1Counter = 0;
unsigned int m2Counter = 0;

unsigned int leftMotorRPM = 0;
unsigned int rightMotorRPM = 0;

unsigned int leftSpeed = 250;
unsigned int rightSpeed = 250;

int increment = 5;
// Float for number of slots in encoder disk
float diskslots = 20; // Change to match value of encoder disk
// Interrupt Service Routines (for right and left motors)
// Motor 1 pulse count ISR
void ISR_count1()
{
  counter1++; // increment Motor 1 counter value
  m1Counter++;
}
// Motor 2 pulse count ISR
void ISR_count2()
{
  counter2++; // increment Motor 2 counter value
  m2Counter++;
}

const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)
void setRPM(int left, int right){
  leftMotorRPM = left;
  rightMotorRPM = right;
}
int thereis;
Servo head;
/*motor control*/
void go_Advance(int left, int right)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,left); 
  analogWrite(speedPinR,right);  
}
void go_Left(int speed)  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,speed); 
  analogWrite(speedPinR,speed); 
}
void go_Right()  //Turn right
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
}
void go_Back()  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
}
void setMotorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  setMotorspeed(0,0);
}

void buzz_OFF()  //close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
  
}
// TimerOne ISR

void ISR_timerone()
{
  Timer1.detachInterrupt(); // Stop the timer
  Serial.print("Motor Speed 1: ");
  float rotation1 = (counter1 / diskslots) * 60.00;
  // calculate RPM for Motor 1
  Serial.print(rotation1);
  Serial.print(" RPM - "); // display RPM on serial monitor
  
  Serial.print("Motor Speed 2: ");
  float rotation2 = (counter2 / diskslots) * 60.00;
  // calculate RPM for Motor 2
  Serial.print(rotation2); // display RPM on serial monitor
  Serial.println(" RPM");
  

  //left cruise control
  if(rotation1 > leftMotorRPM && leftSpeed > 5){
    leftSpeed -= increment;
  }else if(rotation1 < leftMotorRPM && leftSpeed < 250){
    leftSpeed += increment;
  }else{
    Serial.print("left else: ");
  }
  //right cruise control
  if(rotation2 > rightMotorRPM && rightSpeed > 5){
    rightSpeed -= increment;
  }else if(rotation2 < rightMotorRPM && rightSpeed < 250){
    rightSpeed += increment;
  }else{
    Serial.print("right else: ");
  }
  Serial.print("left: ");
  Serial.print(leftSpeed);
  Serial.print(" right: ");
  Serial.println(rightSpeed);
  //go_Advance();
  stop_Stop();
  go_Advance(leftSpeed, rightSpeed);
  setMotorspeed(leftSpeed, rightSpeed);

  counter1 = 0; // reset counter to zero
  counter2 = 0; // reset counter to zero
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
}
int counter = 0;
void setup()
{
  buzz_OFF();
  Serial.begin(9600);
  Timer1.initialize(100000); // set timer for 1sec(1000000)
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);
  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);
  // Increase counter 2 when speed sensor pin goes High
  //Timer1.attachInterrupt( ISR_timerone );
  // Enable the timer (this supports serial print for testing)
  //leftSpeed = 250;
  //rightSpeed = 250;
  setRPM(400,400);
  go_Advance(250,250);
  counter = 0;
  head.write(90);
  delay(10000);
}
void loop()
{
  /*
  Serial.println("-----");
  for(int n = 0; n < 3; n++){
    Serial.println(m1Counter);
    while(m1Counter < 150){
      
        go_Left(150);
        //Serial.println("turning");
        //Serial.println(m1Counter);
      }
      Serial.println("Ive stopped:");
      stop_Stop();
      Serial.println(m1Counter);
      
      delay(5000);
    
  }
  Serial.println(m1Counter);
  */
  
  
  //go_Advance(); // turn on motors (and stay on)
  Serial.println(counter);
  //setMotorspeed (200, 200);
  if(counter < 4){
    if(m1Counter < 200){
      go_Advance(250,250);
      //setMotorspeed (130, 130);
    }else{
      stop_Stop();
      delay(1000);
      m1Counter = 0;
      while(m1Counter <= 65){
        go_Left(200);
        Serial.println("turning");
        Serial.println(m1Counter);
      }
      stop_Stop();
      delay(1000);
      counter += 1;
      m1Counter = 0;
      Serial.print("update: ");
      Serial.println(counter);
      
    }
  }else{
    stop_Stop();

    //stop_Stop();
    //set
    Serial.println("bad");
    
  }
  

  

}



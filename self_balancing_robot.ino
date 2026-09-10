#include <MPU6050.h>
#include <Wire.h>
#include <math.h>

int ai1 = 9;
int ai2 = 8;
int pwmA = 10;

int bi1 = 7;
int bi2 = 6;
int pwmB = 5;


double Kp , Ki , Kd ;
double error , Integral , Derivative , targetAngle , output , PWM , lasterror;

float angle , angleAcc , dt , gyroRate;
float angleGyro = 0;
unsigned long previousTime = 0;
unsigned long currentTime;

MPU6050 mpu;
int16_t ax , ay , az , gx , gy , gz;

float sumgx , gyroOff;

void setup() {
  pinMode(ai1 , OUTPUT);
  pinMode(ai2 , OUTPUT);
  pinMode(pwmA , OUTPUT);
  pinMode(bi1 , OUTPUT);
  pinMode(bi2 , OUTPUT);
  pinMode(pwmB , OUTPUT);


  Wire.begin();
  Serial.begin(115200);
  mpu.initialize();

  for(int i =0 ; i<= 1000 ; i++){
    mpu.getMotion6(&ax , &ay , &az ,&gx , &gy , &gz);
    sumgx += gx;
    

  }
  gyroOff = sumgx/1000;
  

  targetAngle = 0;

  Kp =60;
  Ki = 1;
  Kd =6;


}

void loop() {
  angleCal();
  PID();

  if(angle < 0 ){
    digitalWrite(ai1, LOW);
    digitalWrite(ai2 , HIGH);
    analogWrite(pwmA , PWM );
    digitalWrite(bi1, LOW);
    digitalWrite(bi2 , HIGH);
    analogWrite(pwmB , PWM);
  }
  else {
    digitalWrite(ai1, HIGH);
    digitalWrite(ai2 , LOW);
    analogWrite(pwmA , PWM );
    digitalWrite(bi1, HIGH);
    digitalWrite(bi2 , LOW);
    analogWrite(pwmB , PWM);
  }
  


  

}
void angleCal(){
  mpu.getMotion6(&ax , &ay , &az ,&gx , &gy , &gz);
  angleAcc = atan2(ay , az )*180/PI;
  currentTime = micros();
  dt = (currentTime - previousTime)/1000000.0 ;
  previousTime = currentTime ;
  gyroRate = (gx - gyroOff) /131;

  angleGyro = angleGyro + gyroRate*dt;
  
  angle = 0.98*angleGyro + 0.02*angleAcc;

  Serial.print(angle);
  Serial.print("||");
}

void PID(){
  error = targetAngle - angle ;
  Integral += error*dt ; 
  Derivative = (error - lasterror)/dt;

  lasterror = error; 

  output = Kp*error + Ki*Integral + Kd*Derivative;

  output = constrain(output , -255 , 255);


  PWM = abs(output);


  Serial.println(output);


}
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Stepper.h>


struct Home{
  int Q1 = 0;
  int Q2 = 90;
  int Q3 = 90;
  int Q4 = 0;
  int Q5 = 80;
};

//ROBOT PARAMS
const int Q2_0 = 0;
const int Q2_1 = 1;
const int Q3 = 2;
const int Q4 = 3;
const int Q5 = 4;
const int GRIP = 5;
const float L1 = 0.3;
const float L2 = 0.3;
const float L3 = 0.3;
const float L4 = 0.2;
const float L5 = 0.2;
//FALTA CONOCER LOS LÍMITES REALES DEL BRAZO ROBÓTICO
const int Q1_LIMITS[2] = {0, 220};
const int Q2_LIMITS[2] = {0, 180};
const int Q3_LIMITS[2] = {0, 180};
const int Q4_LIMITS[2] = {0, 180};
const int Q5_LIMITS[2] = {0, 180};


//SERVO PARAMS
const int SERVO_FREQUENCY = 50;    // Frecuencia PWM de 50Hz o T=20ms
const float SERVO_MINDEGREES = 0;
const float SERVO_MAXDEGREES = 180;
const float SERVO_MINTICKS = 175; //ancho de pulso en ticks para pocicion 0º - 0.8 ms aprox
const float SERVO_MAXTICKS = 512; //ancho de pulso en ticks para la pocicion 180° - 2.5 ms aprox

//STEPPER PARAMS
const int dirPin = 8;
const int stepPin = 9;
 
const int steps = 200;
int microPausa = 5000;




Adafruit_PWMServoDriver servoController = Adafruit_PWMServoDriver(0x40); 


Home RobotHome;

int currentQ1 = RobotHome.Q1;
int currentQ2_0 = RobotHome.Q2;
int currentQ2_1 = RobotHome.Q2;
int currentQ3 = RobotHome.Q3;
int currentQ4 = RobotHome.Q4;
int currentQ5 = RobotHome.Q5;

int degreesToTicks(int servoDegrees);
int degreesToSteps(int Degrees);
void open_grip();
void close_grip();
void move_q1(int q1); //Stepper Motor
void move_q2(int q2);
void move_q3(int q3);
void move_q4(int q4);
void move_q5(int q5);
void pick_and_place();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  servoController.begin();
  servoController.setPWMFreq(SERVO_FREQUENCY);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  delay(2000);
  goHome();
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}


/************************************************/
/**********IMPLEMENTACIÓN DE FUNCIONES***********/
/************************************************/
void open_grip(){
  servoController.setPWM(GRIP, 0, degreesToTicks(0));
}

void close_grip(){
  servoController.setPWM(GRIP, 0, degreesToTicks(90));
}

void move_q1(int q1){
  if(q1 >= Q1_LIMITS[0] && q1 <= Q1_LIMITS[1]){
    if(currentQ1 < q1){
      digitalWrite(dirPin, HIGH);  // Cambio la dirección
      for(int i = 0; i <= degreesToSteps(q1-currentQ1); i++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(microPausa);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(microPausa);
      }
    }
    else{
      digitalWrite(dirPin, LOW);  // Cambio la dirección
      for(int i = 0; i <= degreesToSteps(currentQ1-q1); i++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(microPausa);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(microPausa);
      }
    }
  }
}

void move_q2(int q2){
  int j = currentQ2_1;
  if(q2>= Q2_LIMITS[0] && q2<=Q2_LIMITS[1]){
    if(q2 > currentQ2_0){
      for(int i = currentQ2_0; i <= q2; i++){
        servoController.setPWM(Q2_0, 0, degreesToTicks(i));
        servoController.setPWM(Q2_1, 0, degreesToTicks(j));
        j--;
        delay(50);
      }
    }
    else{
      for(int i = currentQ2_0; i >= q2; i--){
        servoController.setPWM(Q2_0, 0, degreesToTicks(i));
        servoController.setPWM(Q2_1, 0, degreesToTicks(j));
        j++;
        delay(50);
      }
    }
    currentQ2_0 = q2;
    currentQ2_1 = j;
  }
}

void move_q3(int q3){
  if(q3>= Q3_LIMITS[0] && q3<=Q3_LIMITS[1]){
    if(q3 > currentQ3){
      for(int i = currentQ3; i <= q3; i++){
        servoController.setPWM(Q3, 0, degreesToTicks(i));
        delay(50);
      }
    }
    else{
      for(int i = currentQ3; i >= q3; i--){
        servoController.setPWM(Q3, 0, degreesToTicks(i));
        delay(50);
      }
    }
    currentQ3 = q3;
  }
}

void move_q4(int q4){
  if(q4>= Q4_LIMITS[0] && q4<=Q4_LIMITS[1]){
    if(q4 > currentQ4){
      for(int i = currentQ4; i <= q4; i++){
        servoController.setPWM(Q4, 0, degreesToTicks(i));
        delay(50);
      }
    }
    else{
      for(int i = currentQ4; i >= q4; i--){
        servoController.setPWM(Q4, 0, degreesToTicks(i));
        delay(50);
      }
    }
    currentQ4 = q4;
  }
}

void move_q5(int q5){
  if(q5>= Q5_LIMITS[0] && q5<=Q5_LIMITS[1]){
    if(q5 > currentQ5){
      for(int i = currentQ5; i <= q5; i++){
        servoController.setPWM(Q5, 0, degreesToTicks(i));
        delay(30);
      }
    }
    else{
      for(int i = currentQ5; i >= q5; i--){
        servoController.setPWM(Q5, 0, degreesToTicks(i));
        delay(30);
      }
    }
    currentQ5 = q5;
  }
}


//Transform degrees to ticks
int degreesToTicks(int servoDegrees){
  int ticks = 0;
  
  if(servoDegrees >= SERVO_MINDEGREES && servoDegrees <= SERVO_MAXDEGREES){
    ticks = servoDegrees*((SERVO_MAXTICKS - SERVO_MINTICKS)/(SERVO_MAXDEGREES - SERVO_MINDEGREES)) + SERVO_MINTICKS;
  }
  
  return ticks;
}

//Transform degrees to steps
int degreesToSteps(int Degrees){
  int pasos = 0;
  pasos = (Degrees*steps)/360;
  return pasos;
}

void goHome(){
  close_grip();
  delay(2000);
  servoController.setPWM(Q5, 0, degreesToTicks(RobotHome.Q5));
  delay(2000);
  servoController.setPWM(Q4, 0, degreesToTicks(RobotHome.Q4));
  delay(2000);
  servoController.setPWM(Q3, 0, degreesToTicks(RobotHome.Q3));
  delay(2000);
  servoController.setPWM(Q2_0, 0, degreesToTicks(RobotHome.Q2));
  servoController.setPWM(Q2_1, 0, degreesToTicks(RobotHome.Q2));
  delay(2000); 
}

void pick_and_place(){
  delay(1000);
  move_q2(85);
  delay(1000);
  open_grip();
  delay(1000);
  move_q3(170);
  delay(1000);
  close_grip();
  delay(1000);
  move_q2(RobotHome.Q2);
  delay(1000);
  move_q4(RobotHome.Q4);
  delay(1000);
  move_q3(RobotHome.Q3);
  delay(1000);
  move_q1(60);
  delay(1000);
  move_q2(85);
  delay(1000);
  move_q3(170);
  delay(1000);
  open_grip();
  delay(1000);
  open_grip();
  delay(1000);
  move_q2(RobotHome.Q2);
  delay(1000);
  move_q4(RobotHome.Q4);
  delay(1000);
  move_q3(RobotHome.Q3);
  delay(1000);
}

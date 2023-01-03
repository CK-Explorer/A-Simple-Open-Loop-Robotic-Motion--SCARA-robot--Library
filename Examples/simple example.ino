#define stepper_pin_1   2   //Output pin for controlling the rotation steps of arm 1's stepper motor
#define stepper_pin_2   3   //Output pin for controlling the rotation steps of arm 2's stepper motor
#define stepper_pin_z   4   //Output pin for controlling the rotation steps of z axis's stepper motor
#define direction_1     5   //Output pin for controlling the rotation direction of arm 1's stepper motor
#define direction_2     6   //Output pin for controlling the rotation direction of arm 2's stepper motor
#define direction_z     7   //Output pin for controlling the rotation direction of z-axis's stepper motor
#define endEffectorPin  9   //Output pin to control end effector (servo)
#define limit_stepper_pin_1   12    //Input pin for the end stop for arm 1
#define limit_stepper_pin_2   10    //Input pin for the end stop for arm 2
#define limit_stepper_pin_z   11    //Input pin for the end stop for z-axis

#include <Servo.h>
#include "RobotMotion.h"

/*Hardware parameter (static class member)*/
const float RobotMotion::armLength1 = 10.0; //in cm
const float RobotMotion::armLength2 = 23.5; //in cm
const float RobotMotion::zOffset = 2.4;     //in cm
const float RobotMotion::teethRatio1 = 62.00/20.00;
const float RobotMotion::teethRatio2 = 62.00/20.00;
const float RobotMotion::lead = 0.8;        //in cm/rev
const int RobotMotion::stepPerRev1 = 6400;
const int RobotMotion::stepPerRev2 = 6400;
const int RobotMotion::stepPerRevZaxis = 400;
const float RobotMotion::limitSpeed_v1 = 15000;
const float RobotMotion::limitSpeed_v2 = 15000;
const float RobotMotion::limitSpeed_vZaxis = 15000;
const byte RobotMotion::stepperPin_p1 = stepper_pin_1;
const byte RobotMotion::stepperPin_p2 = stepper_pin_2;
const byte RobotMotion::stepperPin_pZaxis = stepper_pin_z;
const byte RobotMotion::stepperDirPin_p1 = direction_1;
const byte RobotMotion::stepperDirPin_p2 = direction_2;
const byte RobotMotion::stepperDirPin_pZaxis = direction_z;

/*Global coordinates of the end effector*/
float x_coordinate[] = {   0,   23.688,   33.5,   33.5,   33.5,  23.688,   0,   -18.058,   0};//in cm
float y_coordinate[] = {  33.5,   23.688,    0,    0,    0,  23.688,  33.5,    18.058,  33.5};
float z_coordinate[] = {  20,     4.0,  4.0,  1.0,  4.0,    4.0,  20,    20,   20};
int   end_eff_open[] =             {0,    1,    0,    0,      0,   0,     1,   0};    //1 = open; 0 = nothing happens
int   end_eff_close[]=             {0,    0,    1,    0,      0,   0,     1,   0};    //1 = close; 0 = nothing happens
float motion_time[]  =             {4,    1,    2,    2,      1,   4,     1,   1};
float crusingSpeed[]   =             {9500, 8900, 7000, 6550, 7000, 7850, 9300, 9800};

Servo end_effector;
RobotMotion robot(9, true);

/*Before performing any sets of motion, the positions of the robotic arms 
* need to be calibrated.
*/
void end_stop(byte limit_stepper, byte stepperPin, byte dirPin, int dir, int state);

/*Obtain the computed speed and time as well as the present stage of motion. 
* A function pointer that pass into RobotMotion::motion
* (Note: may affect the smooth transition from one position to the other
*        due to Serial.print())
*/
void speedTimeAssign(float speedArm1, float speedArm2, float speedZaxis, 
    float t1, float t2, float tZaxis, int stage);

/*For end effector motion.
* A function pointer that pass into RobotMotion::motion
*/
int pos;    //Position of the servo
void endEffector(int count);
inline void servo_open();
inline void servo_close();

void setup() {
    /*OUTPUT*/
    pinMode(stepper_pin_1, OUTPUT); 
    pinMode(stepper_pin_2, OUTPUT); 
    pinMode(stepper_pin_z, OUTPUT); 
    pinMode(direction_1, OUTPUT); 
    pinMode(direction_2, OUTPUT); 
    pinMode(direction_z, OUTPUT);

    /*INPUT*/
    pinMode(endEffectorPin, INPUT); 
    pinMode(limit_stepper_pin_1, INPUT); 
    pinMode(limit_stepper_pin_2, INPUT); 
    pinMode(limit_stepper_pin_z, INPUT); 
    
    Serial.begin(19200);
    end_effector.attach(endEffectorPin);

    /*Initialize robot object with the motion needed*/
    robot.position.x = x_coordinate;
    robot.position.y = y_coordinate;
    robot.position.z = z_coordinate;
    robot.time.t1 = motion_time;
    robot.time.t2 = motion_time;
    robot.time.tZaxis = motion_time;
    robot.maxCruisingSpeed.v1 = crusingSpeed;
    robot.maxCruisingSpeed.v2 = crusingSpeed;
    robot.maxCruisingSpeed.vZaxis = crusingSpeed;

    robot.setValuesBool(true);
    robot.cooordinates2step();
    robot.printStepDir();

    end_stop(limit_stepper_pin_1, stepper_pin_1, direction_1, LOW, HIGH);
    delay(1000);
    end_stop(limit_stepper_pin_2, stepper_pin_2, direction_2, HIGH, HIGH);
    delay(1000);
    end_stop(limit_stepper_pin_z, stepper_pin_z, direction_z, HIGH, HIGH);
    delay(1000);

    robot.motion(speedTimeAssign, endEffector);
}

void loop() {
}

void end_stop(byte limit_stepper, byte stepperPin, byte dirPin, int dir, int state)
{
    digitalWrite(dirPin, dir);
    int stat;
    do
    {   
        stat = digitalRead(limit_stepper);
        if (stat == state)
        {
            digitalWrite(stepperPin, HIGH);
            delayMicroseconds(300); 
            digitalWrite(stepperPin, LOW);
            delayMicroseconds(300); 
        }
    }while(stat == state);
}

void speedTimeAssign(float speedArm1, float speedArm2, float speedZaxis, 
    float t1, float t2, float tZaxis, int stage)
{
    Serial.print(" ( ");
    Serial.print(stage);
    Serial.print(" ) ");
    Serial.print("Motor Speed (arm 1) = ");
    Serial.print(speedArm1);
    Serial.print(" , ");
    Serial.print("Motor Speed (arm 2) = ");
    Serial.print(speedArm2);
    Serial.print(" , ");
    Serial.print("Motor Speed (z axis) = ");
    Serial.print(speedZaxis);
    Serial.print(" , ");
    Serial.print("Time (arm 1) = ");
    Serial.print(t1);
    Serial.print(" , ");
    Serial.print("Time (arm 2) = ");
    Serial.print(t2);
    Serial.print(" , ");
    Serial.print("Time (z axis) = ");
    Serial.println(tZaxis);
}

void endEffector(int count)
{
    if (end_eff_open[count])
        servo_open();

    if (end_eff_close[count])
        servo_close();
}

inline void servo_open()
{
    pos = 90; //end effector release function
    for (pos = 90; pos >= 0; pos -= 1) 
    { // goes from 90 degrees to 0 degrees
        end_effector.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);     
    }
}

inline void servo_close()
{
    pos = 0;  //end effector grab function
    for (pos = 0; pos <= 90; pos += 1) 
    { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        end_effector.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
}
/*Schematic diagram is available at https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/Schematic/schematic.png
*/

#define stepper_pin_1   2   //Output pin for controlling the rotation steps of arm 1's stepper motor
#define stepper_pin_2   3   //Output pin for controlling the rotation steps of arm 2's stepper motor
#define stepper_pin_z   4   //Output pin for controlling the rotation steps of z axis's stepper motor
#define direction_1     5   //Output pin for controlling the rotation direction of arm 1's stepper motor
#define direction_2     6   //Output pin for controlling the rotation direction of arm 2's stepper motor
#define direction_z     7   //Output pin for controlling the rotation direction of z-axis's stepper motor
#define ENBL            8   //ENBL pin for DRV8825
#define endEffectorPin  9   //Output pin to control end effector (servo)
#define limit_stepper_pin_1   12    //Input pin for the end stop for arm 1
#define limit_stepper_pin_2   10    //Input pin for the end stop for arm 2
#define limit_stepper_pin_z   11    //Input pin for the end stop for z-axis
#define interruptPin    18  //Interrupt pin for emergency button
#define resume          37  //Resume button after emergency stopping
#define selectButton1   39  //Input pin for button 1 for motion 1
#define selectButton2   41  //Input pin for button 2 for motion 2
#define selectButton3   43  //Input pin for button 2 for motion 3

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

/*Initializing*/
float x_coordinate_ini[] = {   12.68,      0};  //in cm
float y_coordinate_ini[] = { 11.531,     33.5};
float z_coordinate_ini[] = {   2.4,     20};
float motion_time_ini[]  =    {5};
float cruisingSpeed_ini[]   =    {9000};   //number of step per rotation

/*1st choice (Note: the starting point must be the same as the previous ending point.)*/
float x_coordinate_1[] = {   0,   23.688,   33.5,   33.5,   33.5,  23.688,   0,   -18.058,   0};//in cm
float y_coordinate_1[] = {  33.5,   23.688,    0,    0,    0,  23.688,  33.5,    18.058,  33.5};
float z_coordinate_1[] = {  20,     4.0,  4.0,  1.0,  4.0,    4.0,  20,    20,   20};
int   end_eff_open_1[] =             {0,    1,    0,    0,      0,   0,     1,   0};    //1 = open; 0 = nothing happens
int   end_eff_close_1[]=             {0,    0,    1,    0,      0,   0,     1,   0};    //1 = close; 0 = nothing happens
float motion_time_1[]  =             {4,    1,    2,    2,      1,   4,     1,   1};
float cruisingSpeed_1[]   =             {9500, 8900, 7000, 6550, 7000, 7850, 9300, 9800};

/*2nd choice (Note: the starting point must be the same as the previous ending point.)*/
float x_coordinate_2[] = {   0,   23.688,   33.5,   33.5,   33.5,  23.688,   0,   -18.058,   0};//in cm
float y_coordinate_2[] = {  33.5,   23.688,    0,    0,    0,  23.688,  33.5,    18.058,  33.5};
float z_coordinate_2[] = {  20,    11.5, 11.5,  8.5, 11.5,   11.5,  20,    20,   20};
int   end_eff_open_2[] =             {0,    1,    0,    0,      0,   0,     1,   0};
int   end_eff_close_2[]=             {0,    0,    1,    0,      0,   0,     1,   0};
float motion_time_2[]  =             {3,    1,    2,    2,      1,   3,     1,   1};
float cruisingSpeed_2[]   =             {8500, 7900, 7030, 6550, 7300, 7855, 8336, 9799};

/*3rd choice (Note: the starting point must be the same as the previous ending point.)*/
float x_coordinate_3[] = {   0,   23.688,   33.5,   33.5,   33.5,  23.688,   0,   -18.058,   0};//in cm
float y_coordinate_3[] = {  33.5,   23.688,    0,    0,    0,  23.688,  33.5,    18.058,  33.5};
float z_coordinate_3[] = {  20,    19.0,  19.0,16.0, 19.0,   19.0,    20,    20,   20};
int   end_eff_open_3[] =             {0,    1,    0,    0,      0,   0,     1,   0};
int   end_eff_close_3[]=             {0,    0,    1,    0,      0,   0,     1,   0};
float motion_time_3[]  =             {1,    1,    1,    1,      1,     1,     1,   1};
float cruisingSpeed_3[]   =             {9350, 8910, 8400, 8750, 7600, 7825, 9250, 9870};

Servo end_effector;
RobotMotion robot_ini(2, true);
RobotMotion robot1(9, true);
RobotMotion robot2(9, true);
RobotMotion robot3(9, true);

/*An emergency button to disable the robotic motion, by sending HIGH to all
* ENBL pins of the DRV8825 motor drivers.
* Hold the resume button to enable the motion.
*/
void stop_all();

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
void endEffector1(int count);
void endEffector2(int count);
void endEffector3(int count);
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
    pinMode(interruptPin, INPUT);
    pinMode(resume, INPUT);
    pinMode(selectButton1, INPUT);
    pinMode(selectButton2, INPUT);
    pinMode(selectButton3, INPUT);

    Serial.begin(19200);
    end_effector.attach(endEffectorPin);
    attachInterrupt(digitalPinToInterrupt(interruptPin), stop_all, LOW);
    digitalWrite(ENBL, LOW);

    /*Initialize robot_ini*/
    robot_ini.position.x = x_coordinate_ini;
    robot_ini.position.y = y_coordinate_ini;
    robot_ini.position.z = z_coordinate_ini;
    robot_ini.time.t1 = motion_time_ini;
    robot_ini.time.t2 = motion_time_ini;
    robot_ini.time.tZaxis = motion_time_ini;
    robot_ini.maxCruisingSpeed.v1 = cruisingSpeed_ini;
    robot_ini.maxCruisingSpeed.v2 = cruisingSpeed_ini;
    robot_ini.maxCruisingSpeed.vZaxis = cruisingSpeed_ini;

    /*Initialize robot1*/
    robot1.position.x = x_coordinate_1;
    robot1.position.y = y_coordinate_1;
    robot1.position.z = z_coordinate_1;
    robot1.time.t1 = motion_time_1;
    robot1.time.t2 = motion_time_1;
    robot1.time.tZaxis = motion_time_1;
    robot1.maxCruisingSpeed.v1 = cruisingSpeed_1;
    robot1.maxCruisingSpeed.v2 = cruisingSpeed_1;
    robot1.maxCruisingSpeed.vZaxis = cruisingSpeed_1;

    /*Initialize robot2*/
    robot2.position.x = x_coordinate_2;
    robot2.position.y = y_coordinate_2;
    robot2.position.z = z_coordinate_2;
    robot2.time.t1 = motion_time_2;
    robot2.time.t2 = motion_time_2;
    robot2.time.tZaxis = motion_time_2;
    robot2.maxCruisingSpeed.v1 = cruisingSpeed_2;
    robot2.maxCruisingSpeed.v2 = cruisingSpeed_2;
    robot2.maxCruisingSpeed.vZaxis = cruisingSpeed_2;

    /*Initialize robot3*/
    robot3.position.x = x_coordinate_3;
    robot3.position.y = y_coordinate_3;
    robot3.position.z = z_coordinate_3;
    robot3.time.t1 = motion_time_3;
    robot3.time.t2 = motion_time_3;
    robot3.time.tZaxis = motion_time_3;
    robot3.maxCruisingSpeed.v1 = cruisingSpeed_3;
    robot3.maxCruisingSpeed.v2 = cruisingSpeed_3;
    robot3.maxCruisingSpeed.vZaxis = cruisingSpeed_3;

    robot_ini.setValuesBool(true);
    robot1.setValuesBool(true);
    robot2.setValuesBool(true);
    robot3.setValuesBool(true);

    robot_ini.cooordinates2step();
    robot1.cooordinates2step();
    robot2.cooordinates2step();
    robot3.cooordinates2step();

    Serial.println("robot_ini = ");
    robot_ini.printStepDir();
    Serial.println("robot1 = ");
    robot1.printStepDir();
    Serial.println("robot2 = ");
    robot2.printStepDir();
    Serial.println("robot3 = ");
    robot3.printStepDir();

    end_stop(limit_stepper_pin_1, stepper_pin_1, direction_1, LOW, HIGH);
    delay(1000);
    end_stop(limit_stepper_pin_2, stepper_pin_2, direction_2, HIGH, HIGH);
    delay(1000);
    end_stop(limit_stepper_pin_z, stepper_pin_z, direction_z, HIGH, HIGH);
    delay(1000);

    Serial.println("Position 0:");
    robot_ini.motion(speedTimeAssign, [](int){});
    Serial.println("Motion is executed perfectly!");
    
    Serial.println("The robot is ready to perform the motion.");
}

void loop() {
    if (!digitalRead(selectButton1))
    {
        Serial.println("Position 1:");
        robot1.motion(speedTimeAssign, endEffector1);
        Serial.println("Motion is executed perfectly!");
    }
    
    if (!digitalRead(selectButton2))
    {
        Serial.println("Position 2:");
        robot2.motion(speedTimeAssign, endEffector2);
        Serial.println("Motion is executed perfectly!");
    }
    
    if (!digitalRead(selectButton3))
    {
        Serial.println("Position 3:");
        robot3.motion(speedTimeAssign, endEffector3);
        Serial.println("Motion is executed perfectly!");
    }
}

void stop_all()
{
    digitalWrite(ENBL, HIGH);
    for(;;)
    {
        delayMicroseconds(1000);
        if (!digitalRead(resume))
        {
            digitalWrite(ENBL, LOW);
            break;
        }     
    }
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

void endEffector1(int count)
{
    if (end_eff_open_1[count])
        servo_open();

    if (end_eff_close_1[count])
        servo_close();
}

void endEffector2(int count)
{
    if (end_eff_open_2[count])
        servo_open();

    if (end_eff_close_2[count])
        servo_close();
}

void endEffector3(int count)
{
    if (end_eff_open_3[count])
        servo_open();

    if (end_eff_close_3[count])
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
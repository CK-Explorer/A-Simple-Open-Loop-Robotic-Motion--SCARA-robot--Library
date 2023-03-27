## :star2: A Brief Explanation of the Steps of Using This Library

As stated in the title, this page is just a sufficient summary about the steps, hence please refer to the [header file](RobotMotion%20Library/RobotMotion.h) and [documentation](Documentation/Working%20Principle.pdf) if you have any doubt.

### :warning: Note: 

Steps [4](#four-assign-locomotive-information-to-the-object), [5](#five-inform-the-object-that-all-parameters-are-set), [6](#six-convert-coordinates-to-the-number-of-steps-and-directions) and [8](#eight-execute-the-robotic-motion) must be executed in this order. 

# :notebook_with_decorative_cover: Steps
1. [Include Library](#one-include-library)
2. [Setting Hardware Parameters](#two-setting-hardware-parameters)
3. [Instantiate an Object](#three-instantiate-an-object)
4. [Assign Locomotive Information to the Object](#four-assign-locomotive-information-to-the-object)
5. [Inform The Object that All Parameters are Set](#five-inform-the-object-that-all-parameters-are-set)
6. [Convert Coordinates to the Number of Steps and Directions](#six-convert-coordinates-to-the-number-of-steps-and-directions)
7. [Print the Number of Steps and Directions (Optional)](#seven-print-the-number-of-steps-and-directions-optional)
8. [Execute the robotic motion](#eight-execute-the-robotic-motion)

## :one: Include Library

```
#include "RobotMotion.h"
```

## :two: Setting Hardware Parameters

Hardware parameters are statically declared variables, hence they only need to be declared once in the global scope.

Please refer to the [header file](https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/RobotMotion.h) for their full definitions.

```
const float RobotMotion::armLength1 = 10.0; 
const float RobotMotion::armLength2 = 23.5; 
const float RobotMotion::zOffset = 2.4;     
const float RobotMotion::teethRatio1 = 62.00/20.00;
const float RobotMotion::teethRatio2 = 62.00/20.00;
const float RobotMotion::lead = 0.8;        
const int RobotMotion::stepPerRev1 = 6400;
const int RobotMotion::stepPerRev2 = 6400;
const int RobotMotion::stepPerRevZaxis = 400;
const float RobotMotion::limitSpeed_v1 = 15000;
const float RobotMotion::limitSpeed_v2 = 15000;
const float RobotMotion::limitSpeed_vZaxis = 15000;
const byte RobotMotion::stepperPin_p1 = 2;
const byte RobotMotion::stepperPin_p2 = 3;
const byte RobotMotion::stepperPin_pZaxis = 4;
const byte RobotMotion::stepperDirPin_p1 = 5;
const byte RobotMotion::stepperDirPin_p2 = 6;
const byte RobotMotion::stepperDirPin_pZaxis = 7;
```

## :three: Instantiate an Object

`RobotMotion` is the constructor, and it has two parameters, i.e.

* `no_Coordinates`(1st) = the number of waypoints of the end effector needs to reach during robotic motion.
* `selfConfigure`(2nd) = Inform the constructor to allocate the memory space dynamically for the robot's locomotive information, i.e. positions, speeds and times of motion.
    * true = the constructor will **not** allocate any memory space, and hence users need to self-initialize their own array (size = `no_Coordinates`).
    * false = the constructor will allocate memory space (size = `no_Coordinates`), user can use the memory space allocated.
```
RobotMotion robot(2, true);     //no_Coordinates = 2, selfConfigure = true
```

## :four: Assign Locomotive Information to the Object

At least two sets of locomotive information are needed, i.e. two sets of positions, times, and speeds.

(Note: The starting position must be the same as the last located position of the end effector)

The following example is when **`selfConfigure`=true** assigned in the constructor.

```
float x_coordinate[] = {   12.68,      0}; 
float y_coordinate[] = { 11.531,     33.5};
float z_coordinate[] = {   2.4,     20};
float motion_time[]  =    {5};      //For simplicity, all three links have the same time
float crusingSpeed[]   =    {9000};   //For simplicity, all three links have the same speed

robot.position.x = x_coordinate;
robot.position.y = y_coordinate;
robot.position.z = z_coordinate;
robot.time.t1 = motion_time;
robot.time.t2 = motion_time;
robot.time.tZaxis = motion_time;
robot.maxCruisingSpeed.v1 = crusingSpeed;
robot.maxCruisingSpeed.v2 = crusingSpeed;
robot.maxCruisingSpeed.vZaxis = crusingSpeed;
``` 
The following example is when **`selfConfigure`=false** assigned in the constructor.
```
float x_coordinate[] = {   12.68,      0}; 
float y_coordinate[] = { 11.531,     33.5};
float z_coordinate[] = {   2.4,     20};
float motion_time[]  =    {5};      //For simplicity, all three links have the same time
float crusingSpeed[]   =    {9000};   //For simplicity, all three links have the same speed

for (int i = 0; i < 2; i++)
{
    robot.position.x[i] = x_coordinate[i];
    robot.position.y[i] = y_coordinate[i];
    robot.position.z[i] = z_coordinate[i];
}
    
robot.time.t1[1] = motion_time[1];
robot.time.t2[1] = motion_time[1];
robot.time.tZaxis[1] = motion_time[1];
robot.maxCruisingSpeed.v1[1] = crusingSpeed[1];
robot.maxCruisingSpeed.v2[1] = crusingSpeed[1];
robot.maxCruisingSpeed.vZaxis[1] = crusingSpeed[1];
``` 

## :five: Inform The Object that All Parameters are Set

`setValuesBool` is just a redundant function to avoid any accident due to possible mishap when the programmer forgot to assign any necessary parameters.

It accepts only one boolean parameter, thus just set "true" after all parameters are set. 
    
Else, the following two functions below will not be executed.

* `cooordinates2step`
* `motion`

```
robot.setValuesBool(true);
``` 

## :six: Convert Coordinates to the Number of Steps and Directions

`cooordinates2step` will convert the global coordinates of the end effector from the positions set by users to the number of steps and directions required to execute the motion.

Error code = -1, if `setValuesBool` in [step 5](#five-inform-the-object-that-all-parameters-are-set) is not set to true, and hence no calculation will be performed.

Error code = 0, indicates that the function is executed successfully.
```
robot.cooordinates2step();
```

## :seven: Print the Number of Steps and Directions (Optional)
`printStepDir` uses serial print to print the number of steps and directions calculated in the step 6. This is useful for debugging purpose.

```
Serial.begin(19200);    //Remember to initiate serial communication before calling the following function. 
robot.printStepDir();
```

## :eight: Execute the robotic motion
`motion` is the function that makes the robotic links to move according to the user-defined robot's locomotive information. This function accepts two functors (function pointers) as its parameters, i.e.

* `speedTimeFcn` (1st) takes 7 arguments and they are all passed by argument, which is performed before starting the motion of that stage.
    * The first three parameters of this functor are the maximum cruising speed selected by the program. (datatype = float)

    * The following three parameters are the motion of time of each link. (datatype = float)

    * The last parameter is the stage of the locomotive information. (datatype = int)

* `endEffectorFcn` (2nd) takes one argument and it is passed by argument. It is performed after executing the motion of that stage, and hence it is useful to act as a function to control the end effector.
    * The only parameter here is the stage of the locamotive information. (datatype = int)

Error code = -1, if `setValuesBool` in [step 5](#five-inform-the-object-that-all-parameters-are-set) is not set to true, and hence no motion will be performed.

Error code = 0, indicates that the function is executed successfully.
```
robot.motion(speedAssign, endEffector); 
/*speedAssign is a functor of speedTimeFcn(float, float, float, int).
* endEffector is a functor of endEffectorFcn(int).
*/
```

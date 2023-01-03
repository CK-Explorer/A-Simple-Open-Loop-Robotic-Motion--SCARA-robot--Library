/*GitHub Page: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library
* API tutorial: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/Brief%20explanation.md
* Documentation: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/Documentation/Working%20Principle.pdf
*/

#ifndef RobotMotion_h
#define RobotMotion_h

#include <Arduino.h>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <math.h>

class RobotMotion
{
public:
	/*The positions of end effector in any given halt, stored in array.
	* (Note : must have the same measurement unit as armLength1,
	* armLength2 and lead)
	*/
	struct position
	{
		float* x;
		float* y;
		float* z;
	} position;

	/*The time of end effector to move between the above positions, stored 
	* in array (Note: the array size is one element smaller than the 
	* positions' arrays).
	*/
	struct time
	{
		float* t1;
		float* t2;
		float* tZaxis;
	} time;

	/*Maximum crusing speed (number of steps per second) of each motor
	* transistioning between two positions, stored in array (Note: the 
	* array size is one element smaller than the positions' arrays).
	*/
	struct maxCruisingSpeed
	{
		float* v1;
		float* v2;
		float* vZaxis;
	} maxCruisingSpeed;

	/*Hardware parameters: (Static declared)
	* armLength1 = 		the length of arm 1
	* armLength2 = 		the length of arm 2 (Note : must have the same 
	*			measurement unit as armLength1, lead and 
	* 			position)
	* zOffset	=		the extra offset of the end-effector.
	* teethRatio1 = 	the ratio of gear teeth number for arm 1 to the 
	*			stepper motor (arm 1/motor)
	* teethRatio2 = 	the ratio of gear teeth number for arm 2 to the 
	*			stepper motor (arm 2/motor)
	* lead = 		the length of linear travel per one revolution
	*			(Note: must have the same measurement unit as 
	*			armLength1, armLength2 and positions)
	* stepPerRev1 = 	the number of steps to complete a revolution for 
	* 			motor controlling arm 1.
	* stepPerRev2 = 	the number of steps to complete a revolution for 
	* 			motor controllong arm 2.
	* stepPerRevZaxis = 	the number of steps to complete a revolution for 
	* 			motor controllong z-axis.
	*/
	static const float armLength1;
	static const float armLength2;
	static const float zOffset;
	static const float teethRatio1;	
	static const float teethRatio2;
	static const float lead;	
	static const int stepPerRev1;
	static const int stepPerRev2;
	static const int stepPerRevZaxis;

	/*The highest permissible speed (number of steps per second) for
	* each motor. (hardware limitation)
	* (Static declared)
	* limitSpeed_v1 	= 	the speed of motor for arm 1 
	* limitSpeed_v2 	= 	the speed of motor for arm 2
	* limitSpeed_vZaxis 	= 	the speed of motor for z-axis
	*/
	static const float limitSpeed_v1;
	static const float limitSpeed_v2;
	static const float limitSpeed_vZaxis;

	/*Arduino pins' values that output rectangle waveforms, the time 
	* between two consecutive high rising edges = the time to move one 
	* step of the stepper motor, suitable for stepper motor driver 
	* like DRV8825, A4988.
	* (Static declared)
	*/
	static const byte stepperPin_p1;
	static const byte stepperPin_p2;
	static const byte stepperPin_pZaxis;

	/*Arduino pins' values that output directions of the rotation in the 
	* form of high and low, with the following definitions:
	* HIGH = clockwise, LOW = anticlockwise
	* (Static declared)
	*/
	static const byte stepperDirPin_p1;
	static const byte stepperDirPin_p2;
	static const byte stepperDirPin_pZaxis;
	
protected:
	float* angleMotor1;
	float* angleMotor2;
	float* zNew;

	/*The number of steps required to transistion for one position to
	* the other.
	* step1 = 	the number of steps for the motor controlling arm 1.
	* step2 = 	the number of steps for the motor controlling arm 2.
	* stepZaxis = 	the number of steps for the motor controlling z axis.
	*/
	int* step1;
	int* step2;
	int* stepZaxis;

	/*Variables to store the direction of the revolution of the motors,
	* with the following definitions:
	* HIGH = clockwise, LOW = anticlockwise
	*
	* direction1 = 	   the direction for the motor controlling arm 1. 
	* direction2 = 	   the direction for the motor controlling arm 2.
	* directionZaxis = the direction for the motor controlling z-axis.
	*/
	int* direction1;
	int* direction2;
	int* directionZaxis;

	/*no_Coordinates = number of positions of the end effector during robot
	* motion.
	*/
	int no_Coordinates;
	/*If using default constructor, selfConfigure = true, which means user
	* need to self-allocate the memory.
	* If using overloading constructor, selfConfigure = false, which means 
	* the instantiated object will self-allocate the memory.
	*/
	bool selfConfigure;
	/*setValues as a safety setting to ensure user initializes all the 
	* parameters before beginning any calculation and motions.
	* setValues = true, allows to execute the two function members.
	* setValues = false, otherwise.
	*/
	bool setValues;

	/*A structure to hold data for the inner loop of int motion(...).
	*/
	struct MotionParam
	{
		float acc_time = 0.00;
		float tf_s, tf_ms;
		int stepper_state = LOW;
		int calculation_enable = HIGH;
		int number_step = 0;
		int validSpeed;
		float tb_1, tb_3;
		float invOmega, constant, change_time;
		unsigned long previousMicros;
	};

public:
	/*Constructor:
	* no_Coordinates = 	number of positions of the coordinates 
	* 			of the end effector to stop during the 
	* 			robotic motion, (at least equal to 2).
	* selfConfigure = true, then the constructor will not allocate 
	* memory and hence users need to initialize the array.
	* selfConfigure = false, the constructor will allocate the 
	* memory. (Note: beware of such pointer will deallocate when the
	* object reaches its end of life, hence do not let this pointer
	* to be lvalue reference of other pointer.)
	*/
	RobotMotion(const int& no_Coordinates, const bool& selfConfigure);
	~RobotMotion();

	/*Pass true for the setValues manually after all the parameters 
	* listed on the public member are initialized, else no motion
	* will be resulted.
	* 
	* Return value = 0 	: 	Success.
	*
	* The following are errors:
	* Return value = -1 : 	no_Coordinates < 1, insufficient positions
	* 			to perform the motion, needs at least two 
	*			coordinates.
	* Return value = -2 : 	the required arrays are not initialized.
	*/
	int setValuesBool(const bool& setValues);

	/*Convert from the coordinates of end effector to the angles of
	* the motors to transition from those coordinates, then to the
	* number of steps to move the stepper motor's shaft.
	*
	* Return value = 0 	: 	Success.
	*
	* The following are errors:
	* Return value = -1 : 	setValues = false , remember to set it 
	* 			to true after initialize all variables.
	*/
	int cooordinates2step();

	/*Motion of robotic arm with trapezoidal velocity profile
	* with all arms finish the motion according to their corresponding 
	* time specified (from the time struct above), the crusing speed 
	* of each motor will be set accordingly if the desired speed is 
	* not suitable. 
	* Also, if the speed selected by the function exceeds the 
	* permissible hardware speed, the motion of time will be 
	* readjusted, and the desired speed will be the same as the 
	* stepper motor maximum permissible speed.
	*
	* Callback function:
	* speedTimeFcn = the function that passes by argument the 
	* speeds of the motors (steps per second)
	* first argument = speed of motor for arm 1
	* second argument = speed of motor for arm 2
	* third argument = speed of motor for z-axis
	* fourth argument = crusing time for arm 1
	* fifth argument = crusing time for arm 2
	* sixth argument = crusing time for z-axis
	* seventh argument = the stage of the robotic motion
	*
	* endEffectorFcn = the function to control the end effector
	* which passes by argument an integer value that indicates 
	* the stage of the robotic motion.
	*
	* Return value = 0 	: 	Success.
	*
	* The following are errors:
	* Return value = -1 : 	setValues = false , remember to set it 
	* 					    to true after initialize all variables.
	*/
	int motion(void(*speedTimeFcn)(float, float, float, 
        	float, float, float, int), 
		void(*endEffectorFcn)(int));

	/*A function to print the number of steps and directions of the 
	* stepper motor. (Useful for debugging purpose)
	*/
	void printStepDir();

protected:
	/*Convert the coordinates of the positions to the angles.
	*/
	void coordinates2angle();

	/*Remove the extra offset in z-axis.
	*/
	void offsetZaxis();

	/*Convert the difference of angles in each transistions to the numbers
	* of steps that are required to be executed by the stepper motor.
	*/
	void angle2step();

	/*An inline function for int motion(...):
	* Its purpose is to reselect the suitable max cruising speed, 
	* and also readjust the motion time accordingly.
	* (Eq. 18 and 19 from Working Principle.pdf.)
	*/
	inline void retuneSpeedTime(const int& step,
		const float& maxSpeedUser, const float& hardwareSpeedLimit,
    		MotionParam& motionParam);

	/*An inline function for int motion(...):
	* Calculate the constants for Eq.21, so that it can speed up
	* the calculation process of the inner while loop of 
	* int motion(...).
	*/
	inline void parametersCalc(const int& step, 
		MotionParam& motionParam);

	/*An inline function for int motion(...):
	* Calculate the half-delta or delta prime in Eq. 21
	* from Working Principle.pdf.
	*/
	inline void calculationBlock(MotionParam& motionParam);

	/*An inline function for int motion(...):
	* To ensure the stepping process is performing after the time
	* difference exceeds the half delta calculated.
	*/
	inline void steppingDecision(const unsigned long& timeNow,
    		const int& step, const byte& stepperPin,
    		MotionParam& motionParam);
};

#endif
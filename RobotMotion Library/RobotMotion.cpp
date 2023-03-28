/*GitHub Page: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library
* API tutorial: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/RobotMotion%20Library/Brief%20explanation.md
* Documentation: https://github.com/CK-Explorer/A-Simple-Open-Loop-Robotic-Motion--SCARA-robot--Library/blob/main/Documentation/Working%20Principle.pdf
*/

#include "RobotMotion.h"

RobotMotion::RobotMotion(const int& no_Coordinates, 
    const bool& selfConfigure) : no_Coordinates(no_Coordinates),
    selfConfigure(selfConfigure)
{
    setValues = false;
    position.x = nullptr;
    position.y = nullptr;
    position.z = nullptr;
    time.t1 = nullptr;
    time.t2 = nullptr;
    time.tZaxis = nullptr;
    maxCruisingSpeed.v1 = nullptr;
    maxCruisingSpeed.v2 = nullptr;
    maxCruisingSpeed.vZaxis = nullptr;

    if (no_Coordinates > 1 && !selfConfigure)
    {
        position.x = new float[no_Coordinates];
        position.y = new float[no_Coordinates];
        position.z = new float[no_Coordinates];
        angleMotor1 = new float[no_Coordinates];
        angleMotor2 = new float[no_Coordinates];
        zNew = new float[no_Coordinates];
        time.t1 = new float[no_Coordinates - 1];
        time.t2 = new float[no_Coordinates - 1];
        time.tZaxis = new float[no_Coordinates - 1];
        maxCruisingSpeed.v1 = new float[no_Coordinates - 1];
        maxCruisingSpeed.v2 = new float[no_Coordinates - 1];
        maxCruisingSpeed.vZaxis = new float[no_Coordinates - 1];
        step1 = new int[no_Coordinates - 1];
        step2 = new int[no_Coordinates - 1];
        stepZaxis = new int[no_Coordinates - 1];
        direction1 = new int[no_Coordinates - 1];
        direction2 = new int[no_Coordinates - 1];
        directionZaxis = new int[no_Coordinates -1];
    }
}

RobotMotion::~RobotMotion()
{
    if (no_Coordinates > 1)
    {
        if (!selfConfigure)
        {
            delete[] position.x;
            delete[] position.y;
            delete[] position.z;
            delete[] time.t1;
            delete[] time.t2;
            delete[] time.tZaxis;
            delete[] maxCruisingSpeed.v1;
            delete[] maxCruisingSpeed.v2;
            delete[] maxCruisingSpeed.vZaxis;
        }
        delete[] angleMotor1;
        delete[] angleMotor2;
        delete[] zNew;
        delete[] step1;
        delete[] step2;
        delete[] stepZaxis;
        delete[] direction1;
        delete[] direction2;
        delete[] directionZaxis;
    }
}

int RobotMotion::setValuesBool(const bool& setValues)
{
    if (no_Coordinates <= 1)
        return -1;

    if (!position.x && !position.y && !position.z &&
        !time.t1 && !time.t2 && !time.tZaxis && 
        !maxCruisingSpeed.v1 && !maxCruisingSpeed.v2 &&
        !maxCruisingSpeed.vZaxis)
        return -2;
        
	this->setValues = setValues;
    if(selfConfigure == true)
    {
        angleMotor1 = new float[no_Coordinates];
        angleMotor2 = new float[no_Coordinates];
        zNew = new float[no_Coordinates];
        step1 = new int[no_Coordinates - 1];
        step2 = new int[no_Coordinates - 1];
        stepZaxis = new int[no_Coordinates - 1];
        direction1 = new int[no_Coordinates - 1];
        direction2 = new int[no_Coordinates - 1];
        directionZaxis = new int[no_Coordinates -1];
    }
    return 0;
}

int RobotMotion::cooordinates2step()
{
    if (setValues == false)
        return -1;

    offsetZaxis();
    coordinates2angle();
    angle2step();
    return 0;
}

inline void RobotMotion::offsetZaxis()
{
    for (int count = 0; count < no_Coordinates; count++)
        zNew[count] = position.z[count] - zOffset;
}

inline void RobotMotion::coordinates2angle()
{
    for (int count = 0; count < no_Coordinates; count++)
    {
        //theta1 calculation
        float L3 = sqrt(position.x[count] * position.x[count] 
            + position.y[count] * position.y[count]);
        float alpha = acos((armLength1 * armLength1 + L3 * L3 
            - armLength2 * armLength2)  / (2 * armLength1 * L3));
        float phi = fabs(atan(position.y[count] / position.x[count]));
        float theta1 = phi + alpha;

        if (count == 0 && position.x[count] < 0)
            theta1 = M_PI - phi + alpha;

        else if (count >= 1 && position.x[count] < 0)
            theta1 = (M_PI - theta1);

        angleMotor1[count] = theta1;

        //theta2 calculation
        float theta2 = asin((1 / armLength2) * 
            (position.y[count] - armLength1 * sin(theta1)));
        float x_rel_L2 = position.x[count] - armLength1 * cos(theta1);
        float y_rel_L2 = position.y[count] - armLength1 * sin(theta1);

        //2nd quadrant
        if (x_rel_L2 < 0 && y_rel_L2>0)
            angleMotor2[count] = (M_PI - theta2);
        //3rd quadrant
        else if (x_rel_L2 <= 0 && y_rel_L2 <= 0)
            angleMotor2[count] = (M_PI - theta2);
        //4th quadrant
        else if (x_rel_L2 > 0 && y_rel_L2 < 0)
            angleMotor2[count] = (2 * M_PI + theta2);
        //1st quadrant
        else
            angleMotor2[count] = theta2;
    }
}

inline void RobotMotion::angle2step()
{
    for (int count = 1; count < no_Coordinates; count++)
    {
        float delta_theta_1 = angleMotor1[count] - angleMotor1[count - 1];
        float delta_theta_2 = angleMotor2[count] - angleMotor2[count - 1];
        float delta_z = zNew[count] - zNew[count - 1];

        /*To determine the revolution number*/
        int n1 = delta_theta_1 / (2 * M_PI);
        int n2 = delta_theta_2 / (2 * M_PI);
        delta_theta_1 = (delta_theta_1 - (2 * M_PI * n1)) * teethRatio1;
        delta_theta_2 = (delta_theta_2 - (2 * M_PI * n2));

        if (position.x[count] < 0 && position.x[count - 1] >0 
            && delta_theta_2 < 0)
            delta_theta_2 = 2 * M_PI + delta_theta_2;
        else if (position.x[count] >= 0 && position.x[count - 1] < 0 
            && delta_theta_2 >0)
            delta_theta_2 = delta_theta_2 - 2 * M_PI;

        delta_theta_2 *= teethRatio2;

        step1[count - 1] = int(round(delta_theta_1 / (2 * M_PI) 
            * stepPerRev1));
        if (step1[count - 1] < 0)
        {
            step1[count - 1] *= -1.00;
            direction1[count - 1] = HIGH; //clockwise
        }
        else
            direction1[count - 1] = LOW; //anticlockwise
           

        step2[count - 1] = int(round(delta_theta_2 / (2 * M_PI) 
            * stepPerRev2));
        if (step2[count - 1] < 0)
        {
            step2[count - 1] *= -1.00;
            direction2[count - 1] = HIGH; //clockwise
        }
        else
            direction2[count - 1] = LOW; //anticlockwise
            

        stepZaxis[count - 1] = int(round(delta_z * stepPerRevZaxis 
            / lead));
        if (stepZaxis[count - 1] < 0)
        {
            stepZaxis[count - 1] *= -1.00;
            directionZaxis[count - 1] = HIGH; //clockwise
        }
        else
            directionZaxis[count - 1] = LOW; //anticlockwise
  
    }
}

int RobotMotion::motion(void(*speedTimeFcn)(float, float, float, 
        float, float, float, int), 
		void(*endEffectorFcn)(int))
{
    if (setValues == false)
        return -1;

    for (int count = 0; count < (no_Coordinates - 1); count++)
    {
        /*Initialize this variables every loop*/
        MotionParam param1, param2, paramZaxis;
        param1.tf_s = time.t1[count];
        param2.tf_s = time.t2[count];
        paramZaxis.tf_s = time.tZaxis[count];

        /*calculation*/
        /*Readjust max cruising speed and motion time*/
        retuneSpeedTime(step1[count], maxCruisingSpeed.v1[count],
            limitSpeed_v1, param1);
        retuneSpeedTime(step2[count], maxCruisingSpeed.v2[count],
            limitSpeed_v2, param2);
        retuneSpeedTime(stepZaxis[count], maxCruisingSpeed.vZaxis[count],
            limitSpeed_vZaxis, paramZaxis);

        speedTimeFcn(param1.validSpeed, param2.validSpeed, paramZaxis.validSpeed, 
            param1.tf_s, param2.tf_s, paramZaxis.tf_s, count);

        /*Calculating constants to speed up calculation later*/
        parametersCalc(step1[count], param1);
        parametersCalc(step2[count], param2);
        parametersCalc(stepZaxis[count], paramZaxis);

        /*Initialize the stepperPin*/
        digitalWrite(stepperPin_p1, param1.stepper_state);
        digitalWrite(stepperPin_p2, param2.stepper_state);
        digitalWrite(stepperPin_pZaxis, paramZaxis.stepper_state);

        /*Setting the directions of rotations*/
        digitalWrite(stepperDirPin_p1, direction1[count]);
        digitalWrite(stepperDirPin_p2, direction2[count]);
        digitalWrite(stepperDirPin_pZaxis, directionZaxis[count]);
        
        param1.previousMicros = micros();
        param2.previousMicros = micros();
        paramZaxis.previousMicros = micros();
        unsigned long currentMicros;

        while (param1.number_step < step1[count] || param2.number_step < step2[count] 
            || paramZaxis.number_step < stepZaxis[count])
        {
            /*Calculation for half delta*/
            calculationBlock(param1);
            calculationBlock(param2);
            calculationBlock(paramZaxis);

            currentMicros = micros();
            
            /*To ensure the stepping process is done when the time difference
            * is more than the half-delta calculated above.
            */
            steppingDecision(currentMicros, step1[count], 
                stepperPin_p1, param1);
            steppingDecision(currentMicros, step2[count], 
                stepperPin_p2, param2);
            steppingDecision(currentMicros, stepZaxis[count], 
                stepperPin_pZaxis, paramZaxis);
        }
        endEffectorFcn(count);
    }
    return 0;
}

void RobotMotion::printStepDir()
{
    for (int i = 0; i < no_Coordinates - 1; i++)
    {
        Serial.print(" ( ");
        Serial.print(i);
        Serial.print(" ) = ");
        Serial.print(step1[i]);
        Serial.print("\t");
        Serial.print(step2[i]);
        Serial.print("\t");
        Serial.print(stepZaxis[i]);
        Serial.print("\t");
        Serial.print(direction1[i]);
        Serial.print("\t");
        Serial.print(direction2[i]);
        Serial.print("\t");
        Serial.print(directionZaxis[i]);
        Serial.print("\n");
    }
}

inline void RobotMotion::retuneSpeedTime(const int& step,
    const float& maxSpeedUser, const float& hardwareSpeedLimit,
    MotionParam& motionParam)
{
    float speed = step * 1.0 / motionParam.tf_s;

    if (maxSpeedUser > ceil(speed) && 
        maxSpeedUser < floor(2.00 * speed))
        motionParam.validSpeed = round(maxSpeedUser);
    else if (maxSpeedUser > floor(2.00 * speed))
        motionParam.validSpeed = floor(2 * speed);
    else
        motionParam.validSpeed = ceil(speed);

    if (motionParam.validSpeed > hardwareSpeedLimit)
    {
        motionParam.validSpeed = floor(hardwareSpeedLimit);
        motionParam.tf_s = 2.00 * step / motionParam.validSpeed;
    }
}

inline void RobotMotion::parametersCalc(const int& step, MotionParam& motionParam)
{
    /*Starting of third portion of the time interval (in microsecond)*/
    motionParam.tb_3 = (step * 1.00 / motionParam.validSpeed)
        * 1000000.00;

    /*blending time and also the end of the first portion of time interval (in microsecond)*/
    motionParam.tb_1 = (motionParam.tb_3 * -1.00) + (motionParam.tf_s * 1000000.00);

    /*Coefficient for the time of moving one step of stepper motor for
    * 2nd portion of time interval
    * 0.5 = halving the time for high to low transition
    */
    motionParam.invOmega = 1000000.00 / motionParam.validSpeed * 0.5;

    /*Coefficient for the time of moving one step of stepper motor for
    * 1st and 3rd portions
    */
    motionParam.constant = motionParam.tb_1 * motionParam.invOmega;

    /*initialize change of time for the first half delta*/
    motionParam.change_time = sqrt(2.00 * motionParam.tb_1 
        / motionParam.validSpeed) * 1000.00 * 0.5;

    motionParam.tf_ms = motionParam.tf_s * 1000000.00;
}

inline void RobotMotion::calculationBlock(MotionParam& motionParam)
{
    if (motionParam.calculation_enable == HIGH) 
    {
        motionParam.acc_time += motionParam.change_time;
        if (motionParam.acc_time < motionParam.tb_1)
            motionParam.change_time = motionParam.constant / motionParam.acc_time;
        else if (motionParam.acc_time <= motionParam.tb_3 
            && motionParam.acc_time >= motionParam.tb_1)
            motionParam.change_time = motionParam.invOmega;
        else if (motionParam.acc_time >= motionParam.tb_3 
            && motionParam.acc_time <= motionParam.tf_ms)
            motionParam.change_time = motionParam.constant 
                / (motionParam.tf_ms - motionParam.acc_time);
        motionParam.calculation_enable = LOW;
    }
}

inline void RobotMotion::steppingDecision(const unsigned long& timeNow,
    const int& step, const byte& stepperPin,
    MotionParam& motionParam)
{
    if (timeNow - motionParam.previousMicros >= motionParam.change_time 
        && motionParam.number_step < step) 
    {
        if (motionParam.stepper_state == LOW)
        {
            motionParam.stepper_state = HIGH; 
            motionParam.number_step++; 
        } 
        else 
            motionParam.stepper_state = LOW;
        motionParam.previousMicros = timeNow;
        motionParam.calculation_enable = HIGH;
        digitalWrite(stepperPin, motionParam.stepper_state);
    }
}

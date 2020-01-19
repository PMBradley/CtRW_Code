package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class DropServo_DuoMirrored {
    private Robot2019 robot;
    private Servo dropL;
    private Servo dropR;


    public DropServo_DuoMirrored (Robot2019 robot, Servo servoL, Servo servoR)
    {
        this.robot = robot;
        dropL = servoL;
        dropR = servoR;
    }


    boolean isInitilized = false;
    public double reverseMod = 1; // modifier used to reverse the servos if going the wrong direction
    double rangeMax = 1;
    double rangeMin = 0;

    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;
        dropL.setPosition(0);
        dropR.setPosition(0);
    }

    private double powerNegate(double inputPower, double negegationVal){
        inputPower -= .5;
        inputPower *= negegationVal;
        inputPower += .5;

        return (inputPower);
    }

    private double mirrorComp(double inputPower, double rangeMax){
        return (rangeMax - inputPower);
    }

    public void set_reverseMod(double inputMod){
        reverseMod = inputMod;
    }
    public void set_rangeMinMax(double min, double max){
        rangeMin = min;
        rangeMax = max;
    }


    public void set_ServoPower(double servoPower, Servo servoL, Servo servoR){
        dropL = servoL;
        dropR = servoR;

        servoPower = powerNegate(servoPower, reverseMod);

        servoL.setPosition(servoPower);
        servoR.setPosition(mirrorComp(servoPower, 1));
    }

}

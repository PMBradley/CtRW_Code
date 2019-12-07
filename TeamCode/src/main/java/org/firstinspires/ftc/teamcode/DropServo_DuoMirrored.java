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
    double reverseMod = 1; // modifier used to reverse the servos if going the wrong direction


    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;
        dropL.setPosition(0);
        dropR.setPosition(0);
    }

    public void set_reverseMod(double inputMod){
        reverseMod = inputMod;
    }

    public void set_ServoPower(double servoPower, Servo servoL, Servo servoR){
        dropL = servoL;
        dropR = servoR;

        servoL.setPosition(servoPower * reverseMod);
        servoR.setPosition(-servoPower * reverseMod);
    }

}

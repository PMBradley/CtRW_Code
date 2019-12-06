package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class DropServo_DuoMirrored {
    private Robot2019 robot;

    public DropServo_DuoMirrored (Robot2019 robot, Servo servol, Servo servoR)
    {
        this.robot = robot;
        this.dropL = servol;
        this.dropR = servoR;
    }


    boolean isInitilized = false;
    double reverseMod = 1; // modifier used to reverse the servos if going the wrong direction

    Servo dropL = null;
    Servo dropR = null;

    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;
        dropL.setPosition(0);
        dropR.setPosition(0);
    }

    public void set_reverseMod(double inputMod){
        reverseMod = inputMod;
    }

    public void set_ServoPower(double servoPower){
        dropL.setPosition(servoPower * reverseMod);
        dropR.setPosition(-servoPower * reverseMod);
    }

}

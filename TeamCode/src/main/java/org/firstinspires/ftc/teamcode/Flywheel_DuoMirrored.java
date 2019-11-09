package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel_DuoMirrored {

    private Robot2019 robot;

    public Flywheel_DuoMirrored (Robot2019 robot)
    {
        this.robot = robot;
    }


    boolean isInitilized = false;


    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;
        robot.motorIntakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorIntakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void set_Power(double power){

        robot.motorIntakeL.setPower(power);
        robot.motorIntakeL.setPower(-power);
    }

}
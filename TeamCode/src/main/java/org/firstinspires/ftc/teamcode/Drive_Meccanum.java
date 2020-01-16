package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive_Meccanum {

    private Robot2019 robot;


    public Drive_Meccanum(Robot2019 robot)
    {
        this.robot = robot;
    }



    // Math Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    boolean isInitilized = false;


    public void init_motors(){
        isInitilized = true; // flips dat flag

        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // stops motors and resets encoders
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Drive_Vector(double xPower, double yPower, double rPower) // drives based off of powers and relative to the field
    {


    }


    public void drive_Controller(double lStickX, double lStickY, double rStickX) // drives based off of controller inputs and relative to the robot
    {
        double magnitude = Math.hypot(lStickX, lStickY);
        double robotAngle = Math.atan2(lStickY, lStickX)- Math.PI/4;
        double rightX = rStickX;


        final double driveFLPwr = magnitude * Math.cos(robotAngle) + rightX; // set each wheel power to the value resulting from the math
        final double driveFRPwr = magnitude * Math.sin(robotAngle) - rightX;
        final double driveBLPwr = magnitude * Math.sin(robotAngle) + rightX;
        final double driveBRPwr = magnitude * Math.cos(robotAngle) + rightX;

        robot.driveFL.setPower(-driveFLPwr);
        robot.driveFR.setPower(-driveFRPwr);
        robot.driveBL.setPower(driveBLPwr);
        robot.driveBR.setPower(driveBRPwr);
    }
}

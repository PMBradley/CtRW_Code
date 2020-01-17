package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive_Meccanum {


  //  Robot2019 robot = new Robot2019();

    private Robot2019 robot;


    public Drive_Meccanum(Robot2019 robot)
    {
        this.robot = robot;
    }



    //Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    boolean isInitilized = false;


    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;


        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public void Drive_Vector(double x, double y, double r, double gyroHeading)
    {
        //if(!isInitilized)
          //  init_motors();
      //  y *= -1;
       // r *= -1;

        double sin = Math.sin(-gyroHeading * 0.0174533);
        double cos = Math.cos(-gyroHeading * 0.0174533);

        double forward = x * sin + y * cos;
        double clockwise = r;
        double right = x * cos - y * sin;

        double fl = forward + clockwise + right;
        double fr = forward - clockwise - right;
        double bl = forward + clockwise - right;
        double br = forward - clockwise + right;

        robot.driveFL.setPower(-fl);
        robot.driveFR.setPower(fr);
        robot.driveBL.setPower(-bl);
        robot.driveBR.setPower(br);
    }

    public void drive_Controller(double lStickX, double lStickY, double rStickX)
    {
       /* if(!isInitilized)
            init_motors();
       */
        double magnitude = Math.hypot(lStickX, lStickY);
        double robotAngle = Math.atan2(lStickY, lStickX)- Math.PI/4;
        double rightX = rStickX;



        final double driveFLPwr = magnitude * Math.cos(robotAngle) + rightX;
        final double driveFRPwr = magnitude * Math.sin(robotAngle) - rightX;
        final double driveBLPwr = magnitude * Math.sin(robotAngle) + rightX;
        final double driveBRPwr = magnitude * Math.cos(robotAngle) + rightX;

        robot.driveFL.setPower(driveFLPwr);
        robot.driveFR.setPower(driveFRPwr);
        robot.driveBL.setPower(driveBLPwr);
        robot.driveBR.setPower(driveBRPwr);
    }
}

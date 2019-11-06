

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;




public class Motion extends LinearOpMode {

    /* Declare OpMode members. */
    Robot2019         robot   = new Robot2019();


    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    public void init_motors(){
        robot.init(hardwareMap);
        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    @Override
    public void runOpMode() {


    }


    public void manDrive() {

        double magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)- Math.PI/4;
        double rightX = gamepad1.right_stick_x;

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



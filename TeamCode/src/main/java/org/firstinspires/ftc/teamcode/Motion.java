

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
       // robot.init(hardwareMap);
        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }


    @Override
    public void runOpMode() {


    }


    public void Drive(boolean AUTO_MODE_ACTIVE) {
        robot.init(hardwareMap);
        // This is a simplified version of the mecanum drive code we used last year

        double magnitude = 0.0;
        double robotAngle = 0.0;
        double rightX = 0.0;

        if (AUTO_MODE_ACTIVE == true){

            /* Need to develop autonomous code, you may want to think through how
               you want to deal with your waypoints, you will need to do some math to
               get your magnitude, angle and turn (rightX).

               You also will have to decide if you want to use encoders here or just rely on the
               lidar sensors.
             */
        }

        if (AUTO_MODE_ACTIVE == false){
            // If you are in manual mode
            magnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)- Math.PI/4;
            rightX = gamepad1.right_stick_x;

        }



        final double driveFLPwr = magnitude * Math.cos(robotAngle) + rightX;
        final double driveFRPwr = magnitude * Math.sin(robotAngle) - rightX;
        final double driveBLPwr = magnitude * Math.sin(robotAngle) + rightX;
        final double driveBRPwr = magnitude * Math.cos(robotAngle) + rightX;

        robot.driveFL.setPower(driveFLPwr);
        robot.driveFR.setPower(driveFRPwr);
        robot.driveBL.setPower(driveBLPwr);
        robot.driveBR.setPower(driveBRPwr);


        }

        /* You also need to decide if you want to have the lift, grabber, etc. in this same class or
           somewhere else. It may make more sense to have them all here since they are tied to the robot
           object created here, this class also references the gamepads.
         */


    }



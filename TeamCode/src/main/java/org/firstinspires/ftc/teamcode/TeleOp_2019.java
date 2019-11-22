package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends OpMode {



    Robot2019 robot = new Robot2019();
    MainControl control   = new MainControl(robot);

//    Drive_Meccanum   meccanum = new Drive_Meccanum(robot);
//    Flywheel_DuoMirrored flyIntake = new Flywheel_DuoMirrored(robot);
//    Lift_Linear lift = new Lift_Linear(robot);
//    Arm_Swing arm_swing = new Arm_Swing(robot);
//
//    double stick1X;
//    double stick1Y;
//    double stick2X;
//    double rtrigger;
//    double ltrigger;
//    boolean armSwingIn = false;
//    boolean armSwingOut = false;
//    boolean rampUpBtn;
//    boolean rampDownBtn;
//    boolean spinIntakeIn = false;
//    boolean spinIntakeOut = false;
//    boolean clampRelease = false;
//    boolean clampClose = false;
//    boolean clampAction = false;

    // you can just call the other subroutines here and pass a mode flag


     // Code to run ONCE when the driver hits INIT
    public void init(){
        // You can just set the manual mode flag and call the manual mode routine
        control.Set_AutoMode(false); // disables autonomous mode and initializes all auto

        robot.init(hardwareMap);
        control.controlInit(robot);
        telemetry.addData("Say", "It's Droopy McCool Time!");
    }

    public void updateControls(){
        robot.gp1_lstickX =  gamepad1.left_stick_x;
        robot.gp1_lstickY =  gamepad1.left_stick_y;
        robot.gp1_rstickX =  gamepad1.right_stick_x;
        robot.gp1_rbumper = gamepad1.right_bumper;
        robot.gp1_lbumper = gamepad1.left_bumper;
        robot.gp1_a = gamepad1.a;
        robot.gp1_b = gamepad1.b;
        robot.gp2_rtrigger = gamepad2.right_trigger;
        robot.gp2_ltrigger = gamepad2.left_trigger;
        robot.gp2_lbumper = gamepad2.left_bumper;
        robot.gp2_rbumper = gamepad2.right_bumper;
        //clampRelease = gamepad2.dpad_up;
        //clampClose = gamepad2.dpad_down;
        robot.gp2_x = gamepad2.x;

        // Add other buttons

    }

    public void loop(){


       updateControls();

        // Manual subroutines

        control.AUTO_MODE_ACTIVE = false;
        control.manual_mode();

        //control.manual_mode(-stick1Y, stick1X, -stick2X);
//        meccanum.Drive_Controller(-stick1Y, stick1X, -stick2X);
//        flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
//        flyIntake.set_Ramp_Position(rampUpBtn, rampDownBtn);
//        lift.move_Linear(rtrigger , ltrigger);
//        arm_swing.set_arm_position(armSwingIn, armSwingOut);
//        arm_swing.set_clamp_position(clampAction);

    }

    }

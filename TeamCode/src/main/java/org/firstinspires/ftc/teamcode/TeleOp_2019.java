package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends OpMode {



    Robot2019 robot = new Robot2019();
    MainControl control   = new MainControl(robot);
    Drive_Meccanum   meccanum = new Drive_Meccanum(robot);
    Flywheel_DuoMirrored flyIntake = new Flywheel_DuoMirrored(robot);
    Lift_Linear lift = new Lift_Linear(robot);

    double stick1X;
    double stick1Y;
    double stick2X;
    double rtrigger;
    double ltrigger;
    boolean spinIntakeIn = false;
    boolean spinIntakeOut = false;

    // you can just call the other subroutines here and pass a mode flag


     // Code to run ONCE when the driver hits INIT
    public void init(){
        // You can just set the manual mode flag and call the manual mode routine
        control.Set_AutoMode(false); // disables autonomous mode and initializes all auto
        robot.init(hardwareMap);
        telemetry.addData("Say", "It's Droopy McCool Time!");
    }

    public void get_joysticks(){
        stick1X =  gamepad1.left_stick_x;
        stick1Y =  gamepad1.left_stick_y;
        stick2X =  gamepad1.right_stick_x;
        spinIntakeIn = gamepad1.right_bumper;
        spinIntakeOut = gamepad1.left_bumper;
        rtrigger = gamepad2.right_trigger;
        ltrigger = gamepad2.left_trigger;

        // Add other buttons

    }

    public void loop(){


        get_joysticks();

        // Manual subroutines

        //control.manual_mode(-stick1Y, stick1X, -stick2X);
        meccanum.Drive_Controller(-stick1Y, stick1X, -stick2X);
        flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
        lift.move_Linear(rtrigger , ltrigger);


    }

    }

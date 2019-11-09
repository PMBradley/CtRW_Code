package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends OpMode {

    public MainControl control   = new MainControl();

    private Robot2019 robot;

    public TeleOp_2019(Robot2019 robot) { this.robot = robot;}

     // Code to run ONCE when the driver hits INIT
    public void init(){
        // You can just set the manual mode flag and call the manual mode routine
        control.Set_AutoMode(false); // disables autonomous mode and initializes all auto
        robot.init(hardwareMap);
        telemetry.addData("Say", "It's Droopy McCool Time!");
    }

    public void loop(){
        control.manual_mode();

    }

    }

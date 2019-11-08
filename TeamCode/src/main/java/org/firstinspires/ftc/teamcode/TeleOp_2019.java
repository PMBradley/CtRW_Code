package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends LinearOpMode {

    public MainControl         control   = new MainControl();

    @Override
    public void runOpMode() {
        waitForStart();

        // You can just set the manual mode flag and call the manual mode routine
        control.Set_AutoMode(false); // disables autonomous mode and initializes all auto

        while (opModeIsActive()){

            control.manual_mode();




        }
    }
}

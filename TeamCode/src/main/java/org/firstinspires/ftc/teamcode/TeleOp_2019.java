package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends LinearOpMode {

    MainControl         control   = new MainControl();

    @Override
    public void runOpMode() {
        waitForStart();

        // You can just set the manual mode flag and call the manual mode routine
        control.AUTO_MODE_ACTIVE = false;

        while (opModeIsActive()){

            control.manual_mode();




        }
    }
}

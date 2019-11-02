package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp_2019")
public class TeleOp_2019 extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        OpModes teleOp = new OpModes("TeleOp");

        while (opModeIsActive()){
            teleOp.run();
        }
    }
}

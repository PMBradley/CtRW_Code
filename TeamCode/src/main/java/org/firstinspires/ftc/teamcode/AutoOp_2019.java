package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AutoOp_2019")
public class AutoOp_2019 extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        OpModes autoOp = new OpModes("AutoOp");

        while (opModeIsActive()){
            autoOp.run();
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AutoOp_2019")
public class AutoOp_2019 extends LinearOpMode {

    MainControl         control   = new MainControl();

    @Override
    public void runOpMode() {
        waitForStart();

        OpModes autoOp = new OpModes("AutoOp");
        control.AUTO_MODE_ACTIVE = true;

        while (opModeIsActive()){
            // Maybe call the statemachine on loop here, we can can time here since you already extended linearopmode.
            // Loop should keep calling the statemachine until time is done.
            control.AutoStep();



            autoOp.run();
        }
    }
}
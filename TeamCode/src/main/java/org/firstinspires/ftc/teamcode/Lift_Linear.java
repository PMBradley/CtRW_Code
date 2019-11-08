package org.firstinspires.ftc.teamcode;

public class Lift_Linear {
    Robot2019 robot;

    void move_Linear(double linearPower)
    {
        if (robot.touchLift0.getState()) {
            if(linearPower < 0) {
                robot.motorLift.setPower(0);
            }
        }
        else{
            robot.motorLift.setPower(linearPower);
        }
    }
}

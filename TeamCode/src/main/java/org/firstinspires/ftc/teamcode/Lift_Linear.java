package org.firstinspires.ftc.teamcode;

public class Lift_Linear {
    Robot2019 robot;

    public Lift_Linear(Robot2019 robot)
    {
        this.robot = robot;
    }
    void move_Linear(double rpower, double lpower)
    {
        if (rpower>0)
        {
            robot.motorLift.setPower(rpower);
        }
        else if (lpower>0)
        {
            lpower = lpower * -1.0;
            robot.motorLift.setPower(lpower);
        }
    }
}

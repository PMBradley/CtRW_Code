
package org.firstinspires.ftc.teamcode;



public class Lift_Linear {

    Robot2019 robot;

    double ZERO_VALUE = 0.05; // used to prevent tiny ghost inputs

    public Lift_Linear(Robot2019 robot)

    {

        this.robot = robot;

    }

    void move_Controller(double rpower, double lpower)

    {

        if (rpower > ZERO_VALUE)

        {

            rpower = -rpower;

            robot.motorLift.setPower(rpower);

        }

        else if (lpower > ZERO_VALUE)

        {
            robot.motorLift.setPower(lpower);
        }

        else {
            robot.motorLift.setPower(0);
        }

    }

}
package org.firstinspires.ftc.teamcode;

public class Arm_Swing {
    private  Robot2019 robot;
    //Variables
    public Arm_Swing(Robot2019 robot){
        this.robot = robot;
    }


    double armBaseRotation = 0.0;
    double armOpenPosition = 1.0;
    double armClosedPosition = 0.0;
    double clampReleasePosition = 1.0;
    double clampClosePosition = 0.0;
    double wristRotation = 0.0;
    boolean armStopTriggerOpen = false;
    boolean armStopTriggerClose = false;



    public void set_arm_position(boolean armDirectionIn, boolean armDirectionOut){

        double direction = 0.0;
        if(armDirectionIn)
            direction = armClosedPosition;
        else if(armDirectionOut)
            direction = armOpenPosition;
        robot.armPivot.setPosition(direction);

    }
    public void set_clamp_position(boolean clampDirectionRelease, boolean clampDirectionClose){

        double direction = 0.0;
        if(clampDirectionRelease)
            direction = clampReleasePosition;
        else if(clampDirectionClose)
            direction = clampClosePosition;
        robot.armGrab.setPosition(direction);

    }

}

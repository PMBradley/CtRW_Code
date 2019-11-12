package org.firstinspires.ftc.teamcode;

public class Arm_Swing {
    private  Robot2019 robot;
    //Variables
    public Arm_Swing(Robot2019 robot){
        this.robot = robot;
    }


    double armBaseRotation = 0.0;
    double wristRotation = 0.0;
    boolean armStopTriggerOpen = false;
    boolean armStopTriggerClose = false;



    public void fullArmRotation(double armSpeed, int armDirection){

        if (armDirection == 1) {
            //The arm is going to fully open
        }

        if (armDirection == 0) {
            //The arm is going to fully closed
        }
    }


}

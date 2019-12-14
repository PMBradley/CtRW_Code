package org.firstinspires.ftc.teamcode;

public class Arm_Swing {
    private  Robot2019 robot;
    //Variables
    public Arm_Swing(Robot2019 robot){
        this.robot = robot;
    }


    double armBaseRotation = 0.0;
    double armOpenPosition = 0.0;
    double armClosedPosition = 0.60;
    double armCurrentPosition = 0;
    double clampReleasePosition = 0.8;
    double clampClosePosition = 0.25;
    double wristRotation = 0.0;
    boolean armStopTriggerOpen = false;
    boolean armStopTriggerClose = false;
    double armDirection = 1.0;
    double lastArmDirection = 1.0;
    double clampDirection = 1.0;
    public boolean clampIsClosed = true;



    public void set_arm_position(boolean armDirectionIn, boolean armDirectionOut ){

            if(armDirectionIn){
                armDirection = armOpenPosition;

            }
            else if (armDirectionOut){
                armDirection = armClosedPosition;

            }
            robot.armPivot.setPosition(armDirection);



    }
    public void toggle_clamp_position(boolean clampAction){ // toggles the clamp position if a 1


        if((clampAction)&& (clampIsClosed == false)) {

            clampDirection = clampClosePosition;
            clampIsClosed = true;
        }


        else if((clampAction)&& (clampIsClosed)) {
            clampDirection = clampReleasePosition;
            clampIsClosed = false;
        }

        robot.armGrab.setPosition(clampDirection);

    }

    public void set_clamp_position(boolean clampRelease){
        if(clampRelease){
            clampDirection = clampReleasePosition;
        }
        else {
            clampDirection = clampClosePosition;
        }

        robot.armGrab.setPosition(clampDirection);
    }


}

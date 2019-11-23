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
    double armDirection = 1.0;
    double lastArmDirection = 1.0;
    double clampDirection = 1.0;
    public boolean clampIsClosed = true;



    public void set_arm_position(boolean armDirectionIn, boolean armDirectionOut ){

            if(armDirectionIn){
                armDirection = armClosedPosition;

            }
            else if (armDirectionOut){
                armDirection = armOpenPosition;

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

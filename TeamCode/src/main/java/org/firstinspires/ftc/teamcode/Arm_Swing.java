package org.firstinspires.ftc.teamcode;

public class Arm_Swing {
    private  Robot2019 robot;
    //Variables
    public Arm_Swing(Robot2019 robot){
        this.robot = robot;
    }


    double armBaseRotation = 0.0;
    double armOpenPosition = 0.95;
    double armClosedPosition = 0.00;
    double armCurrentPosition = 0;
    double clampReleasePosition = 1.0;
    double clampClosePosition = 0.0;
    double wristRotation = 0.0;
    boolean armStopTriggerOpen = false;
    boolean armStopTriggerClose = false;
    double armDirection = 0.69;
    double lastArmDirection = 1.0;
    double clampDirection = 1.0;
    public boolean clampIsClosed = true;



    public void set_arm_position(boolean armDirectionIn, boolean armDirectionOut ){ // use boolean variables to set the arm position to either in or out or not actively either

            if(armDirectionIn){
                armDirection = armOpenPosition;
            }
            else if (armDirectionOut){
                armDirection = armClosedPosition;
            }
            else {
                armDirection = robot.armPivot.getPosition(); // stay put
            }

            robot.armPivot.setPosition(armDirection);

    }
    public void toggle_clamp_position(boolean clampAction){ // toggles the clamp position if parameter is true
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

    public void set_clamp_position(boolean clampRelease){ // sets the clamp attachment state to either closed or open
        if(clampRelease){
            clampDirection = clampReleasePosition;
        }
        else {
            clampDirection = clampClosePosition;
        }

        robot.armGrab.setPosition(clampDirection);
    }
}

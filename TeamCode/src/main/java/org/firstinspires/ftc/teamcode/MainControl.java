package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MainControl")
public class MainControl extends OpMode {

    //Key parameter variables


    //HardwareMap hMap = robot.mainMap;

    private Robot2019 robot = new Robot2019();
    private ElapsedTime runtime  = new ElapsedTime();


  /* public MainControl(Robot2019 robot)
    {
        this.robot = robot;
    }
  */





    Arm_Swing arm_swing = new Arm_Swing(robot);
    Navigation2019 navigation = new Navigation2019(robot);
    Drive_Meccanum   meccanum = new Drive_Meccanum(robot);
    Flywheel_DuoMirrored flyIntake = new Flywheel_DuoMirrored(robot);
    Lift_Linear lift = new Lift_Linear(robot);
    DropServo_DuoMirrored intakeDrop = new DropServo_DuoMirrored(robot, robot.intakeDropL, robot.intakeDropR);
    DropServo_DuoMirrored pullerDrop = new DropServo_DuoMirrored(robot, robot.pullerDropL, robot.pullerDropR);

    public int INIT_FIELD_POS = 0; // Quad 1,2,3,4
    public int AUTO_RUNS_TOT = 3;
    public int CURR_RUN_TOT = 0;

    public int MODE_CHOICE_TIME = 3000;


    // Startup flags
    public boolean LOOP_FIRST_RUN = true; // used to indicate if it is the first run of the main loop
    public boolean ROBOT_DEAD_STOP = false;
    public boolean SENSORS_READY = false;
    public boolean DRIVE_READY = false;
    public boolean INIT_COMPLETE = false;
    public boolean INIT_FAIL = false;
    public boolean AUTO_MODE_ACTIVE = true; // assumes autonomous until given an input



    // Game Flags
    public boolean WAYPOINT_REACHED = false;
    private boolean DRIVE_SEQ_COMPLETE = false;
    public boolean ROBOT_IS_IDLE = false;
    public boolean BLOCK_CAPTURED = false;
    public boolean BLOCK_FOUND = false;
    public boolean ROBOT_INTAKE_POS = false;
    public boolean ROBOT_PLACE_POS = false;

    //Game double
    public double LIFT_LEVEL = 0.0;




    // Startup initialization

    // Set flag for either Auto or Manual based on Opmode selected
    // If flag is in Auto get initial field position from naviagtion routine
    // If flag is manual mode do not call navigation routine
    // Check sensors and drive and set initialization ok flag

// I would call manual mode from the TeleOp Opmode

    public void init(){
        robot.init(hardwareMap);
        runtime.reset();

        telemetry.addData("Say", "It's Droopy McCool Time!");
    }

    public void loop(){
        updateControls();

        if(LOOP_FIRST_RUN){
            runtime.reset();

            LOOP_FIRST_RUN = false;
        }

        if(robot.gp1_a == true || robot.gp2_a == true){
            AUTO_MODE_ACTIVE = false;
        }

        //manual_mode();

        if(runtime.milliseconds() > MODE_CHOICE_TIME || !AUTO_MODE_ACTIVE){ /* if the time is greater than the mode choice time or Autonomous mode = false, run an opmode, do nothing if not */
            if(AUTO_MODE_ACTIVE){ // if auto-op
                //AutoStep();
                telemetry.addData("Mode","Auto");
            }
            else { // if tele-op
                manual_mode();
               // telemetry.addData("Mode","Tele");
            }
        }




    }

    // Manual Semi-auto Flag Variables
    boolean clampRelease = true;
    boolean autoIntake = false;
    boolean autoBlockUp = false;
    boolean autoBlockDown = false;

    // Toggle first run variables
    boolean clampReleaseFirstRun = true;
    boolean autoIntakeFirstRun = true;
    boolean autoBlockUpFirstRun = true;
    boolean autoBlockDownFirstRun = true;


    // State machine global variables
    public enum State{
        IDLE,
        STATE_0,
        STATE_1,
        STATE_2,
        STATE_3,
        STATE_4,
        STATE_5,
        STATE_6,
        STATE_7,
    }

    State intakeState = State.STATE_0;
    private int[] inStepTimes = {1_500, 15_000, 1_500};// Fail safe progression times for each step of the Intake State Machine
    boolean inStateFirstRun = true;
    int inStateTargetTime = 0;

    State blockUpState = State.STATE_0;
    private int[] upStepTimes = {7_000, 1_500};// Fail safe progression times for each step of the Upward Transfer State Machine
    boolean upStateFirstRun = true;
    int upStateTargetTime = 0;

    State blockDownState = State.STATE_0;
    private int[] downStepTimes = {1_500, 7_000};// Fail safe progression times for each step of the Downward Transfer State Machine
    boolean downStateFirstRun = true;
    int downStateTargetTime = 0;
    double  intakeDropPower = 0.0; // being set later


    public void manual_mode(){
            double drivePowerY = robot.gp1_lstickY;
            double drivePowerX = robot.gp1_lstickX;
            double drivePowerR = robot.gp1_rstickX;
            double liftPowerL = robot.gp2_ltrigger;
            double liftPowerR = robot.gp2_rtrigger;
            double pullerPower = ((1 - robot.gp1_rtrigger) / 2) + .5; // left trigger is positive power, right trigger is negative power
            boolean armSwingIn = robot.gp2_lbumper;
            boolean armSwingOut = robot.gp2_rbumper;
            boolean spinIntakeOut = robot.gp1_lbumper;
            boolean spinIntakeIn = robot.gp1_rbumper;


            //Setters
            if(robot.gp1_a == true || robot.gp2_a == true){
                intakeDropPower = .6;
            }
            else if (robot.gp1_x){
                intakeDropPower = 0;
            }

            //Toggles
            if(robot.gp2_x && clampReleaseFirstRun){// toggling the clamp
                clampRelease = !clampRelease; // toggle
                clampReleaseFirstRun = false;
            }
            else if(!robot.gp2_x){
                clampReleaseFirstRun = true;
            }

            if(robot.gp2_up && autoBlockUpFirstRun){ // toggling the autonomous movement up
                autoBlockUp = !autoBlockUp; // toggle
                autoBlockUpFirstRun = false;
            }
            else if(!robot.gp2_up){
                autoBlockUpFirstRun = true;
            }

            if(robot.gp2_down && autoBlockDownFirstRun){ // toggling the autonomous movement down
                autoBlockDown = !autoBlockDown; // toggle
                autoBlockDownFirstRun = false;
            }
            else if(!robot.gp2_down){
                autoBlockDownFirstRun = true;
            }

            if(robot.gp2_y && autoIntakeFirstRun){ // toggling the autonomous intake
                autoIntake = !autoIntake;
                autoIntakeFirstRun = true;
            }
            else if(!robot.gp2_y){
                autoIntakeFirstRun = false;
            }


            // fail safes
            if(autoBlockDown){ // prevents the robot from trying to move autonomously up and down at the same time - override to down
                autoBlockUp = false;
            }
            if(autoBlockUp){ // if moving up, do not allow the autonomous intake to activate
                autoIntake = false;
            }


            //Semi-Auto
            if(autoIntake == true){
                switch (intakeState){ // autonomous intake state machine
                    case IDLE:
                        spinIntakeIn = false;
                        spinIntakeOut = false;
                        clampRelease = false;

                        autoIntake = false;
                        intakeState = State.STATE_0;
                        break;
                    case STATE_0:  // clamp open state
                        if(inStateFirstRun){
                            inStateTargetTime = (int) runtime.milliseconds() + inStepTimes[0]; // sets target fail safe time for this step

                            inStateFirstRun = false;
                        }

                        clampRelease = true;

                        if(excedesTime(inStateTargetTime)){ // continue conditions
                            intakeState = State.STATE_1;
                            inStateFirstRun = true;
                        }

                        break;
                    case STATE_1:  // spin up and inatke state
                        if(inStateFirstRun){
                            inStateTargetTime = (int) runtime.milliseconds() + inStepTimes[1]; // sets target fail safe time for this step

                            inStateFirstRun = false;
                        }
                        spinIntakeIn = true; // state actions
                        spinIntakeOut = false;
                        clampRelease = true; // maintain clamp open

                        if(!robot.touchBlock2.getState() == true || excedesTime(inStateTargetTime)){ // continue conditions
                            intakeState = State.STATE_2;
                            inStateFirstRun = true;
                        }
                        break;
                    case STATE_2:  // spin up and inatke state
                        if(inStateFirstRun){
                            inStateTargetTime = (int) runtime.milliseconds() + inStepTimes[2]; // sets target fail safe time for this step

                            inStateFirstRun = false;
                        }
                        spinIntakeIn = true; // state actions
                        spinIntakeOut = false;
                        clampRelease = false; // maintain clamp open

                        if(!robot.touchClamp7.getState() == true || excedesTime(inStateTargetTime)){ // continue conditions
                            intakeState = State.IDLE;
                            inStateFirstRun = true;
                        }
                        break;
                }
            }

            if(autoBlockUp){
                switch (blockUpState){ // autonomous movement up state machine
                    case IDLE:
                        armSwingIn = false; // stop arm
                        armSwingOut = false;

                        autoBlockUp = false;
                        blockUpState = State.STATE_0;
                        break;
                    case STATE_0:
                        if(upStateFirstRun){
                            upStateTargetTime = (int) runtime.milliseconds() + upStepTimes[0]; // sets target fail safe time for this step

                            upStateFirstRun = false;
                        }
                        liftPowerL = 1; // move lift up
                        liftPowerR = 0;

                        if(excedesTime(upStateTargetTime)){ // continue conditions (including failsafe times
                            blockUpState = State.STATE_1;
                            upStateFirstRun = true;
                        }
                        break;
                    case STATE_1:
                        if(upStateFirstRun){
                            upStateTargetTime = (int) runtime.milliseconds() + upStepTimes[1]; // sets target fail safe time for this step

                            upStateFirstRun = false;
                        }
                        liftPowerL = 0; // stop lift
                        liftPowerR = 0;

                        armSwingIn = false; // move arm out
                        armSwingOut = true;

                        if(excedesTime(upStateTargetTime)){ // continue conditions (including failsafe times
                            blockUpState = State.IDLE;
                            upStateFirstRun = true;
                        }
                        break;
                }
            }

        if(autoBlockDown){
            switch (blockDownState){ // autonomous movement down state machine
                case IDLE:
                    liftPowerL = 0; // stop lift
                    liftPowerR = 0;

                    autoBlockDown = false;
                    blockDownState = State.STATE_0;
                    break;
                case STATE_0:
                    if(downStateFirstRun){
                        downStateTargetTime = (int) runtime.milliseconds() + downStepTimes[0]; // sets target fail safe time for this step

                        downStateFirstRun = false;
                    }
                    armSwingIn = true; // swing arm in
                    armSwingOut = false;

                    if(excedesTime(downStateTargetTime) || !robot.touchArm1.getState() == true){ // continue conditions (including failsafe times)
                        blockDownState = State.STATE_1;
                        downStateFirstRun = true;
                    }
                    break;
                case STATE_1:
                    if(downStateFirstRun){
                        downStateTargetTime = (int) runtime.milliseconds() + downStepTimes[1]; // sets target fail safe time for this step

                        downStateFirstRun = false;
                    }
                    armSwingIn = false; // stop arm
                    armSwingOut = false;

                    liftPowerL = 0; // move lift down
                    liftPowerR = 1;


                    if(excedesTime(downStateTargetTime) || !robot.touchLift0.getState() == true){ // continue conditions (including failsafe times
                        blockDownState = State.IDLE;
                        downStateFirstRun = true;
                    }
                    break;
            }
        }

        if(!robot.touchBlock2.getState()){
            touchBlockCount++;
        }
        //telemetry.addData("GP1_LTrigger:", robot.gp1_ltrigger);
        telemetry.addData("Touch Lift:", !robot.touchLift0.getState());
        telemetry.addData("Touch Clamp:", !robot.touchClamp7.getState());
        telemetry.addData("Touch Block:", touchBlockCount);
        telemetry.addData("Touch Arm:", !robot.touchArm1.getState());
        telemetry.update();



        //more failsafes
        if(!robot.touchLift0.getState() == true){
            liftPowerL = 0;
        }





        meccanum.drive_Controller(-drivePowerY, drivePowerX, -drivePowerR);
        flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
        lift.move_Controller(liftPowerR , liftPowerL);
        arm_swing.set_arm_position(armSwingIn, armSwingOut);
        arm_swing.set_clamp_position(clampRelease);
        pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);
        intakeDrop.set_ServoPower(intakeDropPower, robot.intakeDropL, robot.intakeDropR);



    }

    int touchBlockCount = 0;

    // State machine code


    public State masterState = State.STATE_0;
    public State driveState = State.STATE_0;


    public void AutoStep() {



     /*//   switch(CURR_STATE){


          //  case STATE_INIT:
              // call subroutine to move lift to initial position
              // may want to check to see if all your sensors are ok also
              // get position on field
              // get first waypoint from navigation
              // check if Robot is stopped if not stop it




              if (AUTO_MODE_ACTIVE == true) { // Include other conditions in this statement
                  CURR_STATE = State.STATE_START;

              }

            break;

            case STATE_START:
                // Check if robot is ready and init passed then move to next state
                // Else you may want to write an error to log and telemetry
                // You may also want to write a message to telemetry that the sequence has started
                CURR_STATE = State.STATE_MOVE_POS;

            break;

            case STATE_MOVE_POS:
                // Call drive routine and pass down first waypoint
                // Drive routine can run until the waypoint is reached
                // Once the waypoint is reached you can keep iterating through this step until
                // drive sequence is complete

                if ((BLOCK_FOUND == true) && (ROBOT_INTAKE_POS)){
                    CURR_STATE = State.STATE_INTAKE_POS;
                }

                if (DRIVE_SEQ_COMPLETE == true){
                    CURR_STATE = State.STATE_SEQ_COMPLETE;

                }

                if ((BLOCK_CAPTURED == true) && (ROBOT_PLACE_POS == true)){
                    CURR_STATE = State.STATE_PLACE_BLOCK;


                }


            break;

            case STATE_INTAKE_POS:
               // Call intake routine
               // Confirm block was caputured
               // I would return back to the move state and read flags


            break;

            case STATE_PLACE_BLOCK:
             // I would call an auto sub routine to move the lift into position based ont the needed level
             //
             // place the block and return to the lowered position.
             // I would then return back to move state and read flags

             break;

            case STATE_SEQ_COMPLETE:
            // I would use this to return to a neutral waypoint and reset the sequence to start
            // read the # of runs and reset navigation

            CURR_RUN_TOT++;


            if (CURR_RUN_TOT == AUTO_RUNS_TOT){
                CURR_STATE = State.STATE_STOP;

            }
            if (CURR_RUN_TOT != AUTO_RUNS_TOT) {
                CURR_STATE = State.STATE_START;

            }

            break;

            case STATE_STOP:
             // This should be an all stop
             // I would include this to make sure no more action after the time expires for autoOp.


            break;
        }


*/


    }


    // Utility Functions
    public void Set_AutoMode(boolean mode){
        AUTO_MODE_ACTIVE = mode;


    }

    public void updateControls(){
        robot.gp1_lstickX =  gamepad1.left_stick_x;
        robot.gp1_lstickY =  gamepad1.left_stick_y;
        robot.gp1_rstickX =  gamepad1.right_stick_x;
        robot.gp1_rbumper = gamepad1.right_bumper;
        robot.gp1_lbumper = gamepad1.left_bumper;
        robot.gp1_a = gamepad1.a;
        robot.gp1_b = gamepad1.b;
        robot.gp2_a = gamepad2.a;
        robot.gp1_rtrigger = gamepad1.right_trigger;
        robot.gp1_ltrigger = gamepad1.left_trigger;
        robot.gp2_rtrigger = gamepad2.right_trigger;
        robot.gp2_ltrigger = gamepad2.left_trigger;
        robot.gp2_lbumper = gamepad2.left_bumper;
        robot.gp2_rbumper = gamepad2.right_bumper;
        robot.gp2_x = gamepad2.x;
        robot.gp2_y = gamepad2.y;
        robot.gp1_x = gamepad1.x;
        robot.gp2_up = gamepad2.dpad_up;
        robot.gp2_down = gamepad2.dpad_down;

        // Add other buttons

    }

    boolean excedesTime(int inTargetTime){  // Takes in a time a returns true if the current time is greater than the input time, if not, it ouptputs false
        boolean output = false;
        if(runtime.milliseconds() > inTargetTime){
            output = true;
        }
        return (output);
    }
    //public void runOpMode() { }
}

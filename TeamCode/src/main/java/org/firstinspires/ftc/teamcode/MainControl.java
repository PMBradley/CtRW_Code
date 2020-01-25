package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;


@TeleOp(name = "MainControl")
public class MainControl extends OpMode {


    // Create the instances of each class for the robot
    private Robot2019 robot = new Robot2019(); // Main robot data class
    private ElapsedTime runtime  = new ElapsedTime(); // internal clock
    Navigation2019 navigation = new Navigation2019(robot);

    Arm_Swing arm_swing = new Arm_Swing(robot); // Hardware classes
    Drive_Meccanum   meccanum = new Drive_Meccanum(robot);
    Flywheel_DuoMirrored flyIntake = new Flywheel_DuoMirrored(robot);
    Vision vision = new Vision(robot);
    Lift_Linear lift = new Lift_Linear(robot);
    DropServo_DuoMirrored intakeDrop = new DropServo_DuoMirrored(robot, robot.intakeDropL, robot.intakeDropR);
    DropServo_DuoMirrored pullerDrop = new DropServo_DuoMirrored(robot, robot.pullerDropL, robot.pullerDropR);

    // Global Variables
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

    public void init(){ // initialization function
        robot.init(hardwareMap);
        runtime.reset();

        telemetry.addData("Say", "It's Droopy McCool Time!");

        
        /*
        Add vision system init stuff here and active



         */
    telemetry.addData("Test point:", "1");
    telemetry.update();
        vision.initVuforia();
        vision.activateTracking();
    }

    public void loop(){ // main loop
        updateControls(); // update the controllers and check the sensors
        checkSensors(); // (checking sensors happens multiple times in the loop to avoid missing an input)
     

        if(LOOP_FIRST_RUN){ // if it is the first run, ensure runtime is correct
            runtime.reset();

            LOOP_FIRST_RUN = false;
        }

        if(robot.gp1_a == true || robot.gp2_a == true){ // if either controller presses A, switch to tele-op mode
            AUTO_MODE_ACTIVE = false;
        }

        checkSensors();

        if(runtime.milliseconds() > MODE_CHOICE_TIME || !AUTO_MODE_ACTIVE){ /* if the time is greater than the mode choice time or Autonomous mode = false, run an opmode, do nothing if not */
            if(AUTO_MODE_ACTIVE){ // if auto-op
               // zomAuto(); // a hacked together autonomous

                telemetry.addData("Mode","Auto");
                AutoStep();

            }
            else { // if tele-op
                manual_mode();
            }
        }

        checkSensors();
    }

    // Manual Semi-auto Flag Variables
    boolean clampRelease = true;
    boolean intake = false;
    boolean autoIntake = false;
    boolean autoBlockUp = false;
    boolean autoBlockDown = false;

    // Toggle first run variables
    boolean clampReleaseFirstRun = true;
    boolean intakeFirstRun = true;
    boolean autoBlockUpFirstRun = true;
    boolean autoBlockDownFirstRun = true;


    // State machine global variables
    public enum State{
        COMPLETE,
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
    private int[] inStepTimes = {0_000, 15_000, 1_000};// Fail safe progression times for each step of the Intake State Machine
    boolean inStateFirstRun = true;
    int inStateTargetTime = 0;

    State blockUpState = State.STATE_0;
    private int[] upStepTimes = {6_400, 1_000};// Fail safe progression times for each step of the Upward Transfer State Machine
    boolean upStateFirstRun = true;
    int upStateTargetTime = 0;

    State blockDownState = State.STATE_0;
    private int[] downStepTimes = {1_000, 8_000};// Fail safe progression times for each step of the Downward Transfer State Machine
    boolean downStateFirstRun = true;
    int downStateTargetTime = 0;
    double  intakeDropPower = 0.0; // being set later


    double SPEED_REDUCE_VALUE = .6; // power values are multiplied by this value if being reduced
    double SPEED_REDUCE_THRESHOLD = .85;

    public void manual_mode(){
        double drivePowerY = robot.gp1_lstickY; // set all values to their corresponding controller values
        double drivePowerX = robot.gp1_lstickX;
        double drivePowerR = robot.gp1_rstickX;
        double liftPowerL = robot.gp2_ltrigger;
        double liftPowerR = robot.gp2_rtrigger;
        double pullerPower = ((1 - robot.gp1_rtrigger) / 2) + .5; // left trigger is positive power, right trigger is negative power
        double Ltrigger = robot.gp1_ltrigger;
        boolean armSwingIn = robot.gp2_lbumper;
        boolean armSwingOut = robot.gp2_rbumper;
        boolean spinIntakeOut = robot.gp1_lbumper;
        boolean spinIntakeIn = robot.gp1_rbumper;



        checkSensors();
        //Setters
        if(robot.gp1_a == true || robot.gp2_a == true){ // if a is being pressed, drop intakes
            //intakeDropPower = -1;
            robot.intakeDropL.setPosition(1.0);
            robot.intakeDropR.setPosition(0.0);
        }
        else { // else don't
          //  intakeDropPower = 0;
        }


        if(Math.abs(robot.gp1_lstickY) < SPEED_REDUCE_THRESHOLD){ // reduce speed unless the stick is all the way forward
            drivePowerY *= SPEED_REDUCE_VALUE;
        }
        if(Math.abs(robot.gp1_lstickX) < SPEED_REDUCE_THRESHOLD){
            drivePowerX *= SPEED_REDUCE_VALUE;
        }
        if(Math.abs(robot.gp1_rstickX) < SPEED_REDUCE_THRESHOLD){
            drivePowerR *= SPEED_REDUCE_VALUE;
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

        if(robot.gp1_y && intakeFirstRun){ // toggling the autonomous intake
         //   intake = !intake;
            intakeFirstRun = false;
        }
        else if(!robot.gp1_y){
            intakeFirstRun = true;
        }

        if(intake){
            spinIntakeIn = true;
        }

        checkSensors();

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
                    spinIntakeIn = false;
                    spinIntakeOut = false;

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

                    if(isTouchBlock() == true || excedesTime(inStateTargetTime)){ // continue conditions
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
                    clampRelease = false; // close clamp

                    if(!robot.touchClamp7.getState() == true || excedesTime(inStateTargetTime)){ // continue conditions
                        intakeState = State.IDLE;
                        inStateFirstRun = true;
                    }
                    break;
            }
        }
        else {
            intakeState = State.STATE_0; // reset state on disable
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
                    liftPowerL = 0; // move lift up
                    liftPowerR = 1;

                    armSwingIn = false; // stop arm movement
                    armSwingOut = false;

                    if(excedesTime(upStateTargetTime) || !robot.touchLiftUp3.getState()){ // continue conditions (including failsafe times)
                        blockUpState = State.STATE_1;
                        upStateFirstRun = true;
                    }
                    break;
                case STATE_1:
                    if(upStateFirstRun) {
                        upStateTargetTime = (int) runtime.milliseconds() + upStepTimes[1]; // sets target fail safe time for this step

                        upStateFirstRun = false;
                    }
                    liftPowerL = 0; // prevent movement down while the arm is swining

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

                    liftPowerL = 1; // move lift down
                    liftPowerR = 0;


                    if(excedesTime(downStateTargetTime) || !robot.touchLift0.getState() == true){ // continue conditions (including failsafe times
                        blockDownState = State.IDLE;
                        downStateFirstRun = true;
                    }
                    break;
            }
        }

        checkSensors();



        //navigation.updateLocation();
        // Telemetry
        //telemetry.addData("GP1_LTrigger:", robot.gp1_ltrigger);
        //  telemetry.addData("Touch Lift:", !robot.touchLift0.getState());
        //  telemetry.addData("Touch Clamp:", !robot.touchClamp7.getState());
        //  telemetry.addData("Touch Block:", touchBlockCount);
        //  telemetry.addData("Touch Arm:", !robot.touchArm1.getState());
        //telemetry.addData("Auto Intake:", autoIntake);


      //  telemetry.addData("Lidar F:", robot.readFlight(robot.flightFront0));
     //   telemetry.addData("Lidar R:", robot.readFlight(robot.flightRight2));
     //   telemetry.addData("Lidar B:", robot.readFlight(robot.flightBack3));
     //   telemetry.addData("Lidar L:", robot.readFlight(robot.flightLeft1));
     //   telemetry.addData("X Pos:", navigation.X);
      //  telemetry.addData("Y Pos:", navigation.Y);
        telemetry.addData("Heading:", navigation.getRotation());
        if(Ltrigger < .5) {
            telemetry.addData("Boost OFF", Ltrigger);
        }
        else {
            telemetry.addData("Boost ON!", Ltrigger);
        }

        telemetry.addData("Power fl", robot.fl);
        telemetry.addData("Power fr", robot.fr);
        telemetry.addData("Power bl", robot.bl);
        telemetry.addData("Power br", robot.br);

      //  telemetry.addData("Lstick X:", drivePowerX);
       // telemetry.addData("Lstick Y:", drivePowerY);
       // telemetry.addData("Lstick R:", drivePowerR);

        telemetry.update();

        //more failsafes
        if(!robot.touchLift0.getState() == true){
            liftPowerL = 0;
        }
        if(!robot.touchLiftUp3.getState() == true){
            liftPowerR = 0;
        }

        //meccanum.drive_Controller(-drivePowerY, drivePowerX, -drivePowerR);
        meccanum.Drive_Vector(-drivePowerX, drivePowerY, -drivePowerR, navigation.getRotation(), true, Ltrigger);
       // meccanum.Drive_Polar(drivePowerX, drivePowerY, drivePowerR, navigation.getRotation(), Ltrigger, true);
        flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
        lift.move_Controller(liftPowerR , liftPowerL);
        arm_swing.set_arm_position(armSwingIn, armSwingOut);
        arm_swing.set_clamp_position(clampRelease);
        pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);
        //intakeDrop.set_ServoPower(intakeDropPower, robot.intakeDropL, robot.intakeDropR);


        lastTouchBlockCount = touchBlockCount;
    }



    // Zombie Auto
    private boolean zomFirstRun = true;
    private double[] zomStateTimes = {1_000};
    private int zomStateTargetTime = 0;
    private double[][] zomCoords = {{0, 0.5, 0}};

    public void zomAuto(){
        double drivePowerX = 0; // set all values to their corresponding controller values
        double drivePowerY = 0;
        double drivePowerR = 0;

        if(zomFirstRun){
            zomStateTargetTime = (int)(runtime.milliseconds() + zomStateTimes[0]);
            zomFirstRun = false;
        }

        if(!excedesTime(zomStateTargetTime)){
            drivePowerX = zomCoords[0][0];
            drivePowerY = zomCoords[0][1];
            drivePowerR = zomCoords[0][2];
        }

        meccanum.Drive_Vector(drivePowerX, drivePowerY, drivePowerR, navigation.getRotation(), false);
    }


    // Autonomous Code
    public State autoState = State.IDLE; // state machine flag - holds which step the state machine is on
    public int stateInc = 0; // keeps an index of which state we are at (only starts counting when at state 0)
    public int quadrant = 0;

    private double[][][] driveCoords = {
            { // quadrant 0 coordinates
                    {0.0, 0.0, 180.0},
                    {0.0, 10.0, 0.0},
                    {0.0, 0.0, 90.0},
                    {0.0, 0.0, 90.0},
                    {0.0, 0.0, 90.0},
            },

            { // quadrant 1 coordinates
                    {30.0, 30.0, 0.0},
                    {30.0, 30.0, 90.0},
                    {30.0, 30.0, 180.0},
                    {30.0, 30.0, 270.0},
                    {30.0, 30.0, 0.0},
            },
    }; // Holds the coordinate points to be used with our navigation - first state holds the quadrant differences - second dimension holds the state - the thrid dimension holds x, y, and r (in that order)
    private double[][] pictureOrder = {

    };
    private int[] autoStepTimes = {0_000, 0_000, 1_000, 1_000, 1_000}; // fail safe times for each step in the autonomous program - in milisecs
    private int autoStartTime = 0;
    private int autoStateTargetTime = 0;
    private boolean autoStateFirstRun = true;

    private double transApproachReduce = 15;
    private double rotApproachReduce = 120;

    public void AutoStep() {
        double liftPowerL = 0.0; // power variables for the manipulators
        double liftPowerR = 0.0; // later set by the state machine to do things
        double pullerPower = 0.0;
        boolean armSwingIn = false;
        boolean armSwingOut = false;
        boolean spinIntakeOut = false;
        boolean spinIntakeIn = false;
        boolean pictureRelative = false;

        double relativeRotation = navigation.getRotation();

        double[] moveCoords = {0.0, 0.0, 0.0}; // array that holds the target xyr coordinate position for the robot - later set to one of the drive coordinates
        double[] movePowers = {0.0, 0.0, 0.0}; // array that holds the actual powers passed to the drive function - later set in the state machine

        navigation.updateRotation();


        switch (autoState){ // main state machine - the state determines the robot's actions - mostly movement, but with some extra manipulator action
            case IDLE:
                autoStartTime = (int) runtime.milliseconds();

             //   quadrant = navigation.getCurrentQuadrant(); // set which quadrant we are starting in

                stateInc = 0;
          //      moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards the first (starting coordinate)

                autoState = State.STATE_0;
                break;
            case STATE_0:
                stateInc = 0;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[stateInc]; // sets target fail safe time for this step
                    autoStateFirstRun = false;
                }

                moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position



                // continue conditions
                if(pictureRelative){ // go to relative position
                    if(navigation.atCoord(moveCoords[0], moveCoords[1], moveCoords[2]) || excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.COMPLETE;
                        autoStateFirstRun = true;
                    }
                }
                else { // if not relative, drive by time (and rotation
                    if(navigation.atRot(moveCoords[2]) && excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.COMPLETE;
                        autoStateFirstRun = true;
                    }
                }

                break;
            case STATE_1:
                stateInc = 1;
                pictureRelative = true;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position



                // continue conditions
                if(pictureRelative){ // go to relative position
                    if(navigation.atCoord(moveCoords[0], moveCoords[1], moveCoords[2]) || excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_2;
                        autoStateFirstRun = true;
                    }
                }
                else { // if not relative, drive by time (and rotation
                    if(navigation.atRot(moveCoords[2]) && excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_2;
                        autoStateFirstRun = true;
                    }
                }
                break;
            case STATE_2:
                stateInc = 2;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position



                // continue conditions
                if(pictureRelative){ // go to relative position
                    if(navigation.atCoord(moveCoords[0], moveCoords[1], moveCoords[2]) || excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_3;
                        autoStateFirstRun = true;
                    }
                }
                else { // if not relative, drive by time (and rotation
                    if(navigation.atRot(moveCoords[2]) && excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_3;
                        autoStateFirstRun = true;
                    }
                }
                break;
            case STATE_3:
                stateInc = 3;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position


                // continue conditions
                if(pictureRelative){ // go to relative pos
                    if(navigation.atCoord(moveCoords[0], moveCoords[1], moveCoords[2]) || excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_4;
                        autoStateFirstRun = true;
                    }
                }
                else { // if not relative, drive by time (and rotation
                    if(navigation.atRot(moveCoords[2]) && excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.STATE_4;
                        autoStateFirstRun = true;
                    }
                }
                break;
            case STATE_4:
                stateInc = 4;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position


                if(pictureRelative){ // go to relative position
                    if(navigation.atCoord(moveCoords[0], moveCoords[1], moveCoords[2]) || excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.COMPLETE;
                        autoStateFirstRun = true;
                    }
                }
                else { // if not relative, drive by time (and rotation
                    if(navigation.atRot(moveCoords[2]) && excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                        autoState = State.COMPLETE;
                        autoStateFirstRun = true;
                    }
                }
                break;
            case COMPLETE:
                moveCoords = driveCoords[quadrant][stateInc]; // keep going to the last known position

              //  if(excedesTime(autoStartTime + 36_000)){ // automatically disable the autonomous mode after there has been enough time for tele-op to start
                 //   AUTO_MODE_ACTIVE = false;
              //  }

                break;
        }

        // Set Translational power values
        if(pictureRelative){ // if driving relative to a picture
            movePowers[0] = (moveCoords[0] - navigation.X) / transApproachReduce; // if the target X move position is less than current X position, move that direction and visa versa
            movePowers[1] = (moveCoords[1] - navigation.Y) / transApproachReduce; // if the target Y move position is less than current Y position, move that direction and visa versa
        }

        // Set Rotational Power Values
        double[] checkRotations = {moveCoords[2], moveCoords[2]};
        if(navigation.ROTATION_DEG - 180 < 0){ // account the range for the fact that it might fall close to the cut off point on the 360 degree range
            checkRotations[0] -= 360; // create a ghost layer underneath the regular target value
        }
        else {
            checkRotations[1] += 360; // create a ghost layer above the regular target value
        }
        if(Math.abs(checkRotations[1] - navigation.ROTATION_DEG) > Math.abs(checkRotations[0] - navigation.ROTATION_DEG)){ // if one is closer
          //  movePowers[2] = (checkRotations[1] - navigation.ROTATION_DEG) / rotApproachReduce; // move towards that one
        }
        else { // else
           // movePowers[2] = (checkRotations[0] - navigation.ROTATION_DEG) / rotApproachReduce; // move away from that one
        }

        if(navigation.getRotation() < moveCoords[2]){
            movePowers[2] = (moveCoords[2] - navigation.getRotation()) / rotApproachReduce;
        }
        else if (navigation.getRotation() > moveCoords[2]){
            movePowers[2] = (moveCoords[2] - navigation.getRotation()) / rotApproachReduce;
        }

        telemetry.addData("Heading:", navigation.getRotation());
        telemetry.addData("Check pos 0:", checkRotations[0]);
        telemetry.addData("Check pos 1:", checkRotations[1]);
        telemetry.addData("State: ", autoState);

        telemetry.update();


        meccanum.Drive_Vector(movePowers[0], movePowers[1], movePowers[2], relativeRotation, false);
        flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
        lift.move_Controller(liftPowerR , liftPowerL);
        arm_swing.set_arm_position(armSwingIn, armSwingOut);
        arm_swing.set_clamp_position(clampRelease);
        pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);
        intakeDrop.set_ServoPower(intakeDropPower, robot.intakeDropL, robot.intakeDropR);
    }


    // Utility Functions
    public void Set_AutoMode(boolean mode){
        AUTO_MODE_ACTIVE = mode;


    }

    int touchBlockCount = 0;
    int lastTouchBlockCount = 0;

    public boolean isTouchBlock(){
        boolean output = false;
        if(touchBlockCount != lastTouchBlockCount){
            output = true;
        }
        return output;
    }

    public void checkSensors(){
        if(!robot.touchBlock2.getState()){
            touchBlockCount++;
        }
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
        robot.gp1_y = gamepad2.y;
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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@TeleOp(name = "MainControlBlue")

public class MainControlBlue extends OpMode {


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
        //vision.initVuforia();
        //vision.activateTracking();
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
                // testAuto(); // a hacked together autonomous
                AutoStep();
                //telemetry.addData("Mode","Auto");
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
        STATE_8,
        STATE_9,
        STATE_10,
        STATE_11,
        STATE_12,
        STATE_13,
        STATE_14,
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
        String targetInfo = "NULL";

        //  targetInfo = vision.targetsAreVisible();
        telemetry.addData("Target Info:", targetInfo);


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

        telemetry.addData("Puler power:", pullerPower);
        //    telemetry.addData("Power fl", robot.fl);
        //    telemetry.addData("Power fr", robot.fr);
        //    telemetry.addData("Power bl", robot.bl);
        //    telemetry.addData("Power br", robot.br);

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



    // Test Auto
    //private State testState = State.STATE_0;
    private int testStep = 0;
    private boolean testMainFirstRun = true;
    private boolean testStateFirstRun = true;
    private int testMainStateTime = 25_000;
    private int testMainStateTargetTime = 0;
    private int testStateTargetTime = 0;
    private double[][] testCoords = {
            {
                    0, .5, 0, // X, Y, R Power
                    0,     // Rotation in degrees
                    200   //Time
            },
            {
                    .5, 0, -.5,
                    270,
                    1_000
            }
    };

    public void testAuto(){
        double drivePowerX = 0; // set all values to their corresponding controller values
        double drivePowerY = 0;
        double drivePowerR = 0;

        if(testMainFirstRun){ // first run time limiter
            testMainStateTargetTime = (int)(runtime.milliseconds() + testMainStateTime);
            testMainFirstRun = false;
        }

        if(!excedesTime(testMainStateTime)) // auto fail safe
        {
            // NOTE: this was made because i (Mark) didn't know how to use the state machine so i made a stepper
            switch (testStep) // process stepper
            {
                case 0: //step 0
                {
                    if(testStateFirstRun){ //step timer
                        testStateTargetTime = (int)(runtime.milliseconds() + testCoords[testStep][4]);
                        testStateFirstRun = false;
                    }
                    if(!excedesTime(testMainStateTargetTime)) //main code
                    {
                        drivePowerX = testCoords[testStep][0];
                        drivePowerY = testCoords[testStep][1];
                        drivePowerR = testCoords[testStep][2];
                    }
                    else //change to next step and reset step timer
                    {
                        testStep++;
                        testStateFirstRun = true;
                    }
                }
                case 1: //step 1
                {
                    if(testStateFirstRun){ //step time limit
                        testStateTargetTime = (int)(runtime.milliseconds() + testCoords[testStep][4]);
                        testStateFirstRun = false;
                    }
                    if(!excedesTime(testMainStateTargetTime)) //main code
                    {
                        drivePowerX = testCoords[testStep][0];
                        drivePowerY = testCoords[testStep][1];
                        //drivePowerR = meccanum.gyroTurn(testCoords[testStep][3], navigation.getRotation(), testCoords[testStep][2]); //gyro turn
                    }
                    else //change to next step and reset step timer
                    {
                        testStep++;
                        testStateFirstRun = true;
                    }
                }
            }
        }

        telemetry.addData("State:", testStep);
        telemetry.addData("Heading:", navigation.getRotation());
        telemetry.addData("Raw Heading:", navigation.getRawRotation());

        telemetry.update();

        meccanum.Drive_Vector(drivePowerX, drivePowerY, drivePowerR, navigation.getRotation());
    }


    // Autonomous Code
    public State autoState = State.IDLE; // state machine flag - holds which step the state machine is on
    public int stateInc = 0; // keeps an index of which state we are at (only starts counting when at state 0)
    public int quadrant = 0;

    private double[][][] driveCoords = {
            { // quadrant 0 coordinates (red mat side)
                    {0, 0.3, 0, 0}, // move forward to mat
                    {0, 0, 0.4, 90}, // rotate to line up with mat
                    {-0.3, 0, 0, 0}, // move right to line up with mat
                    {0.0, .3, 0, 0}, // move against mat
                    {0.0, 0.0, 0.0, 0}, // drop pullers
                    {0.0, -0.5, 0.0, 0}, // move backwards with mat
                    {0.3, 0.0, 0.0, 0}, // left away from mat
                    {0.0, 0.3, 0.0, 0}, // readjust against wall
                    {-0.4, 0.0, 0.0, 0}, // push mat into place (make sure it is in pos)
                    {0.3, 0, 0, 0}, // move left under bridge
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
            },

            { // quadrant 1 coordinates (blue mat side)
                    {0, 0.3, 0, 0}, // move forward to mat
                    {0, 0, 0.4, 90}, // rotate to line up with mat
                    {0.3, 0, 0, 0}, // move left to line up with mat
                    {0.0, .3, 0, 0}, // move against mat
                    {0.0, 0.0, 0.0, 0}, // drop pullers
                    {0.0, -0.5, 0.0, 0}, // move backwards with mat
                    {-0.3, 0.0, 0.0, 0}, // right away from mat
                    {0.0, 0.3, 0.0, 0}, // readjust against wall
                    {0.4, 0.0, 0.0, 0}, // push mat into place (make sure it is in pos)
                    {-0.3, 0, 0, 0}, // move right under bridge
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
            },
    }; // Holds the coordinate points to be used with our navigation - first state holds the quadrant differences - second dimension holds the state - the thrid dimension holds x, y, and r (in that order)
    private double[][] pictureOrder = {

    };
    private int[][] autoStepTimes = {
//            0     1     2   3    4     5     6    7    8    9   10    11
            {880, 5_000, 700, 900, 400, 1_525, 1_300, 2_100, 400, 1_000, 0, 0, 0}, // quadrant 0 times (red side mat)
            {780, 5_000, 400, 600, 400, 1_550, 1_300, 2_500, 400, 1_000, 0, 0, 0}, // quadrant 1 times (blue side mat)
            {170, 5_000, 110, 80, 100, 6_000, 700, 5_000, 100, 100, 200, 220, 0}, // quadrant 2 times (red side block)
            {170, 5_000, 110, 80, 100, 6_000, 700, 5_000, 100, 100, 200, 220, 0}, // quadrant 3 times (blue side block)

    }; // fail safe times for each step in the autonomous program - in milisecs

    private int autoStartTime = 0;
    private int autoStateTargetTime = 0;
    private boolean autoStateFirstRun = true;

    private double transApproachReduce = 15;
    private double rotApproachReduce = 10;

    public void AutoStep() {
        double liftPowerL = 0.0; // power variables for the manipulators
        double liftPowerR = 0.0; // later set by the state machine to do things
        double pullerPower = 1; // defualt to down (.5 is up)
        boolean armSwingIn = false;
        boolean armSwingOut = false;
        boolean spinIntakeOut = false;
        boolean spinIntakeIn = false;
        boolean turnComp = false;

        double[] moveCoords = {30.0, 30.0, 0.0}; // array that holds the target xyr coordinate position for the robot - later set to one of the drive coordinates
        double[] movePowers = {0.0, 0.0, 0.0}; // array that holds the actual powers passed to the drive function - later set in the state machine

        navigation.updateRotation();


        //State Machine

        switch (autoState){ // main state machine - the state determines the robot's actions - mostly movement, but with some extra manipulator action
            case IDLE:
                autoStartTime = (int) runtime.milliseconds();

                quadrant = 1; // set which quadrant we are starting in

                stateInc = 0;

                pullerPower = 0.5;
                //     moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards the first (starting coordinate)

                autoState = State.STATE_0;
                break;
            case STATE_0:
                stateInc = 0;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];
                //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position


                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_1;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_1:
                stateInc = 1;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                turnComp = meccanum.gyroTurn(driveCoords[quadrant][stateInc][3], navigation.getRawRotation(), 10);

                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                if(!turnComp)
                {
                    movePowers[2] = driveCoords[quadrant][stateInc][2];
                }
                //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime) || turnComp){ // continue conditions (including failsafe times)
                    autoState = State.STATE_2;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_2:
                stateInc = 2;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];
                //   moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                pullerPower = 0.5;


                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_3;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_3:
                stateInc = 3;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_4;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_4:
                stateInc = 4;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];


                // Have a whole state to drop the servo

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_5;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_5:
                stateInc = 5;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];



                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_6;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_6:
                stateInc = 6;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];


                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_7;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_7:
                stateInc = 7;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_8;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_8:
                stateInc = 8;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_9;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_9:
                stateInc = 9;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_10;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_10:
                stateInc = 10;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;
                robot.intakeDropL.setPosition(0.0); //
                robot.intakeDropR.setPosition(1.0);

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.STATE_11;
                    autoStateFirstRun = true;
                }
                break;
            case STATE_11:
                stateInc = 11;

                if(autoStateFirstRun){
                    autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                    autoStateFirstRun = false;
                }
                movePowers[0] = driveCoords[quadrant][stateInc][0];
                movePowers[1] = driveCoords[quadrant][stateInc][1];
                movePowers[2] = driveCoords[quadrant][stateInc][2];

                pullerPower = 0.5;
                robot.intakeDropL.setPosition(0.0); //
                robot.intakeDropR.setPosition(1.0);

                if(excedesTime(autoStateTargetTime)){ // continue conditions (including failsafe times)
                    autoState = State.COMPLETE;
                    autoStateFirstRun = true;
                }
                break;
            case COMPLETE:
                //moveCoords = driveCoords[quadrant][stateInc]; // keep going to the last known position

                //   if(excedesTime(autoStartTime + 38_000)){ // automatically disable the autonomous mode after there has been enough time for tele-op to start
                //AUTO_MODE_ACTIVE = false;
                // }
                robot.intakeDropL.setPosition(0.0); //
                robot.intakeDropR.setPosition(1.0);

                break;
        }

        //Patrick's home grown turn code

        // Set Translational power values
        /*
        movePowers[0] = (moveCoords[0] - navigation.X) / transApproachReduce; // if the target X move position is less than current X position, move that direction and visa versa
        movePowers[1] = (moveCoords[1] - navigation.Y) / transApproachReduce; // if the target Y move position is less than current Y position, move that direction and visa versa

        // Set Rotational Power Values
         */

        if(quadrant == 0 || quadrant == 1){
            robot.intakeDropL.setPosition(0.0); //
            robot.intakeDropR.setPosition(1.0);
        }


        meccanum.Drive_Vector(movePowers[0], movePowers[1], movePowers[2], navigation.getRotation());
        pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);


        telemetry.addData("State:", autoState);
        telemetry.update();

        //meccanum.Drive_Vector(movePowers[0], movePowers[1], movePowers[2], navigation.getRotation());
        //   flyIntake.set_Power(spinIntakeIn, spinIntakeOut);
        //  lift.move_Controller(liftPowerR , liftPowerL);
        //  arm_swing.set_arm_position(armSwingIn, armSwingOut);
        //  arm_swing.set_clamp_position(clampRelease);
        //  pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);
        //  intakeDrop.set_ServoPower(intakeDropPower, robot.intakeDropL, robot.intakeDropR);
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
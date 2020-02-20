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

    public int MODE_CHOICE_TIME = 2000; // time it waits for you to press the button for tele-op before going to autonomous

    // Startup flags
    public boolean LOOP_FIRST_RUN = true; // used to indicate if it is the first run of the main loop
    public boolean ROBOT_DEAD_STOP = false;
    public boolean SENSORS_READY = false;
    public boolean DRIVE_READY = false;
    public boolean INIT_COMPLETE = false;
    public boolean INIT_FAIL = false;
    public boolean AUTO_MODE_ACTIVE = true; // assumes autonomous until given an input
    public boolean DYNAMIC_PARK_ACTIVE = false; // used to start the dynamic parking state machine

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
    // telemetry.addData("Test point:", "1");
    telemetry.update();
     //   vision.initVuforia();/
       // vision.activateTracking();
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
                testAuto(); // the testing auto
             //   AutoStep(); // regular auto function

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
    private int[] upStepTimes = {4_400, 1_000};// Fail safe progression times for each step of the Upward Transfer State Machine
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
        double pullerPower = ((robot.gp1_rtrigger) / 2) + .5; // right trigger
        double Ltrigger = robot.gp1_ltrigger;
        boolean armSwingIn = robot.gp2_lbumper;
        boolean armSwingOut = robot.gp2_rbumper;
        boolean spinIntakeOut = robot.gp1_lbumper;
        boolean spinIntakeIn = robot.gp1_rbumper;
        String targetInfo = "NULL";

        double relativeHeading =  navigation.getRawRotation();

        targetInfo = vision.targetsAreVisible();
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
        if(robot.gp1_b == true){ // moves the robot slowly while we have the mat
            drivePowerX = -0.4; // set movement left power

            pullerPower = 0.5; // set pullers to be down

            relativeHeading = 0; // set robot to drive relative to field
        }

        if(robot.gp1_x == true){ // gyro reset
            robot.init_imu();
        }

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
            intake = !intake;
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

        meccanum.Drive_Gyro_Vector(drivePowerX, drivePowerY, drivePowerR, relativeHeading, true, Ltrigger);
      //  meccanum.Drive_Vector(-drivePowerX, drivePowerY, -drivePowerR, relativeHeading, true, Ltrigger);

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
             // quadrant 1 coordinates (red mat side)
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step

    };

    public void testAuto(){
        double drivePowerX = 0; // set all values to their corresponding controller values
        double drivePowerY = 0;
        double drivePowerR = 0;
        double targetHeading = 0;


        if(!excedesTime(testMainStateTime)) // auto fail safe
        {
            drivePowerR = 0.5;
            targetHeading = 90;
        }


        telemetry.update();

        meccanum.Drive_Gyro_Vector(drivePowerX, drivePowerY, drivePowerR, navigation.getRawRotation(), targetHeading);
    }



    // Autonomous Code
    // Quadrant Detection Variables
    public State detectState = State.IDLE; // state machine flag - holds which step the state machine is on
    public int detectInc = 0; // keeps an index of which state we are at (only starts counting when at state 0)
    public boolean quadrantFound = false; //lol jk☺ - Rohit

    private double[][] detectCoords = {
                    {0, 0.3, 0, 0}, // move forward to be able to rotate
                    {0, 0, 0.6, 180}, // rotate 180 to look at picture
                    {0, 0, 0, 0}, // move right to line up with mat
                    {0.0, .3, -0.6, 0}, // rotate back to 0

    }; // Holds the coordinate points to be used with our navigation - first state holds the quadrant differences - second dimension holds the state - the thrid dimension holds x, y, and r (in that order)

    private int[] detectStepTimes = {
//            0     1     2   3    4     5     6    7    8    9   10    11
            300, 3_000, 500, 3_000, // Times in milisecs
    }; // fail safe times for each step in the autonomous program - in milisecs

    private int detectStartTime = 0;
    private int detectStateTargetTime = 0;
    private boolean detectStateFirstRun = true;

    // Main Auto Variables
    public State autoState = State.IDLE; // state machine flag - holds which step the state machine is on
    public int stateInc = 0; // keeps an index of which state we are at (only starts counting when at state 0)
    public int quadrant = 0;

    private double[][][] driveCoords = {
            { // quadrant 0 coordinates (red quarry side)
                    {0.0, 0.0, 0.5, 0}, // determine which spot the block is in
                    {0.0, 0.0, 0.5, 80}, // lock onto the picture
                    {0.0, -60.0, 0.5, 80}, // move to line up with block
                    {10.0, -50.0, 0.5, 90}, // get da block
                    {-5.0, -50.0, 0.5, 90}, // move down to move under bridge
                    {-5.0, -110.0, 0.5, 90}, // move to align with foundation
                    {-1.0, 1.0, 0.5, 90}, // move up to mat (lidar)
                    {0.0, 0.0, 0.0, 90}, // drop pullers
                    {-1.0, 0.0, 0.5, 0}, // rotate mat into position
                    {1.0, 1.0, 0.5, 0}, // ensure mat is in the proper position (lidar)
                    {1.0, 1.0, 0.5, 90}, // rotate and align with mat to place the block (while dropping the lift) (lidar)
                    {1.0, 1.0, 0.0, 0}, // align to move under the bridge (lidar)
                    {-5.0, -20.0, 0.5, 90}, // move to line up with 2nd block
                    {10.0, -20.0, 0.5, 120}, // move to line up with 2nd block via on the other axses
                    {10.0, -10.0, 0.5, 120}, // get da 2nd block
                    {-5.0, -10.0, 0.5, 90}, // move down to move under bridge
                    {-5.0, -110.0, 0.5, 90}, // move to foundation side
                    {1.0, 1.0, 0.5, 90}, // align with mat to place the block (lidar)
                    {0.0, 0.0, 0.0, 0}, // do the block placement things
                    {0.0, 0.0, 0.0, 0}, // move to prep to park
            },

            { // quadrant 1 coordinates (red mat side)
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
            },

            { // quadrant 2 coordinates (blue quarry side)
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
                    {0.0, 0.0, 0.0, 0}, //
            },

            { // quadrant 3 coordinates (blue mat side)
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
                    {0.0, 0.0, 0.0, 0}, // Empty step
            },
    }; // Holds the coordinate points to be used with our navigation - first state holds the quadrant differences - second dimension holds the state - the thrid dimension holds x, y, and r (in that order)

    private double[][][] lidarCoords = {
            { // quadrant 0 coordinates (red quarry side)
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //  // determine which spot the block is in
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, // move up to mat (lidar)
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, // ensure mat is in the proper position (lidar)
                    {0.0, 0.0}, // rotate and align with mat to place the block (while dropping the lift) (lidar)
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //
            },

            { // quadrant 1 coordinates (red mat side)
                    {0.0, 0.0}, // Empty step

            },

            { // quadrant 2 coordinates (blue quarry side)
                    {0.0, 0.0}, //
                    {0.0, 0.0}, //

            },

            { // quadrant 3 coordinates (blue mat side)
                    {0.0, 0.0}, // Empty step
            },
    }; // holds the desired distances for lidars on each step

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

        String targetInfo = "NULL";

        targetInfo = vision.targetsAreVisible();
        navigation.updateRotation();


        //State Machine

        if (!quadrantFound) { // if a quadrant has not yet been found, run the quadrant finding state machine
            switch (detectState) { // main state machine - the state determines the robot's actions - mostly movement, but with some extra manipulator action
                case IDLE:
                    detectStartTime = (int) runtime.milliseconds();


                    detectInc = 0;

                    pullerPower = 0.5;
                    //     moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards the first (starting coordinate)

                    detectState = State.STATE_0;
                    break;
                case STATE_0:
                    detectInc = 0;

                    if (detectStateFirstRun) {
                        detectStateTargetTime = (int) runtime.milliseconds() + detectStepTimes[stateInc]; // sets target fail safe time for this step

                        detectStateFirstRun = false;
                    }
                    movePowers[0] = detectCoords[detectInc][0];
                    movePowers[1] = detectCoords[detectInc][1];
                    movePowers[2] = detectCoords[detectInc][2];
                    //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                    pullerPower = 0.5;

                    if (excedesTime(detectStateTargetTime)) { // continue conditions (including failsafe times)
                        detectState = State.STATE_1;
                        detectStateFirstRun = true;
                    }
                    break;
                case STATE_1:
                    detectInc = 1;

                    if (detectStateFirstRun) {
                        detectStateTargetTime = (int) runtime.milliseconds() + detectStepTimes[detectInc]; // sets target fail safe time for this step

                        detectStateFirstRun = false;
                    }
                    turnComp = meccanum.gyroTurn(detectCoords[detectInc][3], navigation.getRawRotation(), 5);

                    movePowers[0] = detectCoords[detectInc][0];
                    movePowers[1] = detectCoords[detectInc][1];
                    if (!turnComp) {
                        movePowers[2] = detectCoords[detectInc][2];
                    }
                    //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                    pullerPower = 0.5;

                    if (excedesTime(detectStateTargetTime) || turnComp) { // continue conditions (including failsafe times)
                        detectState = State.STATE_2;
                        detectStateFirstRun = true;
                    }
                    break;
                case STATE_2:
                    detectInc = 2;

                    if (detectStateFirstRun) {
                        detectStateTargetTime = (int) runtime.milliseconds() + detectStepTimes[stateInc]; // sets target fail safe time for this step

                        detectStateFirstRun = false;
                    }
                    movePowers[0] = detectCoords[detectInc][0];
                    movePowers[1] = detectCoords[detectInc][1];
                    movePowers[2] = detectCoords[detectInc][2];

                    pullerPower = 0.5;

                    String currentTrackable = vision.getVisibleTarget();

                    if (currentTrackable != "NULL") {
                        switch (currentTrackable) {
                            case "Red Perimeter 2":
                                quadrant = 0;
                                break;
                            case "Red Perimeter 1":
                                quadrant = 1;
                                break;
                            case "Blue Perimeter 1":
                                quadrant = 2;
                                break;
                            case "Blue Perimeter 2":
                                quadrant = 3;
                                break;
                        }
                    }

                    if (excedesTime(detectStateTargetTime) || currentTrackable != "NULL") { // continue conditions (including failsafe times)
                        detectState = State.STATE_3;
                        detectStateFirstRun = true;
                    }
                    break;
                case STATE_3:
                    detectInc = 3;

                    if (detectStateFirstRun) {
                        detectStateTargetTime = (int) runtime.milliseconds() + detectStepTimes[stateInc]; // sets target fail safe time for this step

                        detectStateFirstRun = false;
                    }
                    turnComp = meccanum.gyroTurn(detectCoords[detectInc][3], navigation.getRawRotation(), 5);

                    movePowers[0] = detectCoords[detectInc][0];
                    movePowers[1] = detectCoords[detectInc][1];
                    if (!turnComp) {
                        movePowers[2] = detectCoords[detectInc][2];
                    }

                    pullerPower = 0.5;

                    if (excedesTime(detectStateTargetTime) || turnComp) { // continue conditions (including failsafe times)
                        detectState = State.STATE_4;
                        detectStateFirstRun = true;
                    }
                    break;
                case STATE_4:
                    detectInc = 4;

                    if (detectStateFirstRun) {
                        detectStateTargetTime = (int) runtime.milliseconds() + detectStepTimes[stateInc]; // sets target fail safe time for this step

                        detectStateFirstRun = false;
                    }
                    turnComp = meccanum.gyroTurn(detectCoords[detectInc][3], navigation.getRawRotation(), 5);

                    movePowers[0] = detectCoords[detectInc][0];
                    movePowers[1] = detectCoords[detectInc][1];

                    if (!turnComp) {
                        movePowers[2] = detectCoords[detectInc][2];
                    }


                    pullerPower = 0.5;

                    if (excedesTime(detectStateTargetTime) || turnComp) { // continue conditions (including failsafe times)
                        detectState = State.COMPLETE;
                        detectStateFirstRun = true;
                    }
                    break;
                case COMPLETE:

                    pullerPower = 0.5;
                    quadrantFound = true;

                    if(quadrant == 1 || quadrant == 3){ // if the quadrant is either mat side
                        autoState = State.COMPLETE;
                    }

                    break;
            }
        }
        else if(quadrantFound && (quadrant == 0 || quadrant == 2)){ // if quadrant found, run the main auto portion
                switch (autoState) { // main state machine - the state determines the robot's actions - mostly movement, but with some extra manipulator action
                    case IDLE:
                        autoStartTime = (int) runtime.milliseconds();


                        stateInc = 0;

                        pullerPower = 0.5;
                        //     moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards the first (starting coordinate)

                        autoState = State.STATE_0;
                        break;
                    case STATE_0:
                        stateInc = 0;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];
                        //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position


                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_1;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_1:
                        stateInc = 1;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        turnComp = meccanum.gyroTurn(driveCoords[quadrant][stateInc][3], navigation.getRawRotation(), 10);

                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        if (!turnComp) {
                            movePowers[2] = driveCoords[quadrant][stateInc][2];
                        }
                        //moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime) || turnComp) { // continue conditions (including failsafe times)
                            autoState = State.STATE_2;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_2:
                        stateInc = 2;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];
                        //   moveCoords = driveCoords[quadrant][stateInc]; // flag to move towards target position

                        pullerPower = 0.5;


                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_3;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_3:
                        stateInc = 3;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_4;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_4:
                        stateInc = 4;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];


                        // Have a whole state to drop the servo

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_5;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_5:
                        stateInc = 5;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];


                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_6;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_6:
                        stateInc = 6;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];


                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_7;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_7:
                        stateInc = 7;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_8;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_8:
                        stateInc = 8;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_9;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_9:
                        stateInc = 9;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_10;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_10:
                        stateInc = 10;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;
                        robot.intakeDropL.setPosition(0.0); //
                        robot.intakeDropR.setPosition(1.0);

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
                            autoState = State.STATE_11;
                            autoStateFirstRun = true;
                        }
                        break;
                    case STATE_11:
                        stateInc = 11;

                        if (autoStateFirstRun) {
                            autoStateTargetTime = (int) runtime.milliseconds() + autoStepTimes[quadrant][stateInc]; // sets target fail safe time for this step

                            autoStateFirstRun = false;
                        }
                        movePowers[0] = driveCoords[quadrant][stateInc][0];
                        movePowers[1] = driveCoords[quadrant][stateInc][1];
                        movePowers[2] = driveCoords[quadrant][stateInc][2];

                        pullerPower = 0.5;
                        robot.intakeDropL.setPosition(0.0); //
                        robot.intakeDropR.setPosition(1.0);

                        if (excedesTime(autoStateTargetTime)) { // continue conditions (including failsafe times)
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
            }


            if (autoState == State.COMPLETE) {
                DYNAMIC_PARK_ACTIVE = true;
            }

            if(DYNAMIC_PARK_ACTIVE = true)
            {//☺
                double[] nPwr = DynamicPark(); //setting powers from dynamic park
                movePowers[0] = nPwr[0];
                movePowers[1] = nPwr[1];
                movePowers[2] = nPwr[2];
                pullerPower   = nPwr[3];
            }


            if (quadrant == 0 || quadrant == 2) { // move intakes up if doing skystone auto (if in the right
                robot.intakeDropL.setPosition(0.0);
                robot.intakeDropR.setPosition(1.0);
            }


            meccanum.Drive_Gyro_Vector(movePowers[0], movePowers[1], movePowers[2], navigation.getRawRotation(), driveCoords[quadrant][stateInc][3]);
            pullerDrop.set_ServoPower(pullerPower, robot.pullerDropL, robot.pullerDropR);


            telemetry.addData("State:", autoState);
            telemetry.addData("", targetInfo);
            telemetry.update();

        }



    // ---Dynamic Park Code---
    // Dynamic Parking Variables
    private double[] lidarValues = {0, 0, 0, 0};
    //                              L BL BR  R
    private int parkStep = 0;
    private int lidarSide = 0;
    private int step5StartTime = 0;
    private double wallDistance = 0;
    private boolean parkFirstRun = true;
    private boolean wallLeft = false;
    private boolean wallRight = false;
    private boolean moveWallSide = true;
    private boolean firstRunStep4 = true;
    private boolean firstRunStep5 = true;
    private boolean parkFinishêd = false;

    public double[] DynamicPark()
    {
        double[] movePower = {0, 0, 0};
        double pullerPower = 0;

        if(parkFirstRun)
        {
            parkStep = 1;
            parkFirstRun = false;
        }
        switch (parkStep)
        {
            case 1:
            {
                lidarValues = getLidar(0);
                if (lidarValues[0] < 100)
                {
                    lidarSide = 1;
                    wallLeft = true;
                    parkStep = 2;
                }
                else if (lidarValues[3] < 100)
                {
                    lidarSide = 2;
                    wallRight = true;
                    parkStep = 3;
                }
            }
            case 2:
            {
                lidarValues = getLidar(1);
                if(moveWallSide)
                {
                    if(lidarValues[0] > 10)
                    {
                        if(lidarValues[1] < 100 || lidarValues[2] < 100)
                        {
                            movePower[1] = -0.3;
                        }
                        else
                        {
                            parkStep = 4;
                        }
                    }
                    else
                    {
                        moveWallSide = false;
                    }
                }
                else
                {
                    if(lidarValues[0] < 100)
                    {
                        if(lidarValues[1] < 100 || lidarValues[2] < 100)
                        {
                            movePower[1] = 0.3;
                        }
                        else
                        {
                            parkStep = 4;
                        }
                    }
                    else
                    {
                        moveWallSide = true;
                    }
                }
            }
            case 3:
            {
                lidarValues = getLidar(2);
                if(moveWallSide)
                {
                    if(lidarValues[3] > 10)
                    {
                        if(lidarValues[1] < 100 || lidarValues[2] < 100)
                        {
                            movePower[1] = -0.3;
                        }
                        else
                        {
                            parkStep = 4;
                        }
                    }
                    else
                    {
                        moveWallSide = false;
                    }
                }
                else
                {
                    if(lidarValues[3] < 100)
                    {
                        if(lidarValues[1] < 100 || lidarValues[2] < 100)
                        {
                            movePower[1] = 0.3;
                        }
                        else
                        {
                            parkStep = 4;
                        }
                    }
                    else
                    {
                        moveWallSide = true;
                    }
                }
            }
            case 4:
            {
                lidarValues = getLidar(lidarSide);
                double totalWallDistance = 0;
                if(firstRunStep4)
                {
                    firstRunStep4 = false;
                    if(wallLeft)
                    {
                        wallDistance = lidarValues[0];
                    }
                    if(wallRight)
                    {
                        wallDistance = lidarValues[3];
                    }
                }
                if(wallLeft)
                {
                    totalWallDistance = Math.abs(wallDistance - lidarValues[0]);
                }
                if(wallRight)
                {
                    totalWallDistance = Math.abs(wallDistance - lidarValues[3]);
                }

                if(totalWallDistance < 10)
                {
                    if (moveWallSide) {

                        movePower[1] = -0.3;
                    } else {

                        movePower[1] = 0.3;
                    }
                }
                else
                {
                    parkStep = 5;
                }
            }
            case 5:
            {
                if(firstRunStep5)
                {
                    step5StartTime = (int)runtime.milliseconds();
                    firstRunStep5 = false;
                }
                if(!excedesTime(step5StartTime + 500))
                {
                    if(wallLeft)
                    {
                        movePower[0] = 0.3;
                    }
                    if(wallRight)
                    {
                        movePower[0] = -0.3;
                    }
                }
                else
                {
                    parkStep = 6;
                }
            }
        }

        return new double[] {movePower[0], movePower[1], movePower[2], pullerPower};
    }
    private double[] getLidar(int side) // 0 = All lidars, 1 = Left Lidars, 2 = Right lidars
    {
        double lValue[] = {0, 0, 0, 0};
        switch (side)
        {
            case 0:
                lValue[0] = robot.readFlight(robot.flightLeft1);
                lValue[3] = robot.readFlight(robot.flightRight2);
                break;
            case 1:
                lValue[0] = robot.readFlight(robot.flightLeft1);
                break;
            case 2:
                lValue[3] = robot.readFlight(robot.flightRight2);
                break;
        }
        lValue[1] = robot.readFlight(robot.flightFront0);
        lValue[2] = robot.readFlight(robot.flightBack3);

        return lValue;
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
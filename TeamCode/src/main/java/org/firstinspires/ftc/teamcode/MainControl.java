package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



public class MainControl {

    //Key parameter variables

    Motion         motion   = new Motion();
    Navigation2019 navigation = new Navigation2019();


    public int INIT_FIELD_POS = 0; // Quad 1,2,3,4
    public int AUTO_RUNS_TOT = 3;
    public int CURR_RUN_TOT =0;



    // Startup flags
    public boolean ROBOT_DEAD_STOP = false;
    public boolean SENSORS_READY = false;
    public boolean DRIVE_READY = false;
    public boolean INIT_COMPLETE = false;
    public boolean INIT_FAIL = false;
    public boolean AUTO_MODE_ACTIVE = false;



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


    // State Steps

    public enum State{

        // You need to define what your states are going to be called.
        STATE_INIT,
        STATE_START,
        STATE_MOVE_POS,
        STATE_INTAKE_POS,
        STATE_PLACE_BLOCK,
        STATE_SEQ_COMPLETE,
        STATE_STOP

    }



    // Startup initialization

    // Set flag for either Auto or Manual based on Opmode selected
    // If flag is in Auto get initial field position from naviagtion routine
    // If flag is manual mode do not call navigation routine
    // Check sensors and drive and set initialization ok flag

// I would call manual mode from the TeleOp Opmode
    public void manual_mode(){

        if (AUTO_MODE_ACTIVE == false){

            // you can just call the other subroutines here and pass a mode flag







        }

    }


    // State machine code


    public State CURR_STATE = State.STATE_INIT;

    public void AutoStep() {



        switch(CURR_STATE){


            case STATE_INIT:
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





    }


    public void Set_AutoMode(boolean mode){
        AUTO_MODE_ACTIVE = mode;
    }

}

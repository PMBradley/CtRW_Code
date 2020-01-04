package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import java.lang.Math;

public class Navigation2019 {

    Robot2019 robot;

    /* Constructor */
    public Navigation2019(Robot2019 robot){
        this.robot = robot;
    }

    // Field dimensions
    private double FRAME_OFFSET = 22.0; // CM from frame edge to center
    private double FIELD_LENGTH = 11.75; //
    private double FIELD_WIDTH = 11.75;
    private double FIELD_HALF_LENGTH = FIELD_LENGTH/2;
    private double FIELD_HALF_WIDTH = FIELD_WIDTH/2;

    //Waypoints
    private double[][] WAYPOINT;
    private double DistanceErrors[];

    private int CURRENT_WAYPOINT = 0;

    // Wall Distances
    private double LEFT_LIDAR_DISTANCE = 0.0;
    private double RIGHT_LIDAR_DISTANCE = 0.0;
    private double BACK_LIDAR_DISTANCE = 0.0;
    private double FRONT_LIDAR_DISTANCE = 0.0;

    public double X = 0.0;
    public double Y = 0.0;
    public double ROTATION_DEG = 0.0;
    private double lastX = X;
    private double lastY = Y;

    public double CURRENT_LOCATION[]; // X, Y, Rotation



    public double[] getCurrentLocation(){

        updateLocation();

        return CURRENT_LOCATION;
    }

    public double getX(){
        updateLocation();

        return X;
    }

    public double getY(){
        updateLocation();

        return Y;
    }

    public double getRotation(){
        updateRotation();

        return ROTATION_DEG;
    }

 public int getCurrentQuadrant(){

     int Quadrant = 0;

     //Set quadrant for autonomous start, probably need to add the skystone target also from Vuforia
     if((LEFT_LIDAR_DISTANCE < FIELD_HALF_LENGTH) && (RIGHT_LIDAR_DISTANCE > FIELD_HALF_LENGTH)
             && (BACK_LIDAR_DISTANCE < FIELD_HALF_WIDTH) && (FRONT_LIDAR_DISTANCE > FIELD_HALF_WIDTH)){

      Quadrant = 1; // Blue 1

         // Load position specific waypoints based on starting position

         WAYPOINT[0][0] = 0.0;
         WAYPOINT[0][1] = 0.0;
         WAYPOINT[0][2] = 0.0;
         WAYPOINT[0][3] = 0.0;

         WAYPOINT[1][0] = 0.0;
         WAYPOINT[1][1] = 0.0;
         WAYPOINT[1][2] = 0.0;
         WAYPOINT[1][3] = 0.0;

         WAYPOINT[2][0] = 0.0;
         WAYPOINT[2][1] = 0.0;
         WAYPOINT[2][2] = 0.0;
         WAYPOINT[2][3] = 0.0;

         WAYPOINT[3][0] = 0.0;
         WAYPOINT[3][1] = 0.0;
         WAYPOINT[3][2] = 0.0;
         WAYPOINT[3][3] = 0.0;

     }

     if((LEFT_LIDAR_DISTANCE > FIELD_HALF_LENGTH) && (RIGHT_LIDAR_DISTANCE < FIELD_HALF_LENGTH)
             && (BACK_LIDAR_DISTANCE < FIELD_HALF_WIDTH) && (FRONT_LIDAR_DISTANCE > FIELD_HALF_WIDTH)){

         Quadrant = 2; // Blue 2

         // Load position specific waypoints based on starting position

         WAYPOINT[0][0] = 0.0;
         WAYPOINT[0][1] = 0.0;
         WAYPOINT[0][2] = 0.0;
         WAYPOINT[0][3] = 0.0;

         WAYPOINT[1][0] = 0.0;
         WAYPOINT[1][1] = 0.0;
         WAYPOINT[1][2] = 0.0;
         WAYPOINT[1][3] = 0.0;

         WAYPOINT[2][0] = 0.0;
         WAYPOINT[2][1] = 0.0;
         WAYPOINT[2][2] = 0.0;
         WAYPOINT[2][3] = 0.0;

         WAYPOINT[3][0] = 0.0;
         WAYPOINT[3][1] = 0.0;
         WAYPOINT[3][2] = 0.0;
         WAYPOINT[3][3] = 0.0;

     }



     return Quadrant;
 }

 public double[] getDistanceError(){

        double DistanceErrorFront = 0.0;
        double DistanceErrorBack = 0.0;
        double DistanceErrorLeft = 0.0;
        double DistanceErrorRight = 0.0;


        int LEFT = 0;
        int RIGHT = 1;
        int BACK = 2;
        int FRONT = 3;

        DistanceErrorLeft = WAYPOINT[CURRENT_WAYPOINT][LEFT] - CURRENT_LOCATION[LEFT];
        DistanceErrorRight = WAYPOINT[CURRENT_WAYPOINT][RIGHT] - CURRENT_LOCATION[RIGHT];
        DistanceErrorBack = WAYPOINT[CURRENT_WAYPOINT][BACK] - CURRENT_LOCATION[BACK];
        DistanceErrorFront = WAYPOINT[CURRENT_WAYPOINT][FRONT] - CURRENT_LOCATION[FRONT];


        // May need to include some absolutes
        DistanceErrors[0] = DistanceErrorLeft;
        DistanceErrors[1] = DistanceErrorRight;
        DistanceErrors[2] = DistanceErrorBack;
        DistanceErrors[3] = DistanceErrorFront;

        // Waypoint_Tracking();

    return  DistanceErrors;
 }

 /*public void Waypoint_Tracking(){

        //boolean Waypoint_Reached = false;
        double  RANGE_OFFSET = 1.0;

        if ((DistanceErrors[0] < RANGE_OFFSET) &&
            (DistanceErrors[1] < RANGE_OFFSET) &&
            (DistanceErrors[2] < RANGE_OFFSET) &&
                (DistanceErrors[3] < RANGE_OFFSET)
        ){
            CURRENT_WAYPOINT++;


        }


 }*/

 public void updateLocation(){
     updateRotation();
     updateFlight();


 }


 int[] sensorWalls = {0, 1 , 2, 3}; // 0 is front wall, 1 is right wall, 2 is back wall, 3 is left wall

 public void updateFlightWalls(){ // updates what sensors are
     updateRotation();
     double[] cornerAngles = {315, 45, 135, 205}; // angles to corners (0 being a vertical line) - front left corner is 0, front right is 1, etc

     cornerAngles[3] = clipDegrees(Math.atan(X/Y) + 180); // + 180 accounts for 0 degrees being in front
     cornerAngles[0] = clipDegrees(cornerAngles[3] + 90);
     cornerAngles[1] = clipDegrees(cornerAngles[0] + 90);
     cornerAngles[2] = clipDegrees(cornerAngles[1] + 90);


 }

 public void updateFlight(){
    updateFlightWalls();

    lastX = X;
    lastY = Y;


 }

 public void updateRotation(){
   // robot.imu.;
 }


 public double clipDegrees(double inputDeg) {
     double output = inputDeg;
     if (output > 0) {
         while (output > 360) {
             output -= 360;
         }
     } else {
         while (output < -360) {
             output += 360;
         }
     }
    return (output);
 }
}

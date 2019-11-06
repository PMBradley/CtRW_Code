package org.firstinspires.ftc.teamcode;

public class Navigation2019 {


    /* Constructor */
    public Navigation2019(){

    }

    // Field dimensions
    private double FRAME_OFFSET = 7.0;
    private double FIELD_LENGTH = 11.75;
    private double FIELD_WIDTH = 11.75;
    private double FIELD_HALF_LENGTH = FIELD_LENGTH/2;
    private double FIELD_HALF_WIDTH = FIELD_WIDTH/2;

    //Waypoints
    private double[][] WAYPOINT;
    private double DistanceErrors[];

    private int CURRENT_WAYPOINT =0;

    // Wall Distances
    private double LEFT_LIDAR_DISTANCE = 0.0;
    private double RIGHT_LIDAR_DISTANCE = 0.0;
    private double BACK_LIDAR_DISTANCE = 0.0;
    private double FRONT_LIDAR_DISTANCE = 0.0;


    public double CURRENT_LOCATION[]; //W1, W2, W3, W4


    // Get initial LIDAR distances

    //   LEFT_LIDAR_DISTANCE = getLidarSensor(flight0);
    //   RIGHT_LIDAR_DISTANCE = getLidarSensor(flight1);
    //   FRONT_LIDAR_DISTANCE = getLidarSensor(flight2);
    //   BACK_LIDAR_DISTANCE = getLidarSensor(flight3);

    public double[] getCurrentLocation(){



        //   LEFT_LIDAR_DISTANCE = getLidarSensor(flight0);
        //   RIGHT_LIDAR_DISTANCE = getLidarSensor(flight1);
        //   FRONT_LIDAR_DISTANCE = getLidarSensor(flight2);
        //   BACK_LIDAR_DISTANCE = getLidarSensor(flight3);


       // Get location vectors
        CURRENT_LOCATION[0] = LEFT_LIDAR_DISTANCE;
        CURRENT_LOCATION[1] = RIGHT_LIDAR_DISTANCE;
        CURRENT_LOCATION[2] = BACK_LIDAR_DISTANCE;
        CURRENT_LOCATION[3] = FRONT_LIDAR_DISTANCE;


     //Active distance finding
     return CURRENT_LOCATION;
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

 public void Waypoint_Tracking(){

        //boolean Waypoint_Reached = false;
        double  RANGE_OFFSET = 1.0;

        if ((DistanceErrors[0] < RANGE_OFFSET) &&
            (DistanceErrors[1] < RANGE_OFFSET) &&
            (DistanceErrors[2] < RANGE_OFFSET) &&
                (DistanceErrors[3] < RANGE_OFFSET)
        ){
            CURRENT_WAYPOINT++;


        }


 }

}

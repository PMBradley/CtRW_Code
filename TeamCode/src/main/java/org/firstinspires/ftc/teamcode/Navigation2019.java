package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.lang.Math;

public class Navigation2019 {

    Robot2019 robot;

    /* Constructor */
    public Navigation2019(Robot2019 robot){
        this.robot = robot;
    }

    // Field dimensions
    private double FRAME_OFFSET = 22.0; // CM from frame edge to center
    private double FIELD_HEIGHT = 11.75; //
    private double FIELD_WIDTH = 11.75;
    private double FIELD_HALF_LENGTH = FIELD_HEIGHT/2;
    private double FIELD_HALF_WIDTH = FIELD_WIDTH/2;

    private double[] FLIGHT_OFFSETS = {20, 17, 24, 21}; // offsets from middle for each time of flight sensor

    private double transMargin = 5; // the number of centimeters that the robot has to be within the target translational position
    private double rotMargin = 3; // the number of degrees that the robot has to be within the target rotation

    //Waypoints
 //   private double[][] WAYPOINT;
  //  private double DistanceErrors[];
 //   private int CURRENT_WAYPOINT = 0;

    // Wall Distances
    private double LEFT_LIDAR_DISTANCE = 0.0;
    private double RIGHT_LIDAR_DISTANCE = 0.0;
    private double BACK_LIDAR_DISTANCE = 0.0;
    private double FRONT_LIDAR_DISTANCE = 0.0;

    public double X = 10.0;
    public double Y = 10.0;
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

    void setNavMargin(double inTranslationMargin, double inRotationMargin){
        transMargin = inTranslationMargin;
        rotMargin = inRotationMargin;
    }

    public boolean atCoord(double inX, double inY, double inR){ // determines if the robot is currently within the margin of errors to a coordinate
        boolean output = false;
        boolean[] withinConstraint = {false, false, false};

        // Check translation
        if(X < inX + transMargin && X > inX - transMargin){
            withinConstraint[0] = true;
        }
        if(Y < inY + transMargin && Y > inY - transMargin){
            withinConstraint[1] = true;
        }

        // Check rotation
        double[] checkRotations = {clipDegrees(inR - rotMargin), clipDegrees(inR + rotMargin)}; // the range that the robot must be within
        if(ROTATION_DEG - rotMargin < 0){ // account the range for the fact that it might fall close to the cut off point on the 360 degree range
            checkRotations[0] -= 360;
        }
        else if(ROTATION_DEG + rotMargin >= 360){
            checkRotations[1] += 360;
        }

        if(ROTATION_DEG >= checkRotations[0] && ROTATION_DEG <= checkRotations[1]){ // check rotation
            withinConstraint[2] = true;
        }

        if(withinConstraint[0] && withinConstraint[1] && withinConstraint[2]){ // if within parameters on all three constraints
            output = true;
        }

        return (output);
    }



 public void updateLocation(){
     updateRotation();
     updateFlight();
     //double temp = robot.readFlight(robot.flightFront0);

 }


 int[] sensorWalls = {0, 1 , 2, 3}; // 0 is front wall, 1 is right wall, 2 is back wall, 3 is left wall

 public void updateFlightWalls(){ // updates what sensors are
     updateRotation();
     double[] cornerAngles = {315, 45, 135, 205}; // angles to corners (0 being a vertical line) - front left corner is 0, front right is 1, etc

     cornerAngles[3] = clipDegrees(Math.atan(X/Y) + 180); // + 180 accounts for 0 degrees being in front
     cornerAngles[0] = clipDegrees(cornerAngles[3] + 90); // the other corners are just variations on the previous ones
     cornerAngles[1] = clipDegrees(cornerAngles[0] + 90);
     cornerAngles[2] = clipDegrees(cornerAngles[1] + 90);

     double rotationInc = 0;

     for(int i = 0; i < 4; i++){
         boolean foundCorner = false;

         for(int j = 0; j < 4 && !foundCorner; j++){
             double sensorLocation = clipDegrees(ROTATION_DEG + rotationInc); // get the degree orientation of the current sensor

             if (j < 3) { // two if statements to prevent weirdness with needing to check between corner 3 and 0
                 if(sensorLocation > cornerAngles[j] && sensorLocation < cornerAngles[j + 1]){
                     sensorWalls[i] = j;
                     foundCorner = true;
                 }
             }
             else { // if none of the other ones
                 if(sensorLocation > cornerAngles[3]){  // hope it is this one
                     sensorWalls[i] = j; // set the current
                     foundCorner = true;
                 }
             }
         }

         rotationInc += 90;
     }
 }

 public void updateFlight(){
    updateFlightWalls();

    double newX = 1;
    double newY = 1;
    double[] reading = {1, 1, 1, 1};

    for(int i = 0; i < 4; i++){ // run once for each sensor

        double wallDistance = 1;

        switch (i){ // get the reading from the proper time of flight sensor
            case 0:
                reading[i] = robot.readFlight(robot.flightFront0) + FLIGHT_OFFSETS[i]; // get the reading and adjust to the offset
                break;
            case 1:
                reading[i] = robot.readFlight(robot.flightRight2) + FLIGHT_OFFSETS[i];
                break;
            case 2:
                reading[i] = robot.readFlight(robot.flightBack3) + FLIGHT_OFFSETS[i];
                break;
            case 3:
                reading[i] = robot.readFlight(robot.flightLeft1) + FLIGHT_OFFSETS[i];
                break;
        }

        double wallOffset = 0; // mathematical offset for which wall is being measured
//
        for(int j = 0; j < sensorWalls[i]; j++){
            wallOffset += 90;
        }

        wallDistance = reading[i] * Math.cos(clipDegrees(ROTATION_DEG + wallOffset)); // set the wall distance


        if(sensorWalls[i] == 0 || sensorWalls[i] == 2){ // if measuring the top or bottom walls
            double tempY = 1;

            if(sensorWalls[i] == 2){ // if measuring bottom wall
                tempY = wallDistance; // set to the reading
            }
            else {
                tempY = FIELD_HEIGHT - wallDistance; // field height minus the reading
            }

            boolean setValue = true;

            for(int j = 0; j < i; j++){ // check each sensor before it
                if(sensorWalls[i] == sensorWalls[j]){ // if that sensor is measuring the same wall
                    if(reading[i] < reading[j]){ // and had a greater reading
                        setValue = false; // do not overwrite the old y value
                    }
                }
            }

            if(setValue){
                newY = tempY;
            }
        }
        else { // if measuring the left or right walls (not the other ones)
            double tempX = 1;

            if(sensorWalls[i] == 3){ // if measuring left wall
                tempX = wallDistance; // set to the reading
            }
            else {
                tempX = FIELD_WIDTH - wallDistance; // field height minus the reading
            }

            boolean setValue = true;

            for(int j = 0; j < i; j++){ // check each sensor before it
                if(sensorWalls[i] == sensorWalls[j]){ // if that sensor is measuring the same wall
                    if(reading[i] < reading[j]){ // and had a greater reading
                        setValue = false; // do not overwrite the old x value
                    }
                }
            }

            if(setValue){
                newX = tempX;
            }
        }

    }

    X = newX;
    Y = newY;
 }

 public void updateRotation(){
     double tempRot = robot.getHeading();

     tempRot *= -1;

     ROTATION_DEG = clipDegrees(tempRot);
 }


 public double clipDegrees(double inputDeg) { // utility function used to ensure a rotation value never goes to or above 360 degrees
     double output = inputDeg;
     if (output >= 0) {
         while (output >= 360) {
             output -= 360;
         }
     } else {
         while (output < 0) {
             output += 360;
         }
     }
    return (output);
 }


 public int getCurrentQuadrant(){
     int quadrant = 0;

     if(X > FIELD_HALF_WIDTH){
         quadrant = 1;
     }

     return quadrant;
 }
}

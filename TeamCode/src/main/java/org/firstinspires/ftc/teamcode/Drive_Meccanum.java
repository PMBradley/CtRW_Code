package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive_Meccanum {


  //  Robot2019 robot = new Robot2019();

    private Robot2019 robot;

    private double turnDivisor = 0.5;
    private double speedDivisor = 0.40;


    public Drive_Meccanum(Robot2019 robot)
    {
        this.robot = robot;
    }



    //Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private double teleTargetHeading = 0;
    private double rotateSpeedFactor = 10;
    private double correctionPower = 0.5;

    boolean isInitilized = false;

    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;


        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public void Drive_Vector(double x, double y, double r, double heading) { // use with auto only
        //x = Math.sin(heading) * x;
        //y = Math.cos(heading) * y;

        y *= -1; // negate y to emulate the negative values given by controller y

        double sin = Math.sin(heading * 0.0174533);
        double cos = Math.cos(heading * 0.0174533);

        double forward = (x * sin) + (y * cos);
        double right = (x * cos) - (y * sin);

        robot.fl = forward + (r * turnDivisor) + right;
        robot.fr = (forward - (r * turnDivisor) - right);
        robot.bl = forward + (r * turnDivisor) - right;
        robot.br = (forward - (r * turnDivisor) + right);

        robot.driveFL.setPower(robot.fl);
        robot.driveFR.setPower(-robot.fr);
        robot.driveBL.setPower(robot.bl);
        robot.driveBR.setPower(-robot.br);
    }

    public void Drive_Vector(double x, double y, double r, double heading, boolean limiter, double Ltrigger) { // use with controller only
        //x = Math.sin(heading) * x;
        //y = Math.cos(heading) * y;
        double boostFactor = Ltrigger;

        if (boostFactor < .5 && limiter == true)
        {
            x = x * speedDivisor;
            y = y * speedDivisor;
        }

        double sin = Math.sin(heading * 0.0174533);
        double cos = Math.cos(heading * 0.0174533);

        double forward = (x * sin) + (y * cos);
        double right = (x * cos) - (y * sin);

        robot.fl = forward + (r * turnDivisor) + right;
        robot.fr = (forward - (r * turnDivisor) - right);
        robot.bl = forward + (r * turnDivisor) - right;
        robot.br = (forward - (r * turnDivisor) + right);

        robot.driveFL.setPower(robot.fl);
        robot.driveFR.setPower(-robot.fr);
        robot.driveBL.setPower(robot.bl);
        robot.driveBR.setPower(-robot.br);
    }

    // How much power to give the motors at different distances from the target
    double[] PIDMargins = {2, 20, 80, 360};
    double[] PIDPowers = {0, 0.15, 0.3, 0.5};

    public void Drive_Gyro_Vector(double x, double y, double r, double heading, boolean limiter, double boostFactor) { // use with tele-op only
        double minRotPower = 0.2;
        double correctionDegreeMargin = 1;
        double correctionDivisor = 0.02;
        double rPower = 0.0;

        x *= -1; // negate the inputs to correct for controller deviations
        r *= -1;

        teleTargetHeading += r * rotateSpeedFactor; // move the target heading

        double tempTargetHeading = teleTargetHeading;

        if (boostFactor < .5 && limiter == true)
        {
            x = x * speedDivisor;
            y = y * speedDivisor;
        }

        if(tempTargetHeading > heading + 180){
            tempTargetHeading -= 360;
        }
        else if(tempTargetHeading < heading - 180){
            tempTargetHeading += 360;
        }

        double targetDelta = Math.abs(heading - tempTargetHeading); // the raw distance to the target

       // rPower = (tempTargetHeading - heading)*correctionDivisor; // do math to set the power

        if(targetDelta < PIDMargins[0]){ // progress down the list until one it is found to be true (remembering if an else if is true, the if above it must be false)
            rPower = PIDPowers[0];
        }
        else if(targetDelta < PIDMargins[1]){
            rPower = PIDPowers[1];
        }
        else if(targetDelta < PIDMargins[2]){
            rPower = PIDPowers[2];
        }
        else if(targetDelta < PIDMargins[3]){
            rPower = PIDPowers[3];
        }

        if(tempTargetHeading - heading < 0){ // negate the power if the target is in a negative direction relative to the heading
            rPower *= -1;
        }


        double sin = Math.sin(heading * 0.0174533);
        double cos = Math.cos(heading * 0.0174533);

        double forward = (x * sin) + (y * cos);
        double right = (x * cos) - (y * sin);

        robot.fl = forward + (rPower * turnDivisor) + right;
        robot.fr = (forward - (rPower * turnDivisor) - right);
        robot.bl = forward + (rPower * turnDivisor) - right;
        robot.br = (forward - (rPower * turnDivisor) + right);

        robot.driveFL.setPower(robot.fl);
        robot.driveFR.setPower(-robot.fr);
        robot.driveBL.setPower(robot.bl);
        robot.driveBR.setPower(-robot.br);
    }

    public void Drive_Gyro_Vector(double x, double y, double r, double heading, double targetHeading) { // use with auto only
        double minRotPower = 0.1;
        double correctionDegreeMargin = 1;
        double correctionDivisor = .01;

        y *= -1; // negate y to emulate the negative values given by controller y


        if(targetHeading > heading + 180){
            targetHeading -= 360;
        }
        else if(targetHeading < heading - 180){
            targetHeading += 360;
        }

        double targetDelta = Math.abs(heading - targetHeading); // the raw distance to the target

        // rPower = (tempTargetHeading - heading)*correctionDivisor; // do math to set the power

        if(targetDelta < PIDMargins[0]){ // progress down the list until one it is found to be true (remembering if an else if is true, the if above it must be false)
            r = PIDPowers[0];
        }
        else if(targetDelta < PIDMargins[1]){
            r = PIDPowers[1];
        }
        else if(targetDelta < PIDMargins[2]){
            r = PIDPowers[2];
        }
        else if(targetDelta < PIDMargins[3]){
            r = PIDPowers[3];
        }

        if(targetHeading - heading < 0){ // negate the power if the target is in a negative direction relative to the heading
            r *= -1;
        }
        //r = r * (Math.sqrt(Math.abs(targetHeading - heading)))/correctionDivisor;
     //   r = (targetHeading - heading) * correctionDivisor

       /* if(!(Math.abs(heading - targetHeading) < correctionDegreeMargin)) { // if it is not within the degree margin of error, then move at minimum power towards target
            if (r > 0 && r < minRotPower) {
                r = minRotPower;
            } else if (r < 0 && r > -minRotPower) {
                r = -minRotPower;
            }
        }
        else {
            r = 0;
        }*/

        double sin = Math.sin(heading * 0.0174533);
        double cos = Math.cos(heading * 0.0174533);

        double forward = (x * sin) + (y * cos);
        double right = (x * cos) - (y * sin);

        robot.fl = forward + (r * turnDivisor) + right;
        robot.fr = (forward - (r * turnDivisor) - right);
        robot.bl = forward + (r * turnDivisor) - right;
        robot.br = (forward - (r * turnDivisor) + right);

        robot.driveFL.setPower(robot.fl);
        robot.driveFR.setPower(-robot.fr);
        robot.driveBL.setPower(robot.bl);
        robot.driveBR.setPower(-robot.br);
    }

    public boolean gyroTurn(double targetHeading, double currentHeading, double turntolerance)
    {
        //Gyro turn code
        //double targetHeading  - 0 to 360
        //double currentHeading - 0 to 360
        //double turnDirection  - -1 to 1
        boolean turnComplete = false;

        if(Math.abs(currentHeading - targetHeading) <= turntolerance)
        {
            turnComplete = true;
        }
        return turnComplete;
    }
}

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

    boolean isInitilized = false;


    public void init_motors(){
        // robot.init(hardwareMap);
        isInitilized = true;


        robot.driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    public void Drive_Polar(double x, double y, double r, double heading, boolean limiter) { // use with auto only
        //x = Math.sin(heading) * x;
        //y = Math.cos(heading) * y;

        y *= -1; // negate y to emulate the negative values given by controller y

        if (Math.abs(x) < .9 && limiter == true)
        {
            x = x * speedDivisor;
        }
        if(Math.abs(y) < .9 && limiter == true)
        {
            y = y * speedDivisor;
        }

        double sin = Math.sin(-heading * 0.0174533);
        double cos = Math.cos(-heading * 0.0174533);

        y = x * sin + y * cos;
        x = x * cos - y * sin;

        double fl = y - x - (r * turnDivisor);
        double fr = y + x + (r * turnDivisor);
        double bl = y + x - (r * turnDivisor);
        double br = y - x + (r * turnDivisor);

        robot.driveFL.setPower(fl);
        robot.driveFR.setPower(-fr);
        robot.driveBL.setPower(bl);
        robot.driveBR.setPower(-br);
    }

    public void Drive_Polar(double x, double y, double r, double heading, double Ltrigger, boolean limiter) { // use with controller only
        //x = Math.sin(heading) * x;
        //y = Math.cos(heading) * y;
        double boostFactor = Ltrigger;

        if (boostFactor < .5 && limiter == true)
        {
            x = x * speedDivisor;
            y = y * speedDivisor;
        }

        double[] tmp = adapt_Heading(x, y, heading);
        x = tmp[0];
        y = tmp[1];

        double fl = y - x - (r * turnDivisor);
        double fr = y + x + (r * turnDivisor);
        double bl = y + x - (r * turnDivisor);
        double br = y - x + (r * turnDivisor);

        robot.driveFL.setPower(fl);
        robot.driveFR.setPower(-fr);
        robot.driveBL.setPower(bl);
        robot.driveBR.setPower(-br);
    }

    public void Drive_Vector(double x, double y, double r, double heading, boolean limiter) { // use with auto only
        //x = Math.sin(heading) * x;
        //y = Math.cos(heading) * y;

        y *= -1; // negate y to emulate the negative values given by controller y

        if (Math.abs(x) < .9 && limiter == true)
        {
            x = x * speedDivisor;
        }
        if(Math.abs(y) < .9 && limiter == true)
        {
            y = y * speedDivisor;
        }

        double sin = Math.sin(-heading * 0.0174533);
        double cos = Math.cos(-heading * 0.0174533);

       // (AngleUnit.DEGREES.toRadians())


        double fl = y - x - (r * turnDivisor);
        double fr = y + x + (r * turnDivisor);
        double bl = y + x - (r * turnDivisor);
        double br = y - x + (r * turnDivisor);

        robot.driveFL.setPower(fl);
        robot.driveFR.setPower(-fr);
        robot.driveBL.setPower(bl);
        robot.driveBR.setPower(-br);
    }

    public void Drive_Vector(double x, double y, double r, double heading, double Ltrigger, boolean limiter) { // use with controller only
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

    public void drive_Controller(double lStickX, double lStickY, double rStickX)
    {
       /* if(!isInitilized)
            init_motors();
       */
        double magnitude = Math.hypot(lStickX, lStickY);
        double robotAngle = Math.atan2(lStickY, lStickX)- Math.PI/4;
        double rightX = rStickX;



        final double driveFLPwr = magnitude * Math.cos(robotAngle) + rightX;
        final double driveFRPwr = magnitude * Math.sin(robotAngle) - rightX;
        final double driveBLPwr = magnitude * Math.sin(robotAngle) + rightX;
        final double driveBRPwr = magnitude * Math.cos(robotAngle) + rightX;

        robot.driveFL.setPower(driveFLPwr);
        robot.driveFR.setPower(driveFRPwr);
        robot.driveBL.setPower(driveBLPwr);
        robot.driveBR.setPower(driveBRPwr);
    }

    private double[] adapt_Heading(double x, double y, double heading)
    {
        double r = Math.sqrt(x*x + y*y);
        double Θ = Math.asin((y * Math.sin(Math.PI/2))/r);

        Θ += heading * (Math.PI/180);

        double newX = r * Math.cos(Θ);
        double newY = r * Math.sin(Θ);

        return new double[] {newX, newY};
    }
}

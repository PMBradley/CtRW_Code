package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Robot2019 {
    // Motor and servo variables
    public DcMotor driveFL = null;
    public DcMotor driveFR = null;
    public DcMotor driveBL = null;
    public DcMotor driveBR = null;

    public DcMotor motorLift = null;
    public DcMotor motorIntakeL = null;
    public DcMotor motorIntakeR = null;

    public Servo armPivot = null;
    public Servo armGrab = null;
    public Servo intakeDropL = null;
    public Servo intakeDropR = null;
    public Servo pullerDropL = null;
    public Servo pullerDropR = null;

    // Sensor Variables
    public DigitalChannel touchLift0 = null;
    public DigitalChannel touchArm1 = null;
    public DigitalChannel touchBlock2 = null;
    public DigitalChannel touchLiftUp3 = null;
    public DigitalChannel touchLeft4 = null;
    public DigitalChannel touchRight5 = null;
    public DigitalChannel touchBack6 = null;
    public DigitalChannel touchClamp7 = null;

    public Rev2mDistanceSensor flightFront0 = null;
    public Rev2mDistanceSensor flightLeft1 = null;
    public Rev2mDistanceSensor flightRight2 = null;
    public Rev2mDistanceSensor flightBack3 = null;

    public BNO055IMU imu = null;
    public WebcamName webcamName = null;

    // The all important hardware map
    HardwareMap mainMap = null;


    int readRedundancy = 1; // non-boolean sensors check that many times when using the appropreate read function

    // Controller variables - stored here to have access to them in all places with access to this class
    double gp1_lstickX = 0.0;
    double gp1_lstickY =  0.0;
    double gp1_rstickX =  0.0;
    boolean gp1_rbumper = false;
    boolean gp1_lbumper = false;
    boolean gp1_a = false;
    boolean gp1_b = false;
    boolean gp2_a = false;
    double gp1_rtrigger = 0.0;
    double gp1_ltrigger = 0.0;
    double gp2_rtrigger = 0.0;
    double gp2_ltrigger = 0.0;
    boolean gp2_lbumper = false;
    boolean gp2_rbumper = false;
    boolean gp2_x = false;
    boolean gp2_y = false;
    boolean gp1_x = false;
    boolean gp1_y = false;
    boolean gp2_up = false;
    boolean gp2_down = false;

    double fl = 0;
    double fr = 0;
    double bl = 0;
    double br = 0;

    public Robot2019(){ } // constructor

    public void init(HardwareMap hMap){
        mainMap = hMap;


        // Grabbing motors from hardware map
        driveFL = mainMap.get(DcMotor.class, "driveFL");
        driveFR = mainMap.get(DcMotor.class, "driveFR");
        driveBL = mainMap.get(DcMotor.class, "driveBL");
        driveBR = mainMap.get(DcMotor.class, "driveBR");
        motorLift = mainMap.get(DcMotor.class, "motorLift");
        motorIntakeL = mainMap.get(DcMotor.class, "intakeL");
        motorIntakeR = mainMap.get(DcMotor.class, "intakeR");

        // Grabbing servos from hardware map
        armPivot = mainMap.get(Servo.class, "armPivot");
        armGrab = mainMap.get(Servo.class, "armGrab");
        intakeDropL = mainMap.get(Servo.class, "intakeDropL");
        intakeDropR = mainMap.get(Servo.class, "intakeDropR");
        pullerDropL = mainMap.get(Servo.class, "pullerDropL");
        pullerDropR = mainMap.get(Servo.class, "pullerDropR");

        // Grabbing sensors from hardware map
        touchLift0 = mainMap.get(DigitalChannel.class, "touchLift0");
        touchArm1 = mainMap.get(DigitalChannel.class, "touchArm1");
        touchBlock2 = mainMap.get(DigitalChannel.class, "touchBlock2");
        touchLiftUp3 = mainMap.get(DigitalChannel.class, "touchLiftUp3");
        touchLeft4 = mainMap.get(DigitalChannel.class, "touchLeft4");
        touchRight5 = mainMap.get(DigitalChannel.class, "touchRight5");
        touchBack6 = mainMap.get(DigitalChannel.class, "touchBack6");
        touchClamp7 = mainMap.get(DigitalChannel.class, "touchClamp7");

        flightFront0 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightFront0");
        flightLeft1  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightLeft1");
        flightRight2 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightRight2");
        flightBack3  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightBack3");

        imu = mainMap.get(BNO055IMU.class, "imu");

        //webcamName = mainMap.get(WebcamName.class, "Webcam 1"); // the webcam for webcam things

        armPivot.setPosition(0.69);

        // Set up the Internal Measurement Unit
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "DroopyIMU.json"; // see the calibration sample opmode
        parameters.loggingEnabled      =  true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    // Utility functions
    public double readFlight(DistanceSensor inFlight){
        double[] readings = new double[readRedundancy];
        double output = 0.0;

        for(int i = 0; i < readRedundancy; i++){
            readings[i] = inFlight.getDistance(DistanceUnit.CM);
        }

        for(int i = 0; i < readRedundancy; i++){ // addition part of averaging
            output += readings[i];
        }
        output /= readRedundancy; // division part of averaging

        return (output);
    }

    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}


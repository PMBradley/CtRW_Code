package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


public class Robot2019 {
    // Motor and servo variables
    public DcMotor driveFL = null; //
    public DcMotor driveFR = null;
    public DcMotor driveBL = null;
    public DcMotor driveBR = null;

    public DcMotor motorLift = null;
    public DcMotor motorIntakeL = null;
    public DcMotor motorIntakeR = null;

    public Servo armPivot = null;
    public Servo armGrab = null;

    public DigitalChannel touchLift0 = null;
   /* public DigitalChannel touchArm1 = null;
    public DigitalChannel touchBlock2 = null;
    public DigitalChannel touchFront3 = null;
    public DigitalChannel touchLeft4 = null;
    public DigitalChannel touchRight5 = null;
    public DigitalChannel touchBack6 = null;
    public DigitalChannel touchSensor7 = null;

    public DistanceSensor flightFront0 = null;
    public DistanceSensor flightLeft1 = null;
    public DistanceSensor flightRight2 = null;
    public DistanceSensor flightBack3 = null;*/

    HardwareMap mainMap = null;

    //Vision variables
   // WebcamName webcam = null;


    public Robot2019(){


    } // constructor

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
        armPivot = mainMap.get(Servo.class, "armPivot");
        armGrab = mainMap.get(Servo.class, "armGrab");

        // Grabbing sensors from hardware map
       // touchLift0 = mainMap.get(DigitalChannel.class, "touchLift0");
    /*    touchArm1 = mainMap.get(DigitalChannel.class, "touchArm1");
        touchBlock2 = mainMap.get(DigitalChannel.class, "touchBlock2");
        touchFront3 = mainMap.get(DigitalChannel.class, "touchFront3");
        touchLeft4 = mainMap.get(DigitalChannel.class, "touchLeft4");
        touchRight5 = mainMap.get(DigitalChannel.class, "touchRight5");
        touchBack6 = mainMap.get(DigitalChannel.class, "touchBack6");

        flightFront0 = mainMap.get(DistanceSensor.class, "flightFront0");
        flightLeft1 = mainMap.get(DistanceSensor.class, "flightLeft1");
        flightRight2 = mainMap.get(DistanceSensor.class, "flightRight2");
        flightBack3 = mainMap.get(DistanceSensor.class, "flightBack3");

        webcam = mainMap.get(WebcamName.class, "webcam");

        // Setup of time of flight sensors
        Rev2mDistanceSensor flight0 = (Rev2mDistanceSensor)flightFront0;
        Rev2mDistanceSensor flight1 = (Rev2mDistanceSensor)flightLeft1;
        Rev2mDistanceSensor flight2 = (Rev2mDistanceSensor)flightRight2;
        Rev2mDistanceSensor flight3 = (Rev2mDistanceSensor)flightBack3;
*/

    }
}


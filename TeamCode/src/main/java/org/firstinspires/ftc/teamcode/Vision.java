package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.List;

import static android.R.attr.angle;
import static android.R.attr.targetName;
import static android.R.attr.track;
import static android.view.View.X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Vision {
    public int MAX_TARGETS = 4;
    private VuforiaTrackables targets;
    private VuforiaTrackables  targetsSkyStone;
    static OpenGLMatrix targetOrientation;
    static OpenGLMatrix robotFromCamera;
    private boolean targetFound = false;
    private String targetString = "NULL";
    public String trackableString = "NULL";
    public boolean targetVisible = false;
    private OpenGLMatrix lastLocation = null;

    private static final float mmPerInch        = 25.4f;
    private float cmPerInch = 2.54f;

    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


    /*

    private VuforiaTrackable targetBLP;
    private VuforiaTrackable targetRDP;
    private VuforiaTrackable targetFRP;
    private VuforiaTrackable targetBKP;

    */


    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    //Setup Vuforia Trackers for each marker
    /*
    private VuforiaTrackableDefaultListener listenerBLP;
    private VuforiaTrackableDefaultListener listenerRDP;
    private VuforiaTrackableDefaultListener listenerFRP;
    private VuforiaTrackableDefaultListener listenerBKP;
   */


    private VuforiaTrackableDefaultListener listenerStoneTarget;
    private VuforiaTrackableDefaultListener listenerblueRearBridge;
    private VuforiaTrackableDefaultListener listenerredRearBridge;
    private VuforiaTrackableDefaultListener listenerredFrontBridge;
    private VuforiaTrackableDefaultListener listenerblueFrontBridge;
    private VuforiaTrackableDefaultListener listenerred1;
    private VuforiaTrackableDefaultListener listenerred2;
    private VuforiaTrackableDefaultListener listenerfront1;
    private VuforiaTrackableDefaultListener listenerfront2;
    private VuforiaTrackableDefaultListener listenerblue1;
    private VuforiaTrackableDefaultListener listenerblue2;
    private VuforiaTrackableDefaultListener listenerrear1;
    private VuforiaTrackableDefaultListener listenerrear2;

    //When called activates tracking for targets
    public void activateTracking(){
        if(targetsSkyStone != null){
            targetsSkyStone.activate();
        }
    }

    public void deactivateTracking(){
        if(targetsSkyStone != null){
            targetsSkyStone.deactivate();


        }
    }

    private Robot2019 robot;

    public Vision(Robot2019 robot)
    {
        this.robot = robot;
    }


    public void initVuforia(){

        File captureDirectory = AppUtil.ROBOT_DATA_DIR;
        VuforiaLocalizer vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(robot.viewId);

        parameters.cameraName = robot.webcamName;

        parameters.vuforiaLicenseKey = "AaDvMLn/////AAABmTZQLjsUR0whgqv7jiVLfwZ3LemI+CeNKa3ByyqPZpIM0rZtqUnW7cA2uF5XIGnJJFZ2MQol1ERxZLZeMl/mZuI2BlWuD42PO2Q3yIeB8bCtLAHXSFGE6ZM3XHOU9KDtzyzgkHBUEPF1Pw1nzLM8r2PW4JQFPOPMqYYFKOjZfHjLhgeY6n6x45L8RziscD7jLSzGVqvJgF5uVK2Xi1IZr/8MH+W6vMBjE4EUuhOZNyevM8XjhYkO50xq03ETi/yKdIDL8U+ef31+HS/kPZp6v4N1zNQFIx2geUwps9cVIftQxw3EXhrmbcGrLCB9rmHNUHVyrq/scKWvj+bCfaPn/zA554wZ+vpBI/2UDInghC4p";

        vuforia  = ClassFactory.getInstance().createVuforia(parameters);

        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        vuforia.enableConvertFrameToBitmap();


        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
        /*
        targets = vuforia.loadTrackablesFromAsset("SkyStone");


        targetBLP = targets.get(0);
        targetBLP.setName("BlueAlliance");

        targetRDP = targets.get(1);
        targetRDP.setName("RedAlliance");

        targetFRP = targets.get(2);
        targetFRP.setName("FrontWall");

        targetBKP = targets.get(3);
        targetBKP.setName("BackWall");
        */

        // Trackables
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
      /*  VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge"); */
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables.addAll(targetsSkyStone);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels




        targetOrientation = OpenGLMatrix
                .translation(0, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES,
                        90,
                        0 ,
                        -90
                ));



        robotFromCamera = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZY,
                        AngleUnit.DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            trackable.setLocation(targetOrientation);
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        /*Setup Vuforia Trackers for each marker
        listenerBLP = (VuforiaTrackableDefaultListener)targetBLP.getListener();
        listenerRDP = (VuforiaTrackableDefaultListener)targetRDP.getListener();
        listenerFRP = (VuforiaTrackableDefaultListener)targetFRP.getListener();
        listenerBKP = (VuforiaTrackableDefaultListener)targetBKP.getListener();
        */

        listenerStoneTarget = (VuforiaTrackableDefaultListener)stoneTarget.getListener();
        /*
        listenerblue1 = (VuforiaTrackableDefaultListener)blue1.getListener();
        listenerblue2 = (VuforiaTrackableDefaultListener)blue2.getListener();
        listenerblueFrontBridge = (VuforiaTrackableDefaultListener)blueFrontBridge.getListener();
        listenerblueRearBridge = (VuforiaTrackableDefaultListener)blueRearBridge.getListener();
        listenerfront1 = (VuforiaTrackableDefaultListener)front1.getListener();
        listenerfront2 = (VuforiaTrackableDefaultListener)front2.getListener();
        listenerrear1 = (VuforiaTrackableDefaultListener)rear1.getListener();
        listenerrear2 = (VuforiaTrackableDefaultListener)rear2.getListener();
        listenerredFrontBridge = (VuforiaTrackableDefaultListener)redFrontBridge.getListener();
        listenerredRearBridge = (VuforiaTrackableDefaultListener)redRearBridge.getListener();
        listenerred1 = (VuforiaTrackableDefaultListener)red1.getListener();
        listenerred2 = (VuforiaTrackableDefaultListener)red2.getListener();
        */
    }

    public String targetsAreVisible(){

        float xTranslation = 0;
        float yTranslation = 0;
        float zTranslation = 0;

        targetVisible = false;
        trackableString = "NULL";

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                VectorF translation = robotLocationTransform.getTranslation();
                trackableString = trackable.getName();
                if(translation != null){
                    xTranslation = translation.get(0) / cmPerInch;
                    yTranslation = translation.get(1) / cmPerInch;
                    zTranslation = translation.get(2) / cmPerInch;
                }


                targetString = "Visible Target: " + trackableString + " Pos (in) {X, Y, Z} = "
                        + (xTranslation) + " " + (yTranslation) + " "
                        + (zTranslation);


                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if(targetString == ""){
            targetString = "NULL";
        }


        return targetString;
    }

    public String getVisibleTarget(){

        float xTranslation;
        float yTranslation;
        float zTranslation;
        trackableString = "NULL";
        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                VectorF translation = robotLocationTransform.getTranslation();
                trackableString = trackable.getName();
             //   xTranslation = translation.get(0) / mmPerInch;
             //   yTranslation = translation.get(1) / mmPerInch;
             //   zTranslation = translation.get(2) / mmPerInch;


                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }




        return trackableString;
    }

    public double[] getTranslation(){

        float xTranslation = 0;
        float yTranslation = 0;
        float zTranslation = 0;

        targetVisible = false;


        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                VectorF translation = robotLocationTransform.getTranslation();
                trackableString = trackable.getName();
                if(translation != null){
                    xTranslation = translation.get(0) / cmPerInch;
                    yTranslation = translation.get(1) / cmPerInch;
                    zTranslation = translation.get(2) / cmPerInch;
                }



                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }


        double[] output = {xTranslation, yTranslation, zTranslation};

        return output;
    }



}

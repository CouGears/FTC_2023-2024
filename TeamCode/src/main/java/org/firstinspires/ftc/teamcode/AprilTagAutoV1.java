package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous


public class AprilTagAutoV1 {

    //******************OUR VARS******************
    private static final double rev = 537.7;
    private static final double inch = rev / (3.78 * 3.14);
    private static final double feet = inch * 12 + (10 * inch);
    private ElapsedTime runtime = new ElapsedTime();
    AutonMethods robot = new AutonMethods();
    //******************FTC EXAMPLE VARS******************
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    @Override

    public void init() {
        robot.init(hardwareMap, telemetry, true);
        telemetry.addData("Status", "Initialized");
        initAprilTag();
        if (USE_WEBCAM) setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        switch (robot.counter) {
            case 0://Finding Tag
                while (targetFound == false) {
                    robot.turn(10);
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) && ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                            targetFound = true;
                            desiredTag = detection;
                        }
                    }
                }
                robot.counter++;
                break;
            case 1: //Turning to tag
                double bearingInRads =  Math.toRadians(desiredTag.ftcPose.bearing);
                double strafeDistance = Math.cos(bearingInRads) * desiredTag.ftcPose.range;
                for (int i = 0; i < 4; i++) {
                    robot.drive(0, strafeDistance/4, .5);
                    robot.turn(desiredTag.ftcPose.bearing);
                }
                robot.counter++;
                break;
            case 2: //
                robot.drive( desiredTag.ftcPose.range - 12, 0,1);
                robot.counter++;
                break;
        }
    }


    //**********************FTC EXAMPLE APRILTAG FUNCTIONS**********************
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    private void scanAprilTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData(">","Drive using joysticks to find valid target\n");
        }
    }
}



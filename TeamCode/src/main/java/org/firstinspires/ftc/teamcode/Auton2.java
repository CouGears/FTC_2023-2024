/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.autoncamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutonMethods;
import org.firstinspires.ftc.teamcode.autoncamera.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous


public class Auton2 extends LinearOpMode {
    AutonMethods robot = new AutonMethods();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double rev = 537.7; //312 rpm motor
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    int position = 1;
    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;


    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Status", "Initialized");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (isStopRequested() || !opModeIsActive()) return;


        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            // Park Left
            position = 1;
        } else if (tagOfInterest.id == MIDDLE) {
            // Park Middle
            position = 2;
        } else if (tagOfInterest.id == RIGHT) {
            // Park Right
            position = 3;
        }


        if (isStopRequested()) return;
        // Start is now pressed!


        // FSM
        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We essentially define the flow of the state machine through this switch statement
            switch (robot.counter) {
                //TODO: Write your Auton HERE
                case 0:
                    telemetry.addData(String.valueOf(position), "");
                    telemetry.update();
                    if (position == 1) {
                        telemetry.addData("Park Location", "Left");
                        telemetry.update();
                    } else if (position == 2) {
                        telemetry.addData("Park Location", "Middle");
                        telemetry.update();
                    } else if (position == 3) {
                        telemetry.addData("Park Location", "Right");
                        telemetry.update();
                    }
                    robot.counter++;
                    break;
                case 1:
                    robot.drive(.1 * feet, 0, 0.5);
                    robot.counter++;
                    break;
                case 2:
                    robot.drive(0, .5 * feet, .5);
                    robot.counter++;
                    break;
                case 3:
                    robot.drive(1.5 * feet, 0, .5);
                    //robot.sleep(100);
                    robot.counter++;
                    break;
                case 4:
                    //robot.turn(-45);
                    robot.counter++;
                    break;
                case 5:
                    //lift
                    //robot.lift(7475);
                    robot.counter++;
                    break;
                case 6:
                    //dump
                    //robot.dump();
                    robot.counter++;
                    break;
                case 7:
                    // robot.turn(45);
                    robot.counter++;
                    break;
                case 8:
                    // robot.lift(0);
                    robot.counter++;
                    break;
                case 9:
                    //robot.drive(0,-.25*feet,1);
                    robot.counter++;
                    break;
                case 10:
                    if (position == 1) {
                        robot.drive(0, -2 * feet, .5);
                    } else if (position == 2) {
                        robot.drive(0, 0 * feet, .5);
                    } else if (position == 3) {
                        robot.drive(0, 2 * feet, 0.5);
                    }
                    robot.counter++;
                    break;
                case 11:
                    camera.stopStreaming();
                    robot.counter++;
                    break;
            }

            // Things to keep running
            // We update drive continuously in the background, regardless of state


        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        position = detection.id;
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
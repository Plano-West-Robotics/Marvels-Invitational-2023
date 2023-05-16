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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class SleeveDetector {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double METERS_PER_INCH = 25.4/1000;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 2 * METERS_PER_INCH;

    static int ID_TAG_ZONE_ONE = 6;
    static int ID_TAG_ZONE_TWO = 7;
    static int ID_TAG_ZONE_THREE = 8;

    AprilTagDetection tagOfInterest = null;

    OpMode opMode;
    SignalZone detectedZone;

    public SleeveDetector(OpMode opMode) {
        this.opMode = opMode;
        // just a default
        this.detectedZone = SignalZone.ZoneTwo;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        opMode.telemetry.setMsTransmissionInterval(50);
    }

    public SignalZone update() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        boolean tagFound = false;

        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == ID_TAG_ZONE_ONE) {
                detectedZone = SignalZone.ZoneOne;
            } else if (tag.id == ID_TAG_ZONE_TWO) {
                detectedZone = SignalZone.ZoneTwo;
            } else if (tag.id == ID_TAG_ZONE_THREE) {
                detectedZone = SignalZone.ZoneThree;
            } else {
                continue;
            }

            // the continue wasn't reached, therefore we found something
            tagOfInterest = tag;
            tagFound = true;
            break;
        }

        this.opMode.telemetry.addData("Zone", this.detectedZone);

        if (tagFound) {
            this.opMode.telemetry.addLine("Detected tag " + tagOfInterest.id + " corresponding to zone " + this.detectedZone);
            this.opMode.telemetry.addLine("Location data:");
            tagToTelemetry(tagOfInterest);
        } else {
            if (tagOfInterest == null) {
                this.opMode.telemetry.addLine("No tags have been detected so far, using the default zone");
            } else {
                this.opMode.telemetry.addLine("Unable to detect tag, last detection is tag " + tagOfInterest.id + " corresponding to zone " + this.detectedZone);
                this.opMode.telemetry.addLine("Location data:");
                tagToTelemetry(tagOfInterest);
            }
        }

        this.opMode.telemetry.update();

        return this.detectedZone;
    }

    void tagToTelemetry(AprilTagDetection detection) {
        this.opMode.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        this.opMode.telemetry.addLine(String.format("Translation X: %.2f metres", detection.pose.x));
        this.opMode.telemetry.addLine(String.format("Translation Y: %.2f metres", detection.pose.y));
        this.opMode.telemetry.addLine(String.format("Translation Z: %.2f metres", detection.pose.z));
        this.opMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        this.opMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        this.opMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public enum SignalZone {
        ZoneOne, ZoneTwo, ZoneThree
    }
}
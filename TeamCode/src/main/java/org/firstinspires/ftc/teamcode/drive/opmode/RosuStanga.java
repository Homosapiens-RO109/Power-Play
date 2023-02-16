package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm_control.ArmControl;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Date;

@Config
@Autonomous(group = "redparcare", name = "Rosu Parcare Stanga")
public class RosuStanga extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
    double fx = 821.993;
    double fy = 828.993;
    double cx = 330.489;
    double cy = 248.997;

    // UNITS ARE METERS
    double tagsize = 0.166;

    final int FIRST_ID_TAG_OF_INTEREST = 1; // 36h11 family
    final int SECOND_ID_TAG_OF_INTEREST = 0;
    final int THIRD_ID_TAG_OF_INTEREST = 9;

    AprilTagDetection tagOfInterest = null;

    public int aprilTagMagic(Telemetry t) {
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            t.addData("Found", currentDetections.size());
            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    t.addData(String.valueOf(tag.id), "FOUND");

                    if (tag.id == FIRST_ID_TAG_OF_INTEREST || tag.id == SECOND_ID_TAG_OF_INTEREST || tag.id == THIRD_ID_TAG_OF_INTEREST) {
                        tagOfInterest = tag;
                        t.addData("Tag", tagOfInterest.id);
                        t.update(); // It only took us 20 minutes!
                                    // That was a lie!
                        return tagOfInterest.id;
                    }
                }
            }
            t.update();
        }
        return -1;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAcc = SampleMecanumDrive.getAccelerationConstraint(15);

        ArmControl.init(hardwareMap, telemetry);

        //CAMERA SETUP
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);

        final int[] aprilTag = new int[1];

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { // old:800 x 448
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                aprilTag[0] = aprilTagMagic(telemetry);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        final long sightStartSeeing = new Date().getTime();
        aprilTag[0] = aprilTagMagic(telemetry);
        telemetry.addData("Saw tag in (ms)", new Date().getTime() - sightStartSeeing);
        final long startTime = new Date().getTime();

        TrajectorySequence putPreload = drive.trajectorySequenceBuilder(new Pose2d(-35.40, -65.50, Math.toRadians(90.00)))
                .UNSTABLE_addTemporalMarkerOffset(3.06,() -> {
                    ArmControl.setArmLevel(ArmControl.Levels.THIRD);
                })
                .UNSTABLE_addTemporalMarkerOffset(4.00,() -> {
                    ArmControl.openClip();
                    ArmControl.setArmLevel(ArmControl.Levels.DOWN);
                })
                .lineToSplineHeading(new Pose2d(-11.61, -59.92, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-11.43, -12.36, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-23.50, -8.64, Math.toRadians(90.00)))
                .build();
        drive.setPoseEstimate(putPreload.start());

        switch (aprilTag[0]) {
            case FIRST_ID_TAG_OF_INTEREST: {
                // TODO: Code here
                break;
            }
            case SECOND_ID_TAG_OF_INTEREST: {
                // TODO: More code here
                int sugiPula = 69;
                break;
            }
            case THIRD_ID_TAG_OF_INTEREST: {
                // TODO: Even more code here
                int inteliSense = 0;
                break;
            }
        }

        long secondsTillParsed = new Date().getTime() - startTime;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Created path in (ms)", secondsTillParsed);
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            ArmControl.closeClip();
            ArmControl.setArmLevel(ArmControl.Levels.DOWN);
            drive.followTrajectorySequence(putPreload);
        }
    }
}

//            t = drive.trajectorySequenceBuilder(new Pose2d(-36.05, -64.62, Math.toRadians(90.00)))
//                    .addTemporalMarker(() -> {
//                        ArmControl.closeClip();
//                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//                    })
//                    .lineToSplineHeading(new Pose2d(-11.71, -60.75, Math.toRadians(90.00)))
//                    .lineToSplineHeading(new Pose2d(-12.45, -11.34, Math.toRadians(90.00)))
//                    .addTemporalMarker(() -> {
//                        ArmControl.setArmLevel(ArmControl.Levels.THIRD);
//                    })
//                    .waitSeconds(1.69)
//                    .lineToSplineHeading(new Pose2d(-23.88, -9.31, Math.toRadians(90.00)))
//                    .addTemporalMarker(() ->  {
//                        ArmControl.openClip();
//                    })
//                    .addTemporalMarker(() -> {
////                        ArmControl.closeClip();
//                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//                    })
//
//                    .build();

//
////        TrajectorySequence incercare1 = drive.trajectorySequenceBuilder(new Pose2d(-36.78, -64.81, Math.toRadians(90.00)))
////                .forward(5)
////                .strafeRight(24)
////                .forward(48)
////                .strafeLeft(10.8)
////                .build();
//
//        TrajectorySequence incercare1 = drive.trajectorySequenceBuilder(new Pose2d(-36.51, -65.31, Math.toRadians(90.00)))
//                .lineTo(new Vector2d(-12.17, -62.52))
//                .lineToSplineHeading(new Pose2d(-12.91, -14.59, Math.toRadians(90.00)))
//                .lineToSplineHeading(new Pose2d(-23.50, -9.57, Math.toRadians(90.00)))
//                .build();
//
////        drive.trajectorySequenceBuilder(new Pose2d(-36.05, -64.62, Math.toRadians(90.00)))
////                .lineToSplineHeading(new Pose2d(-36.23, -14.47, Math.toRadians(90.00)))
////                .lineToSplineHeading(new Pose2d(-23.88, -8.94, Math.toRadians(90.00)))
////                .build();
//
////                .lineTo(new Vector2d(-36.78, -57.80))
////                .lineTo(new Vector2d(-12.63, -57.25))
////                .lineTo(new Vector2d(-12.63, -9.86))
////                .lineTo(new Vector2d(-24.00, -9.13))

//            t = drive.trajectorySequenceBuilder(new Pose2d(-36.51, -65.31, Math.toRadians(90.00)))
//                    .addTemporalMarker(() -> {
//                        ArmControl.closeClip();
//                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//                    })
//                    .lineTo(new Vector2d(-12.17, -62.52))
//                    .lineToSplineHeading(new Pose2d(-12.91, -14.59, Math.toRadians(90.00)))
//                    .lineToSplineHeading(new Pose2d(-23.50, -9.57, Math.toRadians(90.00)))
//                    .addTemporalMarker(() -> {
//                        ArmControl.setArmLevel(ArmControl.Levels.THIRD);
//                    })
//                    .waitSeconds(1.69)
//                    .addTemporalMarker(() ->  {
//                        ArmControl.openClip();
//                    })
//                    .waitSeconds(.69)
//                    .addTemporalMarker(() -> {
//                        ArmControl.closeClip();
//                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//                    })
//                    .build();
//                    .addTemporalMarker(() -> {
//                        ArmControl.closeClip();
//                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//                    })`
//                        ArmControl.closeClip();

/*
// SOMEWHAT WORKING CODE FOR PRELOAD
        if (aprilTag[0] == FIRST_ID_TAG_OF_INTEREST) {
            t = drive.trajectorySequenceBuilder(new Pose2d(-36.05, -64.62, Math.toRadians(90.00)))
                    .lineToSplineHeading(new Pose2d(-11.71, -60.75, Math.toRadians(90.00))).lineToSplineHeading(new Pose2d(-12.45, -11.34, Math.toRadians(90.00))).addTemporalMarker(() -> {
                        ArmControl.setArmLevel(ArmControl.Levels.THIRD);
                    }).waitSeconds(1).lineToSplineHeading(new Pose2d(-23.88, -9.31, Math.toRadians(90.00))).addTemporalMarker(() -> {
                        ArmControl.setArmLevel(ArmControl.Levels.DOWN);
                    }).lineToSplineHeading(new Pose2d(-46.74, -12.81, Math.toRadians(180.00))).addTemporalMarker(() -> {
                        ArmControl.setArmLevel(ArmControl.Levels.FIRST);
                        ArmControl.openClip();
                    }).splineTo(new Vector2d(-59.83, -12.45), Math.toRadians(180.00)).addTemporalMarker(() -> {
                        ArmControl.closeClip();
                    }).build();

        }
* */
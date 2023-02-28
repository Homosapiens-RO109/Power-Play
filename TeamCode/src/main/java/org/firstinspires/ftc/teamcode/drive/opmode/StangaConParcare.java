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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

@Config
@Autonomous(group = "left", name = "Partea Stanga Con + Parcare", preselectTeleOp = "Homosapiens TeleOP FSM")
public class StangaConParcare extends LinearOpMode {

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
    double tagsize = 0.035;

    final int FIRST_ID_TAG_OF_INTEREST = 1; // 36h11 family
    final int SECOND_ID_TAG_OF_INTEREST = 2;
    final int THIRD_ID_TAG_OF_INTEREST = 0;

    AprilTagDetection tagOfInterest = null;

    public int aprilTagMagic(Telemetry t) {
        // This has been fixed.
        Date startTime = new Date();
        while (!isStopRequested() && (new Date().getTime() - startTime.getTime()) < 15_000) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            t.addData("Found", currentDetections.size());
            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == FIRST_ID_TAG_OF_INTEREST || tag.id == SECOND_ID_TAG_OF_INTEREST || tag.id == THIRD_ID_TAG_OF_INTEREST) {
                        t.addData("Relevant tag", tag.id);
                        t.update();
                        return tag.id;
                    } else {
                        t.addData("Found useless tag", String.valueOf(tag.id));
                        t.update();
                    }
                }
            }
            t.update();
        }
        t.addLine("Had to use default tag");
        t.update();
        return SECOND_ID_TAG_OF_INTEREST;
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
                telemetry.addLine("Camera is up.");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera has errored with code.", errorCode);
            }
        });

        waitForStart();

        final long sightStartSeeing = new Date().getTime();
//        try {
//            aprilTag[0] = aprilTagMagic(telemetry);
//        } catch (Exception e) {
//            aprilTag[0] = SECOND_ID_TAG_OF_INTEREST;
//            telemetry.addLine("Failed to set proper tag :(");
//            telemetry.addLine("Defaulted to middle choice");
//        }

        aprilTag[0] = SECOND_ID_TAG_OF_INTEREST;

        // format the following MS to a human readable format
//        SimpleDateFormat sdf = new SimpleDateFormat("ss.SSS");
//        String parsed = sdf.format(new Date().getTime() - sightStartSeeing);

        // TODO: eventually have this all nicely parsed out since I can't be arsed currently
        telemetry.addData("Saw tag in (ms)", new Date().getTime() - sightStartSeeing);
        telemetry.addData("Tag", aprilTag[0]);
        final long startTime = new Date().getTime();

        Pose2d startPose = new Pose2d(-35.40, -65.50, Math.toRadians(90.00));
        TrajectorySequenceBuilder putPreload = drive.trajectorySequenceBuilder(new Pose2d(-35.40, -65.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-11.61, -59.92, Math.toRadians(90.00))) // first strafe
                .lineToSplineHeading(new Pose2d(-11.43, -12.36, Math.toRadians(90.00))) // goes forward
                .addDisplacementMarker(() -> {
                    ArmControl.setArmLevel(ArmControl.Levels.THIRD);
                })
                .lineToSplineHeading(new Pose2d(-24.4, -8.4, Math.toRadians(90.00))) // strafes left to get to the JB
                .waitSeconds(0.8)
                .addTemporalMarker(() -> {
                    ArmControl.openClip();
                })
                .waitSeconds(0.8)
//                .lineToSplineHeading(new Pose2d(-36.14, -19.42, Math.toRadians(90.00))) // goes more left
                .addDisplacementMarker(() -> {
                    ArmControl.setArmLevel(ArmControl.Levels.DOWN);
                });
//                .lineToSplineHeading(new Pose2d(-35.95, -36.51, Math.toRadians(90.00))); // goes backwards in the middle lane;
        drive.setPoseEstimate(startPose);
        Pose2d endPose = new Pose2d(-35.95, -36.51, Math.toRadians(90.00));

        TrajectorySequence finalPath = null;

//        switch (aprilTag[0]) {
//            case FIRST_ID_TAG_OF_INTEREST: {
//                // you strafe left
////                park = drive.trajectorySequenceBuilder(putPreload.end())
//                finalPath = putPreload
//                        .lineToSplineHeading(new Pose2d(-61.31, -36.60, Math.toRadians(90.00)))
//                        .build();
//                break;
//            }
////            This in theory should go straight to default and do nothing
//            case SECOND_ID_TAG_OF_INTEREST: {
//                // You do nothing
//                finalPath = putPreload.forward(0.1).build();
//                break;
//            }
//            case THIRD_ID_TAG_OF_INTEREST: {
//                // You strafe right
//                finalPath = putPreload
//                        .lineToSplineHeading(new Pose2d(-11.71, -36.41, Math.toRadians(90.00)))
//                        .build();
//                break;
//            }
//            default: {
//                // You do nothing
//                finalPath = putPreload.forward(0.01).build();
//                break;
//            }
//        }

        finalPath = putPreload.build();

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
            ArmControl.setArmLevel(ArmControl.Levels.FIRST);
            drive.followTrajectorySequence(finalPath);
        }
    }
}

//        TrajectorySequence takeCone = drive.trajectorySequenceBuilder(new Pose2d(-36.14, -19.42, Math.toRadians(90.00)))
//                .UNSTABLE_addTemporalMarkerOffset(1.47,() -> {})
//                .UNSTABLE_addTemporalMarkerOffset(3.96,() -> {})
//                .UNSTABLE_addTemporalMarkerOffset(3.12,() -> {})
//
//                .splineTo(new Vector2d(-60.20, -12.63), Math.toRadians(180.00))
//                .lineTo(new Vector2d(-23.69, -9.86))
//                .lineTo(new Vector2d(-36.41, -9.86))
//                .lineToSplineHeading(new Pose2d(-35.95, -36.51, Math.toRadians(87.75)))
//                .build();


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
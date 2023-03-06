package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
@Autonomous(group = "right", name = "coada calului 2.0", preselectTeleOp = "Homosapiens TeleOP FSM")
public class DreaptaNou extends LinearOpMode {

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
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                telemetry.addLine("Camera is up.");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera has crashed with code.", errorCode);
            }
        });

        // Closes to clip so we make sure the preload stays put
        ArmControl.closeClip();

        waitForStart();

        final long sightStartSeeing = new Date().getTime();
        try {
            aprilTag[0] = aprilTagMagic(telemetry);
        } catch (Exception e) {
            aprilTag[0] = SECOND_ID_TAG_OF_INTEREST;
            telemetry.addLine("Failed to set proper tag :(");
            telemetry.addLine("Defaulted to middle choice");
        }

        telemetry.addData("Saw tag in (ms)", new Date().getTime() - sightStartSeeing);
        telemetry.addData("Tag", aprilTag[0]);
        final long startTime = new Date().getTime();

        Pose2d start = new Pose2d(35, -65, Math.toRadians(90));
        drive.setPoseEstimate(start);

        TrajectorySequence Puncte_multe = drive.trajectorySequenceBuilder(start)


                .lineToConstantHeading(new Vector2d(35,-25),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30)) // pana aici merge doar inainte
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {ArmControl.closeClip();})

                .lineToLinearHeading(new Pose2d(28, -4, Math.toRadians(139)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> { ArmControl.setArmLevel(ArmControl.Levels.THIRD);})
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> { ArmControl.openClip(); })
                // aici pune preload

                .lineToLinearHeading(new Pose2d(35, -16, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)) // se intoarce in mijlocu patratului unde e preloadul
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> { ArmControl.setArmLevel(ArmControl.Levels.CONE_5);})

                .lineToLinearHeading(new Pose2d(64, -12, Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> { ArmControl.closeClip();}) // prinde conul 5

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {ArmControl.setArmLevel(ArmControl.Levels.THIRD);})

                .lineToLinearHeading(new Pose2d(32, -16, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .lineToLinearHeading(new Pose2d(28, -4, Math.toRadians(142)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {ArmControl.openClip();}) // pune conul 5

                .lineToLinearHeading(new Pose2d(35, -16, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> { ArmControl.setArmLevel(ArmControl.Levels.CONE_4);})
//
//                .lineToLinearHeading(new Pose2d(32, -16, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(30))
//
//                .lineToLinearHeading(new Pose2d(64, -12, Math.toRadians(0)),
//                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(30))
//
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {ArmControl.closeClip();})
//
//                .waitSeconds(0.5)
//                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {ArmControl.setArmLevel(ArmControl.Levels.THIRD);})
//
//                .lineToLinearHeading(new Pose2d(32, -16, Math.toRadians(90)),
//                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(30))
//
//                .lineToLinearHeading(new Pose2d(28, -5, Math.toRadians(139)),
//                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(35))
//                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {ArmControl.openClip();})
                .lineToLinearHeading(new Pose2d(35, -35, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {ArmControl.setArmLevel(ArmControl.Levels.DOWN);}) // sunt parcat pe 2 aici
                //dupa va parcati

                .build();


        double xPark = 36.01;
        double yPark = -36.01;

        switch (aprilTag[0]) {
            case FIRST_ID_TAG_OF_INTEREST: {
                xPark = 13.0;
                break;
            }
            case THIRD_ID_TAG_OF_INTEREST: {
                xPark = 60.57;
                break;
            }
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(Puncte_multe.end())
                .lineTo(new Vector2d(36.00, -36.00))
                .lineTo(new Vector2d(xPark, yPark))
                .waitSeconds(8.0)
                .build();

        drive.setPoseEstimate(Puncte_multe.start());

        long secondsTillParsed = new Date().getTime() - startTime;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Created path in (ms)", secondsTillParsed);
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            ArmControl.setArmLevel(ArmControl.Levels.CONE_5);
            drive.followTrajectorySequence(Puncte_multe);
            drive.followTrajectorySequence(park);
        }
    }
}
/*

        TrajectorySequence putPreload = drive.trajectorySequenceBuilder(start)
                .forward(50)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {ArmControl.setArmLevel(ArmControl.Levels.SECOND);})
                .back(10)
                .turn(Math.toRadians(90))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {ArmControl.openClip();})
                .back(7)
                .turn(Math.toRadians(-92))
                .forward(13)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {ArmControl.setArmLevel(ArmControl.Levels.CONE_5);})
                .turn(Math.toRadians(-90))
                .forward(29)
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {ArmControl.closeClip();})
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () ->{ArmControl.setArmLevel(ArmControl.Levels.THIRD);})
                .back(29)
                .turn(Math.toRadians(145))
                .forward(11)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {ArmControl.openClip();})
                .back(11)
                .turn(Math.toRadians(-50))
                .back(25)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {ArmControl.setArmLevel(ArmControl.Levels.DOWN);})
                .build();


        double xPark = 36.01;
        double yPark = -36.01;

        switch (aprilTag[0]) {
            case FIRST_ID_TAG_OF_INTEREST: {
                xPark = 60.57;
                break;
            }
            case THIRD_ID_TAG_OF_INTEREST: {
                xPark = 13.0;
                break;
            }
        }

        TrajectorySequence park = drive.trajectorySequenceBuilder(putPreload.end())
//                .lineTo(new Vector2d(36.00, -11.00))
                .lineTo(new Vector2d(36.00, -36.00))
                .lineTo(new Vector2d(xPark, yPark))
                .waitSeconds(8.0)
                .build();
*/
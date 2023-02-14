package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive", name = "Primul Test RoadRunner")
public class TestAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        TrajectoryVelocityConstraint slowVel = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAcc = SampleMecanumDrive.getAccelerationConstraint(15);

        TrajectorySequence incercare1 =         drive.trajectorySequenceBuilder(new Pose2d(-36.78, -64.81, Math.toRadians(90.00)))
                .forward(5)
                .strafeRight(24)
                .forward(48)
                .strafeLeft(10.8)
//                .lineTo(new Vector2d(-36.78, -57.80))
//                .lineTo(new Vector2d(-12.63, -57.25))
//                .lineTo(new Vector2d(-12.63, -9.86))
//                .lineTo(new Vector2d(-24.00, -9.13))
                .build();

//        drive.trajectorySequenceBuilder(new Pose2d(-36.05, -64.62, Math.toRadians(90.00)))
//                .lineToSplineHeading(new Pose2d(-36.23, -14.47, Math.toRadians(90.00)))
//                .lineToSplineHeading(new Pose2d(-23.88, -8.94, Math.toRadians(90.00)))
//                .build();


        drive.setPoseEstimate(incercare1.start());

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(incercare1);
        }
    }
}
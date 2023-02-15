package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm_control.ArmControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(name = "HomosapiensTeleOPTest", group = "testing")
public class HomosapiensTeleOpTest extends LinearOpMode {
    public double SPEED_MULTIPLIER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmControl.init(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

//        while (!isStopRequested()) {
//            final double triggerMovement = -gamepad1.right_trigger + gamepad1.left_trigger;
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -gamepad1.left_stick_y * SPEED_MULTIPLIER,
//                            -gamepad1.left_stick_x * SPEED_MULTIPLIER,
////                            triggerMovement * SPEED_MULTIPLIER
//                            -gamepad1.right_stick_x * SPEED_MULTIPLIER
//                    )
//            );
//
//            drive.update();
//            // TODO: eventually organise this
//            // muie andrei mosescu cel mai mare fan al meu
//            if (gamepad1.x) {
//                ArmControl.openClip();
//            } else if (gamepad1.square) {
//                ArmControl.closeClip();
//            } else if (gamepad1.left_bumper) {
//                SPEED_MULTIPLIER = 0.4;
//                gamepad1.rumble(0.1, 0.1, 100);
////                gamepad1.rumble(0.1, 0.1, 100);
//            } else if (gamepad1.right_bumper) {
//                SPEED_MULTIPLIER = 0.75;
//                gamepad1.rumble(0.6, 0.6, 200);
////                gamepad1.rumble(0.6, 0.6, 200);
//            } else if (gamepad1.circle) {
//                ArmControl.setArmPower(0.8);
//                telemetry.addData("Arm go", "up");
////            } else if (gamepad1.triangle) {
////                ArmControl.setArmPower(-0.4);
////                telemetry.addData("Arm go", "down");
////            } else {
////                ArmControl.setArmPower(0.0);
////                telemetry.addData("Arm go", "no go");
////            }
//            } else if (gamepad1.dpad_left) {
//                ArmControl.setArmLevel(ArmControl.Levels.FIRST);
//            } else if (gamepad1.dpad_up) {
//                ArmControl.setArmLevel(ArmControl.Levels.SECOND);
//            } else if (gamepad1.dpad_right) {
//                ArmControl.setArmLevel(ArmControl.Levels.THIRD);
//            } else if (gamepad1.dpad_down) {
//                ArmControl.setArmLevel(ArmControl.Levels.DOWN);
//            }
//
//            Pose2d poseEstimate = drive.getPoseEstimate();
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("Servo Pos", ArmControl.clipServo.getPosition());
//            telemetry.addData("Speed multiplier", SPEED_MULTIPLIER);
//            telemetry.addData("Arm 1", ArmControl.armMotor1.getCurrentPosition());
//            telemetry.addData("Arm 2", ArmControl.armMotor2.getCurrentPosition());
//            ArmControl.logArmInfo();
//            telemetry.update();
//        }

        double SPEED = .75;
        while (!isStopRequested()) {
            if (gamepad1.left_bumper) {
                SPEED += .01;
            } else if (gamepad1.right_bumper) {
                SPEED -= .01;
            }
            // emergency stop x
            else if (gamepad1.triangle) {
                SPEED = 0;
            }

            drive.setMotorPowers(SPEED, SPEED, SPEED, SPEED);

            telemetry.addData("Speed", String.valueOf(SPEED));
            telemetry.update();
        }

    }
}

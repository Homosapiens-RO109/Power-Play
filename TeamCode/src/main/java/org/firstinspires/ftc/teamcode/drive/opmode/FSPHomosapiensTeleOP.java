package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

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
@TeleOp(name = "Homosapiens TeleOP FSM", group = "fsm")
public class FSPHomosapiensTeleOP extends LinearOpMode {
    public double SPEED_MULTIPLIER = 0.5;

    public boolean[] keyPressArray = new boolean[69]; // macar asta sa fie nice

    public static enum KEYS {
        DPAD_DOWN(1),
        DPAD_LEFT(2),
        DPAD_UP(3),
        DPAD_RIGHT(4),
        LB(5),
        RB(6),
        X(7),
        SQUARE(8),
        TRIANGLE(9),
        CIRCLE(10);

        final int value;

        KEYS(int v) {
            this.value = v;
        }
    }

    public void updateKeysPressed(Gamepad gp) {
        keyPressArray[KEYS.DPAD_DOWN.value] = gp.dpad_down;
        keyPressArray[KEYS.DPAD_LEFT.value] = gp.dpad_left;
        keyPressArray[KEYS.DPAD_UP.value] = gp.dpad_up;
        keyPressArray[KEYS.DPAD_RIGHT.value] = gp.dpad_right;
        keyPressArray[KEYS.LB.value] = gp.left_bumper;
        keyPressArray[KEYS.RB.value] = gp.right_bumper;
        keyPressArray[KEYS.SQUARE.value] = gp.square;
        keyPressArray[KEYS.TRIANGLE.value] = gp.triangle;
        keyPressArray[KEYS.CIRCLE.value] = gp.circle;
        keyPressArray[KEYS.X.value] = gp.x;
    }

    public void handleClipControls() {
        if (keyPressArray[KEYS.SQUARE.value]) {
            ArmControl.openClip();
        } else if (keyPressArray[KEYS.TRIANGLE.value]) {
            ArmControl.closeClip();
        }

        telemetry.addData("Servo Position", ArmControl.clipServo.getPosition());
    }

    public void handleArmControls() {
        if (keyPressArray[KEYS.DPAD_DOWN.value]) {
            ArmControl.setArmLevel(ArmControl.Levels.DOWN);
        } else if (keyPressArray[KEYS.DPAD_LEFT.value]) {
            ArmControl.setArmLevel(ArmControl.Levels.FIRST);
        } else if (keyPressArray[KEYS.DPAD_UP.value]) {
            ArmControl.setArmLevel(ArmControl.Levels.SECOND);
        } else if (keyPressArray[KEYS.DPAD_RIGHT.value]) {
            ArmControl.setArmLevel(ArmControl.Levels.THIRD);
        }
    }

    public void handleDrive(SampleMecanumDrive drive, boolean isBalan) {
        final double triggerMovement = -gamepad1.right_trigger + gamepad1.left_trigger;
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * SPEED_MULTIPLIER,
                        -gamepad1.left_stick_x * SPEED_MULTIPLIER,
                        SPEED_MULTIPLIER * (isBalan ? triggerMovement : -gamepad1.right_stick_x)
                )
        );

        drive.update();
    }

    public void handleSpeedMultiplier(Gamepad gp) {
        if (gamepad1.left_bumper) {
            SPEED_MULTIPLIER = 0.5;
            gp.rumble(0.1, 0.1, 100);
            gp.setLedColor(255, 0, 0, 100);
        } else if (gamepad1.right_bumper) {
            SPEED_MULTIPLIER = 0.75;
            gp.rumble(0.6, 0.6, 200);
            gp.setLedColor(0, 0, 255, 100);
        }

        telemetry.addData("Speed Multiplier", SPEED_MULTIPLIER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArmControl.init(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            updateKeysPressed(gamepad1);

            handleClipControls();

            handleArmControls();

            handleDrive(drive, true);

            handleSpeedMultiplier(gamepad1);

            telemetry.update();
        }
    }
}

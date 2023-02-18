package org.firstinspires.ftc.teamcode.arm_control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
public class ArmControl {
    public static DcMotor armMotor1 = null;
    public static DcMotor armMotor2 = null;
    public static Servo clipServo = null;

    private static Telemetry telemetry = null;

    // 30, 1277, 2035, 2889
    private static final List<Integer> armMotorLevels = Arrays.asList(3, 1300, 2100, 2920);

    private static final double ARM_MOTOR_POWER_UP = 0.85;
    private static final double ARM_MOTOR_POWER_DOWN = 0.75;

    private static double currentArmPower = 0.0;

    public static enum Levels {
        DOWN (0),
        FIRST (1),
        SECOND (2),
        THIRD (3);

        public final int value;
        Levels(int i) {
            this.value = i;
        }
    }

    private static Levels currentLevel = Levels.DOWN;

    public static void init(HardwareMap hwm, Telemetry t) {
        armMotor1 = hwm.get(DcMotor.class, "m_arm1");
        armMotor2 = hwm.get(DcMotor.class, "m_arm2");

        armMotor1.setTargetPosition(armMotorLevels.get(Levels.DOWN.value));
        armMotor2.setTargetPosition(armMotorLevels.get(Levels.DOWN.value));

        armMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        clipServo = hwm.get(Servo.class, "s_armclip"); // sau nu asa

        armMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = t;
    }

    public static void setArmPower(double power) {
        armMotor1.setPower(power);
        armMotor2.setPower(power);
    }

    public static void runToPosition(int position, double power) {
        armMotor2.setTargetPosition(position);
        armMotor1.setTargetPosition(position);

        setArmPower(power);
    }

    public static void logArmInfo() {
        if (!armMotor1.isBusy() && !armMotor2.isBusy()) {
//            setArmPower(0.0);
            telemetry.addData("Motors", "DONE RUNNING!");
        }

        telemetry.addData("Current level" , currentLevel.value);
        telemetry.addData("Target ticks", armMotorLevels.get(currentLevel.value));
        telemetry.addData("Arm power", currentArmPower);
//        telemetry.update();
    }

//    public static void runManually(double power) {
//        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        if (armMotor1.getCurrentPosition() < -50 || armMotor2.getCurrentPosition() < -50) {
//            armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        setArmPower(power);
//    }

    public static void setArmLevel(Levels level) {
//        double finalPower = 0.0;

        // TODO: possibly rethink this since this we may run into further issues later on
        if (level.equals(currentLevel)) {
            return;
        }

        if (level.compareTo(currentLevel) > 0) {
            currentArmPower = ARM_MOTOR_POWER_UP;
        } else {
            currentArmPower = ARM_MOTOR_POWER_DOWN;
        }

        armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentLevel = level;
        runToPosition(armMotorLevels.get(currentLevel.value), currentArmPower);
    }

    public static void setArmLevel(int level, double power) {
        runToPosition(armMotorLevels.get(level), power);
    }

    public static void openClip() {
        clipServo.setPosition(0.2);
    }

    public static void closeClip() {
        clipServo.setPosition(0.8);
    }

    public static boolean isClipClosed() {
        return clipServo.getPosition() > 0; // sau ceva de genu
    }
}

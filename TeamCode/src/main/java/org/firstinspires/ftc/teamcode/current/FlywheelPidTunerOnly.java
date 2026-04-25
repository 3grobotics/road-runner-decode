package org.firstinspires.ftc.teamcode.current;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Config
@Disabled
@TeleOp(name="Flywheel PID Tuner ONLY", group="tuning")
public class FlywheelPidTunerOnly extends LinearOpMode {

    // -------- Flywheel PIDF (Dashboard) --------
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double targetVel = 0.0;
    public static double velStep   = 50.0;
    public static double maxVel    = 7000.0;

    public static double ticksPerRev = 28.0;

    public static String MOTOR1_NAME = "flywheel1";
    public static String MOTOR2_NAME = "flywheel2";

    // -------- Intake + Gecko force run (Dashboard) --------
    // set to 1 to run both at 0.5, set to 0 to turn them off
    public static int RUN_FEED = 0;
    public static double FEED_POWER = 0.5;

    // Names as you specified
    public static String INTAKE_NAME = "intake";
    public static String GECKO_NAME  = "indexer";

    DcMotorEx f1, f2;
    DcMotor intake;
    DcMotorEx gecko;

    boolean prevUp = false;
    boolean prevDown = false;

    @Override
    public void runOpMode() {

        f1 = hardwareMap.get(DcMotorEx.class, MOTOR1_NAME);
        f2 = hardwareMap.get(DcMotorEx.class, MOTOR2_NAME);

        intake = hardwareMap.get(DcMotor.class, INTAKE_NAME);
        gecko  = hardwareMap.get(DcMotorEx.class, GECKO_NAME);
        gecko.setDirection(DcMotorSimple.Direction.REVERSE);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);

        f1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        f2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Flywheel PID Tuner ONLY (+ optional feed motors)");
        telemetry.addLine("Dashboard: kP / kI / kD / kF / targetVel");
        telemetry.addLine("Dashboard: RUN_FEED (0/1) + FEED_POWER");
        telemetry.addLine("Dpad Up/Down = change target velocity");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- Feed motors (only if RUN_FEED == 1) ----------
            if (RUN_FEED == 1) {
                double p = Range.clip(FEED_POWER, -1.0, 1.0);
                intake.setPower(p);
                gecko.setPower(p);
            } else {
                intake.setPower(0.0);
                gecko.setPower(0.0);
            }

            // ---------- Flywheel target adjust ----------
            boolean up   = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up && !prevUp)     targetVel += velStep;
            if (down && !prevDown) targetVel -= velStep;

            prevUp = up;
            prevDown = down;

            targetVel = Range.clip(targetVel, 0.0, maxVel);

            // ---------- Apply PIDF live ----------
            f1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            f2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            // ---------- Command velocity ----------
            f1.setVelocity(targetVel);
            f2.setVelocity(targetVel);

            // ---------- Telemetry ----------
            double v1 = f1.getVelocity();
            double v2 = f2.getVelocity();

            telemetry.addData("Target (ticks/s)", "%.1f", targetVel);
            telemetry.addData("Flywheel1 (ticks/s)", "%.1f", v1);
            telemetry.addData("Flywheel2 (ticks/s)", "%.1f", v2);
            telemetry.addData("Flywheel1 RPM", "%.1f", (f1.getVelocity() * 60) / 28);
            telemetry.addData("Flywheel2 RPM", "%.1f", (f2.getVelocity() * 60) / 28);

            telemetry.addData("RUN_FEED", RUN_FEED);
            telemetry.addData("FEED_POWER", FEED_POWER);
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("gecko power", gecko.getPower());

            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);
            telemetry.update();

            // Dashboard mirror
            FtcDashboard.getInstance().getTelemetry().addData("targetVel", targetVel);
            FtcDashboard.getInstance().getTelemetry().addData("v1", v1);
            FtcDashboard.getInstance().getTelemetry().addData("v2", v2);
            FtcDashboard.getInstance().getTelemetry().addData("RUN_FEED", RUN_FEED);
            FtcDashboard.getInstance().getTelemetry().addData("FEED_POWER", FEED_POWER);
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
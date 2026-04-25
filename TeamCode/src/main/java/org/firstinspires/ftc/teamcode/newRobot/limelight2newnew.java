package org.firstinspires.ftc.teamcode.newRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.Register;

import java.util.List;
@Disabled
@Config
@TeleOp(name = "Turret + Limelight + Pinpoint (Field Hold + Re-localize)", group = "Test")
public class limelight2newnew extends LinearOpMode {

    private DcMotorEx turret;
    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;
    private FtcDashboard dashboard;

    // ------------------ Tunables ------------------
    public static boolean REVERSE_TURRET = false;

    // Turret encoder conversion (measured ticks per 360° turret output)
    public static double TURRET_TICKS_PER_REV = 418.557;

    // Hard turret limits (robot-relative)
    public static double TURRET_MIN_DEG = -90.0;
    public static double TURRET_MAX_DEG = 90.0;

    // Turret position PD (robot-relative degrees -> power)
    public static double POS_KP = 0.018;
    public static double POS_KD = 0.0015;
    public static double POS_TOL_DEG = 1.5;
    public static double MIN_POWER = 0.06;     // if it oscillates near target: LOWER this or increase POS_TOL_DEG
    public static double MAX_POWER = 0.35;

    // Manual: change hold direction (field-centric)
    public static double MANUAL_DEADBAND = 0.12;
    public static double MANUAL_FIELD_DEG_PER_SEC = 180.0;

    // Limelight
    public static int PIPELINE = 0;
    public static double AIM_OFFSET_DEG = 0.0;

    // Tag debounce (frame-based)
    public static int SEEN_FRAMES_ON = 2;
    public static int LOST_FRAMES_OFF = 6;

    // Toggle aim
    private boolean prevToggle = false;
    public static boolean AIM_ENABLED_DEFAULT = true;
    private boolean aimEnabled = AIM_ENABLED_DEFAULT;

    // ------------------ Re-localize behavior ------------------
    // When tag is seen, compute a tag-corrected field heading and set holdFieldDeg to it.
    public static boolean SNAP_ON_TAG = true;          // true = hard snap, false = blend
    public static double SNAP_ALPHA = 0.35;            // used only if SNAP_ON_TAG=false (0.2–0.6)
    public static double TAG_TX_DEADBAND = 0.8;        // ignore tiny tx noise
    public static double TAG_UPDATE_COOLDOWN_SEC = 0.10; // don't update every frame
    public static boolean TX_SIGN_NEGATIVE = true;     // if it aims away, flip this
    public static double SNAP_ONLY_IF_TX_UNDER = 20.0; // avoid snapping when tx is huge/jumpy

    // ------------------ State ------------------
    private double holdFieldDeg = 0.0;
    private boolean holdInitialized = false;

    private int seenCount = 0;
    private int lostCount = 0;
    private boolean stableSeen = false;

    private double prevPosErr = 0.0;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime tagUpdateTimer = new ElapsedTime();

    // Pinpoint bulk registers (same set as example)
    private final Register[] defaultRegisters = {
            Register.DEVICE_STATUS,
            Register.LOOP_TIME,
            Register.X_ENCODER_VALUE,
            Register.Y_ENCODER_VALUE,
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
            Register.H_VELOCITY,
    };

    // ------------------ Helpers ------------------
    private double ticksToDeg(int ticks) {
        return (ticks * 360.0) / TURRET_TICKS_PER_REV;
    }

    private double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg <= -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    private double clampTurretDeg(double turretRobotDeg) {
        return Range.clip(turretRobotDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);
    }

    private double applyHardLimits(double currentTurretDeg, double pwr) {
        if (currentTurretDeg >= TURRET_MAX_DEG && pwr > 0) return 0.0;
        if (currentTurretDeg <= TURRET_MIN_DEG && pwr < 0) return 0.0;
        return pwr;
    }

    private double blendDeg(double aDeg, double bDeg, double alpha) {
        double err = wrapDeg(bDeg - aDeg); // shortest path
        return wrapDeg(aDeg + err * alpha);
    }

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        dashboard = FtcDashboard.getInstance();

        // Turret setup
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setDirection(REVERSE_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Limelight setup
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        // Pinpoint setup (based on your example)
        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);

        // These offsets/resolutions are for odometry; heading works regardless.
        // Keep them matching your robot’s setup:
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset heading/pose and calibrate IMU before run (robot stationary)
        odo.resetPosAndIMU();

        telemetry.addLine("Ready (Field Hold + Re-localize on Tag)");
        telemetry.update();

        waitForStart();
        loopTimer.reset();
        tagUpdateTimer.reset();

        while (opModeIsActive()) {

            // Update Pinpoint
            odo.update();

            // Toggle aim assist with dpad right
            boolean toggle = gamepad1.dpad_right;
            if (toggle && !prevToggle) {
                aimEnabled = !aimEnabled;
            }
            prevToggle = toggle;

            // dt
            double dt = loopTimer.seconds();
            if (dt < 0.001) dt = 0.001;
            loopTimer.reset();

            // Robot heading from Pinpoint (Pose heading getter needs an AngleUnit argument)
            double robotHeadingDeg = odo.getPosition().getHeading(AngleUnit.DEGREES);

            // Turret angle relative to robot from encoder
            double turretRobotDeg = wrapDeg(ticksToDeg(turret.getCurrentPosition()));

            // Current turret field direction
            double turretFieldDeg = wrapDeg(robotHeadingDeg + turretRobotDeg);

            // Initialize hold direction once
            if (!holdInitialized) {
                holdFieldDeg = turretFieldDeg;
                holdInitialized = true;
            }

            // Read Limelight
            LLResult result = limelight.getLatestResult();
            boolean rawSeen = false;
            double tx = 0.0;

            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    rawSeen = true;
                    tx = result.getTx() - AIM_OFFSET_DEG;
                }
            }

            // Debounce tag seen (frame-based)
            if (rawSeen) { seenCount++; lostCount = 0; }
            else { lostCount++; seenCount = 0; }

            if (!stableSeen && seenCount >= SEEN_FRAMES_ON) stableSeen = true;
            if (stableSeen && lostCount >= LOST_FRAMES_OFF) stableSeen = false;

            // Manual adjust hold direction (field-centric)
            double stick = -gamepad1.right_stick_x;
            if (Math.abs(stick) < MANUAL_DEADBAND) stick = 0.0;

            if (stick != 0.0) {
                // Driver changes the desired field direction
                holdFieldDeg = wrapDeg(holdFieldDeg + stick * MANUAL_FIELD_DEG_PER_SEC * dt);

                // While driver is commanding, don't spam tag updates
                tagUpdateTimer.reset();

            } else if (aimEnabled && stableSeen) {

                // Re-localize target heading using the tag when meaningful
                if (Math.abs(tx) > TAG_TX_DEADBAND && Math.abs(tx) < SNAP_ONLY_IF_TX_UNDER
                        && tagUpdateTimer.seconds() >= TAG_UPDATE_COOLDOWN_SEC) {

                    tagUpdateTimer.reset();

                    // If turret aims the wrong way, flip TX_SIGN_NEGATIVE
                    double txUsed = TX_SIGN_NEGATIVE ? (-tx) : (tx);

                    // Tag-corrected target field heading:
                    // turretFieldDeg is where we are pointing; add tx correction to get where we SHOULD point.
                    double targetFieldDeg = wrapDeg(turretFieldDeg + txUsed);

                    if (SNAP_ON_TAG) {
                        holdFieldDeg = targetFieldDeg;
                    } else {
                        holdFieldDeg = blendDeg(holdFieldDeg, targetFieldDeg, SNAP_ALPHA);
                    }
                }
            }
            // else: no tag -> keep holdFieldDeg unchanged (this prevents “thrash”)

            // Convert desired field direction -> desired turret robot-relative angle
            double targetTurretRobotDeg = wrapDeg(holdFieldDeg - robotHeadingDeg);
            targetTurretRobotDeg = clampTurretDeg(targetTurretRobotDeg);

            // PD on turret position to target
            double posErr = wrapDeg(targetTurretRobotDeg - turretRobotDeg);
            double derr = (posErr - prevPosErr) / dt;
            prevPosErr = posErr;

            double out = (POS_KP * posErr) + (POS_KD * derr);

            // Deadband + stiction
            if (Math.abs(posErr) <= POS_TOL_DEG) {
                out = 0.0;
            } else {
                if (Math.abs(out) < MIN_POWER) out = (out > 0) ? MIN_POWER : -MIN_POWER;
            }

            out = Range.clip(out, -MAX_POWER, MAX_POWER);
            out = applyHardLimits(turretRobotDeg, out);

            turret.setPower(out);

            // Telemetry
            telemetry.addData("AimEnabled", aimEnabled);
            telemetry.addData("rawSeen", rawSeen);
            telemetry.addData("stableSeen", stableSeen);
            telemetry.addData("tx", tx);

            telemetry.addData("robotHeadingDeg", robotHeadingDeg);
            telemetry.addData("turretRobotDeg", turretRobotDeg);
            telemetry.addData("turretFieldDeg", turretFieldDeg);
            telemetry.addData("holdFieldDeg", holdFieldDeg);

            telemetry.addData("targetTurretDeg", targetTurretRobotDeg);
            telemetry.addData("posErr", posErr);
            telemetry.addData("out", out);
            telemetry.update();

            dashboard.getTelemetry().addData("AimEnabled", aimEnabled);
            dashboard.getTelemetry().addData("stableSeen", stableSeen);
            dashboard.getTelemetry().addData("tx", tx);
            dashboard.getTelemetry().addData("turretFieldDeg", turretFieldDeg);
            dashboard.getTelemetry().addData("holdFieldDeg", holdFieldDeg);
            dashboard.getTelemetry().addData("targetTurretDeg", targetTurretRobotDeg);
            dashboard.getTelemetry().addData("posErr", posErr);
            dashboard.getTelemetry().addData("out", out);
            dashboard.getTelemetry().update();
        }

        turret.setPower(0);
        limelight.stop();
    }
}

package org.firstinspires.ftc.teamcode.discoverfest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "blue auto (close) — preload")
public class discoverfestAUTOblueclose extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private Servo  tiltLeft, tiltMid, tiltRight;
    private CRServo pullLeft, pullMid, pullRight;

    // ---- Limelight ----
    private static final int APRILTAG_PIPELINE  = 1;
    private static final int INIT_SCAN_PIPELINE = 2;

    // ---- Aim (TeleOp-style) ----
    private static final double AIM_KP       = 0.04;
    private static final double AIM_KD       = 0.002;
    private static final double AIM_TOL_DEG  = 3.0;
    private static final double AIM_YAW_MAX  = 0.6;
    private static final double AIM_MIN_CMD  = 0.08;
    private static final double AIM_OFFSET_DEG = 1;

    // ===== Catapults =====
    // your directions
    private static final double CAT_DIR_LEFT  = -1.0;
    private static final double CAT_DIR_MID   = -1.0;
    private static final double CAT_DIR_RIGHT = -1.0;

    // big preload
    private static final long   PRELOAD_MS_LEFT  = 1000;
    private static final long   PRELOAD_MS_MID   = 1000;
    private static final long   PRELOAD_MS_RIGHT = 1000;

    // fire pulse
    private static final long   FIRE_MS_LEFT  = 100;
    private static final long   FIRE_MS_MID   = 100;
    private static final long   FIRE_MS_RIGHT = 100;

    private static final double CAT_IDLE_POWER = 0.22;
    private static final long   CAT_GAP_MS     = 500;

    // ===== MOVEMENT (TeleOp motor dirs) =====
    // FL/BL FORWARD, FR/BR REVERSE → +power = go BACK
    private static final double BACK_POWER      = -0.55;
    private static final long   BACK_MS         = 1500;

    // your “turn to scan” were 0 — keeping as-is
    private static final double TURN_TO_SCAN_POWER = 0;   // CW
    private static final long   TURN_TO_SCAN_MS    = 1000;
    private static final double TURN_BACK_POWER    = 0;   // CCW
    private static final long   TURN_BACK_MS       = 1000;

    // old “roll forward” — we’re replacing this with turn+drive-away
    private static final double DRIVE_FWD_POWER    = 0.4;
    private static final long   DRIVE_FWD_MS       = 400;

    // NEW: end-of-auto escape
    private static final double END_TURN_POWER     = 0.45;
    private static final long   END_TURN_MS        = 500;
    private static final double END_DRIVE_POWER    = 1;
    private static final long   END_DRIVE_MS       = 700;

    // TeleOp tilt set
    private static final double TILT_LEFT_TELEOP  = 0.440;
    private static final double TILT_MID_TELEOP   = 0.430;
    private static final double TILT_RIGHT_TELEOP = 0.390;

    private final ElapsedTime aimTimer = new ElapsedTime();

    private enum Side { LEFT, MID, RIGHT }
    private final List<Side> shootOrder = new ArrayList<>();
    private int chosenInitPrimaryId = -1;

    @Override
    public void runOpMode() {
        // ---- hardware ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        tiltLeft   = hardwareMap.get(Servo.class, "tiltLeft");
        tiltMid    = hardwareMap.get(Servo.class, "tiltMid");
        tiltRight  = hardwareMap.get(Servo.class, "tiltRight");

        pullLeft   = hardwareMap.get(CRServo.class, "pullLeft");
        pullMid    = hardwareMap.get(CRServo.class, "pullMid");
        pullRight  = hardwareMap.get(CRServo.class, "pullRight");

        // motor directions (TeleOp)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // tilts from your TeleOp data
        tiltLeft.setPosition(TILT_LEFT_TELEOP);
        tiltMid.setPosition(TILT_MID_TELEOP);
        tiltRight.setPosition(TILT_RIGHT_TELEOP);

        // idle catapults
        pullLeft.setPower(CAT_IDLE_POWER);
        pullMid.setPower(CAT_IDLE_POWER);
        pullRight.setPower(CAT_IDLE_POWER);

        // Limelight
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(INIT_SCAN_PIPELINE);
        limelight.start();

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addLine("blue close auto — preload mode");
            telemetry.update();
        }

        waitForStart();
        if (!opModeIsActive()) { limelight.stop(); return; }

        // 0) Back up
        setDrivePowers(BACK_POWER, BACK_POWER, BACK_POWER, BACK_POWER);
        sleep(BACK_MS);
        stopDrive();

        // 1) Rotate CW to see motif
        setDrivePowers(TURN_TO_SCAN_POWER, -TURN_TO_SCAN_POWER, TURN_TO_SCAN_POWER, -TURN_TO_SCAN_POWER);
        sleep(TURN_TO_SCAN_MS);
        stopDrive();

        // 2) Scan motif (pipeline 2)
        long scanStart = System.currentTimeMillis();
        chosenInitPrimaryId = -1;
        while (opModeIsActive() && System.currentTimeMillis() - scanStart < 1200) {
            LLResult rr = limelight.getLatestResult();
            if (rr != null) {
                List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getTargetArea() > best.getTargetArea()) best = f;
                    }
                    chosenInitPrimaryId = best.getFiducialId();
                    break;
                }
            }
            idle();
        }
        pickOrderFromInit(chosenInitPrimaryId);

        // 3) Rotate CCW back
        setDrivePowers(TURN_BACK_POWER, -TURN_BACK_POWER, TURN_BACK_POWER, -TURN_BACK_POWER);
        sleep(TURN_BACK_MS);
        stopDrive();

        // 4) Switch to aim pipeline
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        sleep(60);

        // 5) Aim PD
        aimTimer.reset();
        double prevErrDeg = 0.0;
        long   lockAccumMs = 0;
        long   searchStart  = System.currentTimeMillis();
        boolean shootAllowed = false;

        while (opModeIsActive() && !shootAllowed) {
            LLResult result = limelight.getLatestResult();
            boolean haveTag = false;
            double  aimErrDeg = 0.0;

            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult fr : tags) {
                        if (fr.getTargetArea() > best.getTargetArea()) best = fr;
                    }
                    try { aimErrDeg = best.getTargetXDegrees(); }
                    catch (Throwable t) { aimErrDeg = result.getTx(); }
                    haveTag = true;
                }
            }

            double dt = Math.max(aimTimer.seconds(), 1e-3);
            aimTimer.reset();

            if (haveTag) {
                double err  = (aimErrDeg - AIM_OFFSET_DEG);
                double derr = (err - prevErrDeg) / dt;

                double yawAuto = -(AIM_KP * err + AIM_KD * derr);
                if (Math.abs(err) <= AIM_TOL_DEG) {
                    yawAuto = 0.0;
                    lockAccumMs += (long)(dt * 1000.0);
                } else {
                    lockAccumMs = 0;
                    double sgn = Math.signum(yawAuto);
                    if (Math.abs(yawAuto) < AIM_MIN_CMD) yawAuto = sgn * AIM_MIN_CMD;
                }

                yawAuto = Range.clip(yawAuto, -AIM_YAW_MAX, AIM_YAW_MAX);
                setDrivePowers(yawAuto, -yawAuto, yawAuto, -yawAuto);
                prevErrDeg = err;

                if (lockAccumMs >= 200) shootAllowed = true;
            } else {
                setDrivePowers(0.22, -0.22, 0.22, -0.22);
                lockAccumMs = 0;
                if (System.currentTimeMillis() - searchStart > 3500) break;
            }
        }
        stopDrive();

        // 6) SHOOT — ALWAYS MIDDLE FIRST, then motif order (skipping MID if we already shot it)
        if (opModeIsActive()) {
            sleep(200);

            // --- middle first ---
            preloadAndFire(pullMid, CAT_DIR_MID, PRELOAD_MS_MID, FIRE_MS_MID, CAT_IDLE_POWER);
            sleep(CAT_GAP_MS);

            // --- then motif order, but skip MID because we just fired it ---
            for (int i = 0; i < shootOrder.size(); i++) {
                Side s = shootOrder.get(i);
                if (s == Side.MID) continue;  // already did it
                switch (s) {
                    case LEFT:
                        preloadAndFire(pullLeft,  CAT_DIR_LEFT,  PRELOAD_MS_LEFT,  FIRE_MS_LEFT,  CAT_IDLE_POWER);
                        break;
                    case RIGHT:
                        preloadAndFire(pullRight, CAT_DIR_RIGHT, PRELOAD_MS_RIGHT, FIRE_MS_RIGHT, CAT_IDLE_POWER);
                        break;
                }
                // gap between non-mid shots
                sleep(CAT_GAP_MS);
            }
        }

        // 7) TURN AND DRIVE AWAY (instead of just rolling forward)
        // turn in place
        setDrivePowers(END_TURN_POWER, -END_TURN_POWER, END_TURN_POWER, -END_TURN_POWER);
        sleep(END_TURN_MS);
        stopDrive();

        // drive away from goal
        setDrivePowers(END_DRIVE_POWER, END_DRIVE_POWER, END_DRIVE_POWER, END_DRIVE_POWER);
        sleep(END_DRIVE_MS);
        stopDrive();

        limelight.stop();
    }

    // Map tag → order (kept same, we just force MID first later)
    private void pickOrderFromInit(int id) {
        shootOrder.clear();
        if (id == 24) {
            shootOrder.add(Side.LEFT); shootOrder.add(Side.MID);  shootOrder.add(Side.RIGHT);
        } else if (id == 21) {
            shootOrder.add(Side.MID);  shootOrder.add(Side.RIGHT); shootOrder.add(Side.LEFT);
        } else if (id == 22) {
            shootOrder.add(Side.RIGHT); shootOrder.add(Side.LEFT); shootOrder.add(Side.MID);
        } else {
            shootOrder.add(Side.RIGHT); shootOrder.add(Side.LEFT); shootOrder.add(Side.MID);
        }
    }

    private void preloadAndFire(CRServo catapult,
                                double dir,
                                long preloadMs,
                                long fireMs,
                                double idlePower) {
        // PRELOAD (pull back / tension)
        catapult.setPower(-dir);
        sleep(preloadMs);

        // tiny pause
        catapult.setPower(0.0);
        sleep(50);

        // FIRE
        catapult.setPower(dir);
        sleep(fireMs);

        // IDLE
        catapult.setPower(idlePower);
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopDrive() {
        setDrivePowers(0,0,0,0);
    }
}
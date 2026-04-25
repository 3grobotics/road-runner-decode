package org.firstinspires.ftc.teamcode.discoverfest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
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
@Autonomous(name = "blue auto")
public class discoverfestAUTOblue extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Correct: LEFT/MIDDLE/RIGHT tilts
    private Servo tiltLeft, tiltMid, tiltRight;

    // Correct: LEFT/MIDDLE/RIGHT catapults
    private CRServo pullLeft, pullMid, pullRight;

    // ---- Limelight ----
    private static final int APRILTAG_PIPELINE   = 1; // AFTER start (blue)
    private static final int INIT_SCAN_PIPELINE  = 2; // DURING init

    // ---- Aim controller (PD on tx) ----
    private static final double AIM_KP       = 0.045;
    private static final double AIM_KD       = 0.0035;
    private static final double AIM_TOL_DEG  = 5.0;
    private static final double AIM_MIN_CMD  = 0.08;
    private static final double AIM_YAW_MAX  = 0.55;
    private static final double SEARCH_SPIN  = 0.23;       // blue spins this way
    private static final double AIM_OFFSET_DEG = -4;         // aim a little to the left

    // ---- Lock + Shoot ----
    private static final long   LOCK_HOLD_MS   = 200;
    private static final long   SHOOT_SETTLE   = 200;
    private static final double SHOOT_PWR      = 1.0;

    // Per-catapult idle powers
    private static final double SHOOT_IDLE_LEFT  = 0.2;
    private static final double SHOOT_IDLE_RIGHT = 0.2;
    private static final double SHOOT_IDLE_MID   = 0.2;

    private static final boolean REQUIRE_LOCK_TO_SHOOT = true;

    // ---- Staggered fire timings ----
    private static final long  SHOOT_MS_EACHLEFT   = 1000;
    private static final long  SHOOT_MS_EACHRIGHT  = 1000;
    private static final long  SHOOT_MS_EACHMID    = 1000;
    private static final long  SHOOT_GAP_MS        = 1000;

    // CRServo directions
    private static final double CAT_DIR_LEFT  = 1.0;
    private static final double CAT_DIR_MID   = 1.0;
    private static final double CAT_DIR_RIGHT = 1.0;

    // ---- Post-shoot drive ----
    private static final double DRIVE_POWER  = .9;
    private static final long   DRIVE_MS     = 500 ;

    // ---- Tilt presets ----
    private static final double LEFT_TILT_PRE   = 0.300;
    private static final double RIGHT_TILT_PRE  = 0.330;
    private static final double MID_TILT_PRE    = 0.310;

    // ---- Pre-drive nudge (you already had) ----
    private static final double PRE_DRIVE_POWER = 0.20;
    private static final long   PRE_DRIVE_MS    = 250;

    // ---- NEW: little forward BEFORE we let it turn/search ----
    private static final double CREEP_FWD_POWER = 0.25;
    private static final long   CREEP_FWD_MS    = 300;

    private final ElapsedTime dtTimer = new ElapsedTime();

    // ===== Order selection based on INIT tag =====
    private enum Side { LEFT, RIGHT, MIDDLE }
    private final List<Side> shootOrder = new ArrayList<>();
    private int chosenInitPrimaryId = -1; // telemetry/debug

    @Override
    public void runOpMode() {
        // ---- Hardware ----
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Correctly named & mapped servos
        tiltLeft   = hardwareMap.get(Servo.class,  "tiltLeft");
        tiltMid    = hardwareMap.get(Servo.class,  "tiltMid");
        tiltRight  = hardwareMap.get(Servo.class,  "tiltRight");

        pullLeft   = hardwareMap.get(CRServo.class,"pullLeft");
        pullMid    = hardwareMap.get(CRServo.class,"pullMid");
        pullRight  = hardwareMap.get(CRServo.class,"pullRight");

        // Drive directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Preset tilts
        tiltLeft.setPosition(LEFT_TILT_PRE);
        tiltMid.setPosition(MID_TILT_PRE);
        tiltRight.setPosition(RIGHT_TILT_PRE);

        // Idle shooters
        pullLeft.setPower(SHOOT_IDLE_LEFT);
        pullMid.setPower(SHOOT_IDLE_MID);
        pullRight.setPower(SHOOT_IDLE_RIGHT);

        // ---- Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(INIT_SCAN_PIPELINE);
        limelight.start();

        // ================== INIT LOOP (pipeline 2) ==================
        while (opModeInInit() && !isStopRequested()) {
            LLStatus st = limelight.getStatus();
            LLResult rr = limelight.getLatestResult();

            if (rr != null) {
                List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    List<Integer> ids = new ArrayList<>(tags.size());
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult f : tags) {
                        ids.add(f.getFiducialId());
                        if (f.getTargetArea() > best.getTargetArea()) best = f;
                    }
                    chosenInitPrimaryId = best.getFiducialId();
                    telemetry.addData("INIT: Tag IDs", ids);
                    telemetry.addData("INIT: Primary ID", chosenInitPrimaryId);
                    try {
                        telemetry.addData("INIT: tx/ty (deg)", "%.1f / %.1f",
                                best.getTargetXDegrees(), best.getTargetYDegrees());
                    } catch (Throwable ignore) {
                        telemetry.addData("INIT: tx/ty (deg)", "%.1f / %.1f", rr.getTx(), rr.getTy());
                    }
                } else {
                    telemetry.addLine("INIT: No AprilTags detected (pipeline 2).");
                }
            } else {
                telemetry.addLine("INIT: No Limelight result yet…");
            }

            // Decide shoot order based on init primary ID
            pickOrderFromInit(chosenInitPrimaryId);
            telemetry.addData("Planned Order", orderToString(shootOrder));
            telemetry.addData("LL Pipeline", "%d (%s)", st.getPipelineIndex(), st.getPipelineType());
            telemetry.update();
            idle();
        }
        // ===========================================================

        waitForStart();
        if (!opModeIsActive()) {  return; }

        // Switch to pipeline 0 for aiming/shooting (red)
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        sleep(40);

        // 1) your original nudge
        setDrivePowers(PRE_DRIVE_POWER, PRE_DRIVE_POWER, PRE_DRIVE_POWER, PRE_DRIVE_POWER);
        sleep(PRE_DRIVE_MS);
        stopDrive();

        // 2) NEW: creep forward a bit more BEFORE the aim loop tries to spin
        setDrivePowers(CREEP_FWD_POWER, CREEP_FWD_POWER, CREEP_FWD_POWER, CREEP_FWD_POWER);
        sleep(CREEP_FWD_MS);
        stopDrive();

        // ======= STATE 1: AUTO-AIM (sustained lock) =======
        dtTimer.reset();
        double prevErrDeg = 0.0;
        long   searchStartMs = System.currentTimeMillis();
        long   lockAccumMs   = 0;
        boolean shootAllowed = false;

        while (opModeIsActive() && !shootAllowed) {
            LLResult rr = limelight.getLatestResult();

            boolean haveTag = false;
            double  txDeg   = 0.0;

            if (rr != null) {
                List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getTargetArea() > best.getTargetArea()) best = f;
                    }
                    try { txDeg = best.getTargetXDegrees(); }
                    catch (Throwable t) { txDeg = rr.getTx(); }
                    haveTag = true;
                }
            }

            double dt = Math.max(dtTimer.seconds(), 1e-3);
            dtTimer.reset();

            if (haveTag) {
                double err  = txDeg - AIM_OFFSET_DEG;
                double derr = (err - prevErrDeg) / dt;

                double cmd  = -(AIM_KP * err + AIM_KD * derr);

                if (Math.abs(err) <= AIM_TOL_DEG) {
                    cmd = 0.0;
                    lockAccumMs += (long)(dt * 1000.0);
                } else {
                    lockAccumMs = 0;
                    cmd = (cmd > 0) ? Math.max(cmd, AIM_MIN_CMD) : Math.min(cmd, -AIM_MIN_CMD);
                }

                double yawCmd = Range.clip(cmd, -AIM_YAW_MAX, AIM_YAW_MAX);
                prevErrDeg = err;

                setDrivePowers(yawCmd, -yawCmd, yawCmd, -yawCmd);

                telemetry.addData("tx(deg)", "%.2f", txDeg);
                telemetry.addData("offset(deg)", AIM_OFFSET_DEG);
                telemetry.addData("err=tx-offset", "%.2f", err);
                telemetry.addData("hold(ms)", lockAccumMs);
                telemetry.update();

                if (lockAccumMs >= LOCK_HOLD_MS) {
                    shootAllowed = true;
                }
            } else {
                // THIS is the “turning” you were talking about — now it only happens
                // after we’ve already crept forward.
                setDrivePowers(SEARCH_SPIN, -SEARCH_SPIN, SEARCH_SPIN, -SEARCH_SPIN);
                lockAccumMs = 0;
                telemetry.addLine("Searching for tag (pipeline 0)...");
                telemetry.update();

                if (System.currentTimeMillis() - searchStartMs > 4000) break;
            }
        }

        stopDrive();

        // ======= STATE 2: SHOOT in chosen order =======
        if (opModeIsActive() && (!REQUIRE_LOCK_TO_SHOOT || shootAllowed)) {
            sleep((int)SHOOT_SETTLE);

            for (int i = 0; i < shootOrder.size(); i++) {
                Side s = shootOrder.get(i);
                switch (s) {
                    case LEFT:
                        fireOne(pullLeft,  CAT_DIR_LEFT,  SHOOT_MS_EACHLEFT,  SHOOT_IDLE_LEFT);
                        break;
                    case RIGHT:
                        fireOne(pullRight, CAT_DIR_RIGHT, SHOOT_MS_EACHRIGHT, SHOOT_IDLE_RIGHT);
                        break;
                    case MIDDLE:
                        fireOne(pullMid,   CAT_DIR_MID,   SHOOT_MS_EACHMID,   SHOOT_IDLE_MID);
                        break;
                }
                if (i != shootOrder.size() - 1) sleep(SHOOT_GAP_MS);
            }
        }

        // ======= STATE 3: DRIVE FORWARD =======
        long end = System.currentTimeMillis() + DRIVE_MS;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            setDrivePowers(DRIVE_POWER, DRIVE_POWER, DRIVE_POWER, DRIVE_POWER);
        }
        stopDrive();

        limelight.stop();
    }

    // Map INIT primary tag → order
    private void pickOrderFromInit(int id) {
        shootOrder.clear();
        if (id == 22) {              // L, M, R
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
            shootOrder.add(Side.RIGHT);
        } else if (id == 21) {       // M, R, L
            shootOrder.add(Side.MIDDLE);
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
        } else if (id == 23) {       // R, L, M
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
        } else {                     // fallback
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
        }
    }

    private String orderToString(List<Side> seq) {
        StringBuilder sb = new StringBuilder();
        for (Side s : seq) sb.append(s == Side.LEFT ? "L" : (s == Side.RIGHT ? "R" : "M")).append(' ');
        return sb.toString().trim();
    }

    private void fireOne(CRServo catapult, double dir, long ms, double idlePower) {
        catapult.setPower(dir * SHOOT_PWR);
        sleep(ms);
        catapult.setPower(idlePower);
    }

    private void setDrivePowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopDrive() {
        setDrivePowers(0, 0, 0, 0);
    }
}
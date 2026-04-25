package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Autonomous(name = "farAutoRed with camera", group = "competition")
public class farAutoRedWithCamera extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public Servo flipper, tiltLeft, tiltMid, tiltRight;
    public CRServo pullRight, pullMid, pullLeft;
    public DigitalChannel leftSwitch, midSwitch, rightSwitch;

    /* ──────────────── Limelight params (matching discoverfest) ──────────────── */
    public Limelight3A limelight;

    private static final int APRILTAG_PIPELINE  = 0; // after start (red)
    private static final int INIT_SCAN_PIPELINE = 2; // during init

    // PD aim
    private static final double AIM_KP = 0.045;
    private static final double AIM_KD = 0.0035;
    private static final double AIM_TOL_DEG = 5.0;
    private static final double AIM_MIN_CMD = 0.08;
    private static final double AIM_YAW_MAX = 0.55;
    private static final double SEARCH_SPIN = -0.23;     // red spins this way
    private static final double AIM_OFFSET_DEG = 4.0;    // slight right bias

    // Lock + shoot behavior
    private static final long   LOCK_HOLD_MS = 200;
    private static final long   SHOOT_SETTLE = 200;
    private static final double SHOOT_PWR    = 1.0;

    // CRServo idle powers
    private static final double SHOOT_IDLE_LEFT  = 0.2;
    private static final double SHOOT_IDLE_MID   = 0.2;
    private static final double SHOOT_IDLE_RIGHT = 0.2;

    private static final boolean REQUIRE_LOCK_TO_SHOOT = true;

    // Staggered fire timing
    private static final long SHOOT_MS_EACHLEFT  = 1000;
    private static final long SHOOT_MS_EACHMID   = 1000;
    private static final long SHOOT_MS_EACHRIGHT = 1000;
    private static final long SHOOT_GAP_MS       = 1000;

    // Shooter directions
    private static final double CAT_DIR_LEFT  = 1.0;
    private static final double CAT_DIR_MID   = 1.0;
    private static final double CAT_DIR_RIGHT = 1.0;

    // Tilt presets you use before winching
    private static final double LEFT_TILT_PRE  = 0.300;
    private static final double MID_TILT_PRE   = 0.310;
    private static final double RIGHT_TILT_PRE = 0.330;

    private final ElapsedTime aimTimer = new ElapsedTime();

    // INIT tag → order selection
    private enum Side { LEFT, MIDDLE, RIGHT }
    private final List<Side> shootOrder = new ArrayList<>();
    private int chosenInitPrimaryId = -1;

    /* ──────────────── Aim action (angular velocity until locked) ──────────────── */
    private class AimWithLL implements Action {
        private final MecanumDrive drive;
        private double prevErr = 0.0;
        private long   prevNs  = 0L;
        private long   lockAccumMs = 0L;

        AimWithLL(MecanumDrive drive) {
            this.drive = drive;
            prevNs = System.nanoTime();
            lockAccumMs = 0;
        }

        @Override public boolean run(@NonNull TelemetryPacket p) {
            // read LL
            LLResult rr = limelight.getLatestResult();
            boolean haveTag = false;
            double txDeg = 0.0;

            if (rr != null) {
                List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult f : tags)
                        if (f.getTargetArea() > best.getTargetArea()) best = f;
                    try { txDeg = best.getTargetXDegrees(); }
                    catch (Throwable t) { txDeg = rr.getTx(); }
                    haveTag = true;
                }
            }

            long now = System.nanoTime();
            double dt = Math.max((now - prevNs) / 1e9, 1e-3);
            prevNs = now;

            if (!haveTag) {
                // search spin
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), SEARCH_SPIN));
                lockAccumMs = 0;
                p.put("Aim", "searching");
                return true;
            }

            // PD on (tx - offset)
            double err  = txDeg - AIM_OFFSET_DEG;
            double derr = (err - prevErr) / dt;
            prevErr = err;

            double yaw = -(AIM_KP * err + AIM_KD * derr);

            if (Math.abs(err) <= AIM_TOL_DEG) {
                yaw = 0.0;
                lockAccumMs += (long) (dt * 1000.0);
            } else {
                lockAccumMs = 0;
                if (Math.abs(yaw) < AIM_MIN_CMD) yaw = Math.copySign(AIM_MIN_CMD, yaw);
            }

            yaw = Range.clip(yaw, -AIM_YAW_MAX, AIM_YAW_MAX);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), yaw));

            p.put("tx", err);
            p.put("yawCmd", yaw);
            p.put("lockMs", lockAccumMs);

            if (lockAccumMs >= LOCK_HOLD_MS) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                p.put("Aim", "locked");
                return false;
            }
            return true;
        }
    }

    /* ──────────────── order + fire helpers ──────────────── */
    private void pickOrderFromInit(int id) {
        shootOrder.clear();
        if (id == 22) {                 // L, M, R
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
            shootOrder.add(Side.RIGHT);
        } else if (id == 21) {          // M, R, L
            shootOrder.add(Side.MIDDLE);
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
        } else if (id == 23) {          // R, L, M
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
        } else {                        // fallback
            shootOrder.add(Side.RIGHT);
            shootOrder.add(Side.LEFT);
            shootOrder.add(Side.MIDDLE);
        }
    }

    private void fireOne(CRServo cat, double dir, long ms, double idle) throws InterruptedException {
        cat.setPower(dir * SHOOT_PWR);
        sleep(ms);
        cat.setPower(idle);
    }

    private void fireByOrder() throws InterruptedException {
        sleep((int)SHOOT_SETTLE);
        for (int i = 0; i < shootOrder.size(); i++) {
            Side s = shootOrder.get(i);
            switch (s) {
                case LEFT:   fireOne(pullLeft,  CAT_DIR_LEFT,  SHOOT_MS_EACHLEFT,  SHOOT_IDLE_LEFT);  break;
                case MIDDLE: fireOne(pullMid,   CAT_DIR_MID,   SHOOT_MS_EACHMID,   SHOOT_IDLE_MID);   break;
                case RIGHT:  fireOne(pullRight, CAT_DIR_RIGHT, SHOOT_MS_EACHRIGHT, SHOOT_IDLE_RIGHT); break;
            }
            if (i != shootOrder.size() - 1) sleep(SHOOT_GAP_MS);
        }
    }

    /* ──────────────── Robot actions (as you had) ──────────────── */
    public class Robot {
        private class posservo implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                tiltLeft.setPosition(.290);
                tiltMid.setPosition(.320);
                tiltRight.setPosition(.310);
                return false;
            }
        }
        public Action posservo() { return new posservo(); }

        private class posservo2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.18);
                tiltLeft.setPosition(0.380);
                tiltMid.setPosition(0.350);
                tiltRight.setPosition(0.400);
                return false;
            }
        }
        public Action posservo2() { return new posservo2(); }

        private class posservo3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.3333333333333333);
                tiltLeft.setPosition(0.03);
                tiltMid.setPosition(0.03);
                tiltRight.setPosition(0.03);

                if (leftSwitch.getState())  pullLeft.setPower(.2);  else pullLeft.setPower(1.0);
                if (midSwitch.getState())   pullMid.setPower(.2);   else pullMid.setPower(1.0);
                if (rightSwitch.getState()) pullRight.setPower(.2); else pullRight.setPower(1.0);
                return true;
            }
        }
        public Action posservo3() { return new posservo3(); }

        private class intake implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ intake.setPower(1); return false; } }
        public Action intake(){ return new intake(); }

        private class intake_stop implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ intake.setPower(0); return false; } }
        public Action intake_stop(){ return new intake_stop(); }

        private class fireM implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullMid.setPower(1); sleep(500); return false; } }
        public Action fireM(){ return new fireM(); }

        private class fireR implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullRight.setPower(1); sleep(500); return false; } }
        public Action fireR(){ return new fireR(); }

        private class fireL implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullLeft.setPower(1); sleep(500); return false; } }
        public Action fireL(){ return new fireL(); }

        private class stopM implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullMid.setPower(0); return false; } }
        public Action stopM(){ return new stopM(); }

        private class stopR implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullRight.setPower(0); return false; } }
        public Action stopR(){ return new stopR(); }

        private class stopL implements Action { @Override public boolean run(@NonNull TelemetryPacket p){ pullLeft.setPower(0); return false; } }
        public Action stopL(){ return new stopL(); }
    }

    /* ──────────────── main opmode ──────────────── */
    @Override
    public void runOpMode() throws InterruptedException {
        // hardware
        intake     = hardwareMap.get(DcMotor.class, "intake");
        tiltRight  = hardwareMap.get(Servo.class, "tiltRight");
        tiltMid    = hardwareMap.get(Servo.class, "tiltMid");
        tiltLeft   = hardwareMap.get(Servo.class, "tiltLeft");
        flipper    = hardwareMap.get(Servo.class, "flipper");

        pullRight  = hardwareMap.get(CRServo.class,"pullRight");
        pullMid    = hardwareMap.get(CRServo.class,"pullMid");
        pullLeft   = hardwareMap.get(CRServo.class,"pullLeft");

        leftSwitch  = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        midSwitch   = hardwareMap.get(DigitalChannel.class, "midSwitch");
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");

        // preset tilts + idle powers
        tiltLeft.setPosition(LEFT_TILT_PRE);
        tiltMid.setPosition(MID_TILT_PRE);
        tiltRight.setPosition(RIGHT_TILT_PRE);
        pullLeft.setPower(SHOOT_IDLE_LEFT);
        pullMid.setPower(SHOOT_IDLE_MID);
        pullRight.setPower(SHOOT_IDLE_RIGHT);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(INIT_SCAN_PIPELINE);
        limelight.start();

        // INIT: scan tags (pipeline 2) to choose order
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
                    telemetry.addData("INIT Tags", ids);
                    telemetry.addData("Primary", chosenInitPrimaryId);
                } else telemetry.addLine("INIT: no tags");
            } else telemetry.addLine("INIT: no result");

            pickOrderFromInit(chosenInitPrimaryId);
            telemetry.addData("Order", shootOrder.toString());
            telemetry.addData("LL pipeline", "%d (%s)", st.getPipelineIndex(), st.getPipelineType());
            telemetry.update();
            idle();
        }

        // switch to shooting pipeline
        limelight.pipelineSwitch(APRILTAG_PIPELINE);

        // RR setup
        Pose2d initialPose = new Pose2d(61, 12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot();

        // ===== Drive + intake + return to shot pose (keep your commented block intact) =====
        Action DRIVE_AND_RETURN = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(55, 12), Math.toRadians(330))
                .stopAndAdd(robot.posservo3())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(45, 29, Math.toRadians(120)), Math.toRadians(90))
                .afterDisp(0, robot.intake())
                .strafeToLinearHeading(new Vector2d(31, 48), Math.toRadians(90))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, 12, Math.toRadians(330)), Math.toRadians(0))
                .afterDisp(0, robot.intake_stop())

                /*  ──────────────── YOUR PREVIOUSLY COMMENTED TRAJECTORIES (kept) ────────────────
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(12, 48, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, 11, Math.toRadians(350)), Math.toRadians(0))
                .afterDisp(0, robot.outtake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, 48, Math.toRadians(180)), Math.toRadians(180))
                .afterDisp(0,robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-12, 48, Math.toRadians(180)), Math.toRadians(180))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, 11, Math.toRadians(350)), Math.toRadians(0))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, 24, Math.toRadians(0)), Math.toRadians(180))
                ──────────────────────────────────────────────────────────────────────────────── */

                .build();

        // post-shoot move (unchanged)
        Action POST_SHOOT = drive.actionBuilder(new Pose2d(55, 12, Math.toRadians(330)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(42, 39, Math.toRadians(180)), Math.toRadians(90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // 1) Do the pathing to (55,12,330)
        Actions.runBlocking(new SequentialAction(DRIVE_AND_RETURN));

        // 2) Micro-aim with Limelight until lock
        Actions.runBlocking(new AimWithLL(drive));

        // 3) Fire in INIT-decided order
        if (!REQUIRE_LOCK_TO_SHOOT || opModeIsActive()) {
            fireByOrder();
        }

        // 4) Reposition
        Actions.runBlocking(POST_SHOOT);

        limelight.stop();
    }
}
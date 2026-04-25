package org.firstinspires.ftc.teamcode.literally_everything_else;


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

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Autonomous(name = "farAutoBlueLaviathan", group = "competition")
public class farAutoBlueLaviathan extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private static final int INIT_SCAN_PIPELINE  = 2; // DURING init
    private DcMotor intake;
    public Servo flipper;
    public Servo tiltLeft;
    public Servo tiltMid ;
    public Servo tiltRight;
    public CRServo pullRight;
    public CRServo pullMid;
    public CRServo pullLeft;
    public boolean lswitchPressed = false;
    public boolean mswitchPressed = false;
    public boolean rswitchPressed = false;
    public DigitalChannel leftSwitch;
    public DigitalChannel midSwitch;
    public DigitalChannel rightSwitch;

    // Limelight + init-scan order
    private Limelight3A limelight;
    private enum SidePre { LEFT, MIDDLE, RIGHT }
    private enum SideSpi1 { LEFT, MIDDLE, RIGHT }
    private enum SideSpi2 { LEFT, MIDDLE, RIGHT }
    private final java.util.List<SidePre> shootOrderPre = new java.util.ArrayList<>();
    private final java.util.List<SideSpi1> shootOrderSpi1 = new java.util.ArrayList<>();
    private final java.util.List<SideSpi2> shootOrderSpi2 = new java.util.ArrayList<>();
    private int chosenInitPrimaryId = -1;

    private void pickOrderFromInit(int id) {
        shootOrderPre.clear();
        if (id == 22) {              // L, M, R
            shootOrderPre.add(SidePre.LEFT);
            shootOrderPre.add(SidePre.MIDDLE);
            shootOrderPre.add(SidePre.RIGHT);
        } else if (id == 21) {       // M, R, L
            shootOrderPre.add(SidePre.MIDDLE);
            shootOrderPre.add(SidePre.RIGHT);
            shootOrderPre.add(SidePre.LEFT);
        } else if (id == 23) {       // R, L, M
            shootOrderPre.add(SidePre.RIGHT);
            shootOrderPre.add(SidePre.LEFT);
            shootOrderPre.add(SidePre.MIDDLE);
        } else {                     // fallback
            shootOrderPre.add(SidePre.RIGHT);
            shootOrderPre.add(SidePre.LEFT);
            shootOrderPre.add(SidePre.MIDDLE);
        }
    }
    private void pickOrderFromSpike1(int id) {
        shootOrderSpi1.clear();
        if (id == 22) {              // L, M, R
            shootOrderSpi1.add(SideSpi1.LEFT);
            shootOrderSpi1.add(SideSpi1.RIGHT);
            shootOrderSpi1.add(SideSpi1.MIDDLE);
        } else if (id == 21) {       // M, R, L
            shootOrderSpi1.add(SideSpi1.RIGHT);
            shootOrderSpi1.add(SideSpi1.MIDDLE);
            shootOrderSpi1.add(SideSpi1.LEFT);
        } else if (id == 23) {       // R, L, M
            shootOrderSpi1.add(SideSpi1.LEFT);
            shootOrderSpi1.add(SideSpi1.MIDDLE);
            shootOrderSpi1.add(SideSpi1.RIGHT);
        } else {                     // fallback
            shootOrderSpi1.add(SideSpi1.RIGHT);
            shootOrderSpi1.add(SideSpi1.LEFT);
            shootOrderSpi1.add(SideSpi1.MIDDLE);
        }
    }

    private void pickOrderFromSpike2(int id) {
        shootOrderSpi2.clear();
        if (id == 22) {              // L, M, R
            shootOrderSpi2.add(SideSpi2.RIGHT);
            shootOrderSpi2.add(SideSpi2.LEFT);
            shootOrderSpi2.add(SideSpi2.MIDDLE);
        } else if (id == 21) {       // M, R, L
            shootOrderSpi2.add(SideSpi2.LEFT);
            shootOrderSpi2.add(SideSpi2.MIDDLE);
            shootOrderSpi2.add(SideSpi2.RIGHT);
        } else if (id == 23) {       // R, L, M
            shootOrderSpi2.add(SideSpi2.RIGHT);
            shootOrderSpi2.add(SideSpi2.MIDDLE);
            shootOrderSpi2.add(SideSpi2.LEFT);
        } else {                     // fallback
            shootOrderSpi2.add(SideSpi2.LEFT);
            shootOrderSpi2.add(SideSpi2.RIGHT);
            shootOrderSpi2.add(SideSpi2.MIDDLE);
        }
    }



    private String orderToStringPre(java.util.List<SidePre> seq) {
        StringBuilder sb = new StringBuilder();
        for (SidePre s : seq) sb.append(s == SidePre.LEFT ? "L" : (s == SidePre.RIGHT ? "R" : "M")).append(' ');
        return sb.toString().trim();
    }
    private String orderToStringSpi1(java.util.List<SideSpi1> seq) {
        StringBuilder sb = new StringBuilder();
        for (SideSpi1 s : seq) sb.append(s == SideSpi1.LEFT ? "L" : (s == SideSpi1.RIGHT ? "R" : "M")).append(' ');
        return sb.toString().trim();
    }
    private String orderToStringSpi2(java.util.List<SideSpi2> seq) {
        StringBuilder sb = new StringBuilder();
        for (SideSpi2 s : seq) sb.append(s == SideSpi2.LEFT ? "L" : (s == SideSpi2.RIGHT ? "R" : "M")).append(' ');
        return sb.toString().trim();
    }







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
                tiltLeft.setPosition(0.370);
                tiltMid.setPosition(0.380);
                tiltRight.setPosition(0.400);
                return false;
            }
        }
        public Action posservo2() { return new posservo2(); }

        private class posservo3 implements Action {
            private boolean leftDone = false;
            private boolean midDone = false;
            private boolean rightDone = false;
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.33);
                // In your Robot class

                // Set tilting servos to the correct position for winching.
                tiltLeft.setPosition(0.03);
                tiltMid.setPosition(0.03);
                tiltRight.setPosition(0.03);

                // --- Left Catapult ---
                // getState() is false when the switch is pressed.
                if (leftSwitch.getState() == true) {
                    leftDone = true;
                    pullLeft.setPower(.2); // Stop the motor
                } else {
                    pullLeft.setPower(1.0); // Run motor backwards to winch
                }

                // --- Middle Catapult ---
                if (midSwitch.getState() == true) {
                    midDone = true;
                    pullMid.setPower(.2); // Stop the motor
                } else {
                    pullMid.setPower(1.0); // Run motor backwards to winch
                }

                // --- Right Catapult ---
                if (rightSwitch.getState() == true) {
                    rightDone = true;
                    pullRight.setPower(.2); // Stop the motor
                } else {
                    pullRight.setPower(1.0); // Run motor backwards to winch
                }

                // The action is complete only when all three switches have been pressed.
                // Return 'true' to continue running, 'false' when done.
                boolean isRunning = !(leftDone && midDone && rightDone);
                return isRunning;
            }
        }

        public Action posservo3() { return new posservo3(); }

        private class thing1 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.38);
                return false;
            }
        }
        public Action thing1() { return new thing1(); }

        private class thing2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.48);
                return false;
            }
        }
        public Action thing2() { return new thing2(); }

        private class thing3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.75);
                return false;
            }
        }
        public Action thing3() { return new thing3(); }


        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class intake_stop implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                return false;
            }
        }
        public Action intake_stop() { return new intake_stop(); }

        private class outtake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(-1);
                return false;
            }
        }
        public Action outtake() { return new outtake(); }

        private class fireM implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullMid.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireM() { return new fireM(); }

        private class fireR implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullRight.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireR() { return new fireR(); }

        private class fireL implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullLeft.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireL() { return new fireL(); }

        private class stopM implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullMid.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopM() { return new stopM(); }

        private class stopR implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullRight.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopR() { return new stopR(); }

        private class stopL implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullLeft.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopL() { return new stopL(); }

        private class init implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.18);
                tiltLeft.setPosition( 0.370);
                tiltMid.setPosition(  0.380);
                tiltRight.setPosition(0.400);
                pullLeft.setPower(.2);
                pullMid.setPower(.2);
                pullRight.setPower(.2);
                return false;
            }
        }
        public Action init() { return new init(); }

        // NEW: fire by order determined in INIT scan
        private class shootByOrderPreload implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                for (int i = 0; i < shootOrderPre.size(); i++) {
                    SidePre s = shootOrderPre.get(i);
                    if (s == SidePre.LEFT) {
                        pullLeft.setPower(1);
                        sleep(600);
                        pullLeft.setPower(0);
                    } else if (s == SidePre.MIDDLE) {
                        pullMid.setPower(1);
                        sleep(600);
                        pullMid.setPower(0);
                    } else {
                        pullRight.setPower(1);
                        sleep(600);
                        pullRight.setPower(0);
                    }
                    if (i != shootOrderPre.size()-1)
                        sleep(300);
                }
                return false;
            }
        }
        public Action shootByOrderPreload() { return new shootByOrderPreload(); }

        private class shootByOrderSpike1 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                for (int i = 0; i < shootOrderSpi1.size(); i++) {
                    SideSpi1 s = shootOrderSpi1.get(i);
                    if (s == SideSpi1.LEFT) {
                        pullLeft.setPower(1);
                        sleep(500);
                        pullLeft.setPower(0);
                    } else if (s == SideSpi1.MIDDLE) {
                        pullMid.setPower(1);
                        sleep(500);
                        pullMid.setPower(0);
                    } else {
                        pullRight.setPower(1);
                        sleep(500);
                        pullRight.setPower(0);
                    }
                    if (i != shootOrderSpi1.size()-1)
                        sleep(300);
                }
                return false;
            }
        }
        public Action shootByOrderSpike1() { return new shootByOrderSpike1(); }

        private class shootByOrderSpike2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                for (int i = 0; i < shootOrderSpi2.size(); i++) {
                    SideSpi2 s = shootOrderSpi2.get(i);
                    if (s == SideSpi2.LEFT) {
                        pullLeft.setPower(1);
                        sleep(500);
                        pullLeft.setPower(0);
                    } else if (s == SideSpi2.MIDDLE) {
                        pullMid.setPower(1);
                        sleep(500);
                        pullMid.setPower(0);
                    } else {
                        pullRight.setPower(1);
                        sleep(500);
                        pullRight.setPower(0);
                    }
                    if (i != shootOrderSpi2.size()-1)
                        sleep(300);
                }
                return false;
            }
        }
        public Action shootByOrderSpike2() { return new shootByOrderSpike2(); }

        // ── NEW: Non-spinning AprilTag aim (pipeline 0). If no tag, finishes immediately.
        private class aimRedTag implements Action {
            private final MecanumDrive drive;
            private boolean inited = false;
            private long lastNs = 0L;
            private long holdMs = 0L;
            private double prevErrDeg = 0.0;

            // Tunables (kept local so nothing else in your file changes)
            private static final double AIM_KP = 0.045;
            private static final double AIM_KD = 0.0035;
            private static final double AIM_TOL_DEG = 2.0;
            private static final double AIM_MIN_CMD = 0.08;
            private static final double AIM_YAW_MAX = 0.55;
            private static final double AIM_OFFSET_DEG = -10;   // adjust if you want a bias
            private static final long   LOCK_HOLD_MS = 150;

            aimRedTag(MecanumDrive d) { this.drive = d; }

            @Override public boolean run(@NonNull TelemetryPacket p) {
                if (!inited) {
                    if (limelight != null) limelight.pipelineSwitch(1);
                    lastNs = System.nanoTime();
                    holdMs = 0L;
                    prevErrDeg = 0.0;
                    inited = true;
                }

                LLResult rr = (limelight != null) ? limelight.getLatestResult() : null;

                boolean haveTag = false;
                double  txDeg   = 0.0;

                if (rr != null) {
                    java.util.List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
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

                long now = System.nanoTime();
                double dt = Math.max((now - lastNs) / 1e9, 1e-3);
                lastNs = now;

                if (!haveTag) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                    p.put("aim", "no tag visible — skipping");
                    return false; // finish immediately, no spin
                }

                double err  = txDeg - AIM_OFFSET_DEG;
                double derr = (err - prevErrDeg) / dt;
                prevErrDeg  = err;

                double cmd = -(AIM_KP * err + AIM_KD * derr);

                if (Math.abs(err) <= AIM_TOL_DEG) {
                    cmd = 0.0;
                    holdMs += (long)(dt * 1000.0);
                } else {
                    holdMs = 0L;
                    if (Math.abs(cmd) < AIM_MIN_CMD) cmd = Math.copySign(AIM_MIN_CMD, cmd);
                }

                double yaw = Math.max(-AIM_YAW_MAX, Math.min(AIM_YAW_MAX, cmd));
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), yaw));
                p.put("aim_tx", err);
                p.put("aim_hold_ms", holdMs);
                p.put("aim_yawCmd", yaw);

                if (holdMs >= LOCK_HOLD_MS) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
                    return false; // done (locked)
                }
                return true; // keep aiming
            }
        }
        public Action aimRedTag(MecanumDrive d) { return new aimRedTag(d); }


    }


    /* ──────────────── main opmode ──────────────── */

    @Override
    public void runOpMode() throws InterruptedException {

        /* ---- initialize hardware ---- */
        intake = hardwareMap.get(DcMotor.class, "intake");
        tiltRight = hardwareMap.get(Servo.class, "tiltRight");
        tiltMid   = hardwareMap.get(Servo.class, "tiltMid");
        tiltLeft  = hardwareMap.get(Servo.class, "tiltLeft");
        flipper  = hardwareMap.get(Servo.class, "flipper");

        // ---- Three pull servos (catapults) ----
        pullRight = hardwareMap.get(CRServo.class,"pullRight");
        pullMid   = hardwareMap.get(CRServo.class,"pullMid");
        pullLeft  = hardwareMap.get(CRServo.class,"pullLeft");
        leftSwitch  = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        midSwitch   = hardwareMap.get(DigitalChannel.class, "midSwitch");
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");

        // *** NEW: create Robot early and run the init action ONCE (no runBlocking) ***
        Robot robotEarly = new Robot();
        robotEarly.init().run(new TelemetryPacket());

        /* ---- Limelight init + INIT SCAN LOOP ---- */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.pipelineSwitch(INIT_SCAN_PIPELINE);
        limelight.start();

        while (opModeInInit() && !isStopRequested()) {
            LLStatus st = limelight.getStatus();
            LLResult rr = limelight.getLatestResult();

            if (rr != null) {
                java.util.List<LLResultTypes.FiducialResult> tags = rr.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    java.util.List<Integer> ids = new java.util.ArrayList<>(tags.size());
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

            pickOrderFromInit(chosenInitPrimaryId);
            pickOrderFromSpike1(chosenInitPrimaryId);
            pickOrderFromSpike2(chosenInitPrimaryId);
            telemetry.addData("Planned Order preload", orderToStringPre(shootOrderPre));
            telemetry.addData("Planned Order spike", orderToStringSpi1(shootOrderSpi1));
            telemetry.addData("Planned Order spike", orderToStringSpi2(shootOrderSpi2));
            telemetry.addData("LL Pipeline", "%d (%s)", st.getPipelineIndex(), st.getPipelineType());
            telemetry.update();
            idle();
        }

        /* ---- traj / action setup ---- */
        Pose2d initialPose = new Pose2d(61,              -9.5, Math.toRadians( -0  ));
        Pose2d endOfPreload = new Pose2d(50,             -14 , Math.toRadians( -340));
        Pose2d endOfgoGrab1 = new Pose2d(40,             -47 , Math.toRadians( -180));
        Pose2d endOfgoGrab1Nopre = new Pose2d(65,        -70 , Math.toRadians( -90 ));
        Pose2d endOfgoGrab1NopreSPECIAL = new Pose2d(65, -70 , Math.toRadians( -90 ));
        Pose2d endOfscoreSpike1 = new Pose2d(50,         -14 , Math.toRadians( -340));
        Pose2d endOfgoGrab2 = new Pose2d(20,             -51 , Math.toRadians( -180));
        Pose2d endOfscoreSpike2 = new Pose2d(50,         -14 , Math.toRadians( -340));
        Pose2d endOfgoGrab3 = new Pose2d(-8,             -47 , Math.toRadians( -180));
        Pose2d endOfscoreSpike3 = new Pose2d(50,         -14 , Math.toRadians( -340));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot();

//.afterDisp(0, robot.thing3())
        Action preload = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-340))
                .stopAndAdd(robot.aimRedTag(drive))
                .waitSeconds(.4)
                .stopAndAdd(robot.shootByOrderPreload())
                .build();

        Action goGrab1 = drive.actionBuilder(endOfPreload)
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(57, -34, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(62, -57), Math.toRadians(-90))
                .waitSeconds(3)
                .stopAndAdd(robot.intake())
                .strafeToLinearHeading(new Vector2d(65, -70), Math.toRadians(-90))
                .build();

        Action goGrab1Nopre = drive.actionBuilder(endOfscoreSpike1)
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(57, -34, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(65, -55), Math.toRadians(-90))
                .stopAndAdd(robot.intake())
                .strafeToLinearHeading(new Vector2d(65, -70), Math.toRadians(-90))
                .build();

        Action goGrab1Nopre2 = drive.actionBuilder(endOfscoreSpike1)
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(57, -34, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(62, -57), Math.toRadians(-90))
                .waitSeconds(3)
                .strafeToLinearHeading(new Vector2d(62, -70), Math.toRadians(-90))
                .stopAndAdd(robot.intake())
                .build();


        Action scoreSpike1 = drive.actionBuilder(endOfgoGrab1Nopre)
                .afterDisp(0, robot.intake())
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-340))
                .afterDisp(1, robot.init())
                .stopAndAdd(robot.aimRedTag(drive))
                .waitSeconds(.4)
                .stopAndAdd(robot.shootByOrderSpike2())
                .build();

        Action scoreSpike12 = drive.actionBuilder(endOfgoGrab1NopreSPECIAL)
                .afterDisp(0, robot.intake_stop())
                .afterDisp(0, robot.init())
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-340))
                .stopAndAdd(robot.aimRedTag(drive))
                .waitSeconds(.4)
                .stopAndAdd(robot.shootByOrderSpike2())
                .build();

        Action scoreSpike13 = drive.actionBuilder(endOfgoGrab1NopreSPECIAL)
                .afterDisp(0, robot.intake_stop())
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-340))
                .afterDisp(1, robot.init())
                .stopAndAdd(robot.aimRedTag(drive))
                .waitSeconds(.4)
                .stopAndAdd(robot.shootByOrderSpike2())
                .build();


        Action goGrab2 = drive.actionBuilder(endOfscoreSpike1)
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40, -51, Math.toRadians(-180)), Math.toRadians(-180))
                .stopAndAdd(robot.intake())
                .strafeToLinearHeading(new Vector2d(20, -51), Math.toRadians(-180))
                .build();

        Action scoreSpike2 = drive.actionBuilder(endOfgoGrab2)
                .afterDisp(0, robot.intake_stop())
                .afterDisp(0, robot.init())
                .strafeToLinearHeading(new Vector2d(50, -14), Math.toRadians(-340))
                .stopAndAdd(robot.aimRedTag(drive))
                .waitSeconds(.4)
                .stopAndAdd(robot.shootByOrderPreload())
                .build();

        /*Action goGrab3 = drive.actionBuilder(endOfPreload)
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(10, 20, Math.toRadians(180)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(10, 55), Math.toRadians(180))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(10, 47), Math.toRadians(180))
                .stopAndAdd(robot.intake())
                .strafeToLinearHeading(new Vector2d(-8, 47), Math.toRadians(180))
                .build();

        Action scoreSpike3 = drive.actionBuilder(endOfgoGrab3)
                .afterDisp(0, robot.intake_stop())
                .afterDisp(0, robot.init())
                .strafeToLinearHeading(new Vector2d(50, 0), Math.toRadians(340))
                .stopAndAdd(robot.shootByOrderSpike2())
                .build();*/

        Action offLine = drive.actionBuilder(endOfscoreSpike1)
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(57, -34, Math.toRadians(-90)), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(65, -55), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(65, -70), Math.toRadians(-90))
                .build();


        /*   INIT ACTIONS                                 */
        // (Removed runBlocking here; init already executed before the scan loop)


        waitForStart();

        if (opModeIsActive() && limelight != null) {
            limelight.pipelineSwitch(0);
        }

        if (isStopRequested()) return;
        if (isStopRequested()){
        }


        /* ---- main autonomous ---- */
        Actions.runBlocking(new SequentialAction(
                preload,
                goGrab1,
                scoreSpike1,
                goGrab1Nopre,
                scoreSpike12,
                goGrab1Nopre2,
                scoreSpike13,
                offLine


        ));

        /* ---- cleanup ---- */
        limelight.stop();
    }
}
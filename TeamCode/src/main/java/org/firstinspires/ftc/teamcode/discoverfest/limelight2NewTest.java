package org.firstinspires.ftc.teamcode.discoverfest;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "new test run this one this one not another one please this one is the copy of the original do not run a different one its this one we are testing because of the catapult thats not working", group = "TeleOp")
public class limelight2NewTest extends LinearOpMode {

    // ---------- Limelight ----------
    private static final int APRILTAG_PIPELINE = 0;

    // ---- Pipeline toggle state ----
    private int currentPipeline = APRILTAG_PIPELINE;
    private boolean prevDpadLeft = false;

    // ---- Quick snap-turn on pipeline switch ----
    private static final double TOGGLE_TURN_POWER = -0.45;
    private static final double TOGGLE_TURN_MS    = 550;

    // ---------- Catapult auto-burst ----------
    private static final double CAT_BURST_MS      = 500;
    private static final double CAT_COOLDOWN_MS   = 2500.0;
    private static final double CAT_BURST_POWER   = 1.0;
    private static final double CAT_IDLE_POWER    = 0.10;

    // ---------- Aim Assist (PD on tx) ----------
    private static final double AIM_KP = 0.04;
    private static final double AIM_KD = 0.002;
    private static final double AIM_TOL_DEG = 1.0;
    private static final double AIM_YAW_MAX = 0.6;
    private static final double AIM_MIN_CMD = 0.08;

    // ---------- Distance via vertical angle (ty) ----------
    private static final double CAMERA_HEIGHT_M  = 0.22;
    private static final double TAG_HEIGHT_M     = 0.75;
    private static final double CAMERA_PITCH_DEG = 76.75;
    private static final double M_TO_IN          = 39.3700787;

    // ---- Distance trigger targets (inches) ----
    private static final double TRG1_IN = 7.7;
    private static final double TRG2_IN = 8.0;
    private static final double DIST_TOL_IN = 0.2;

    private final ElapsedTime runtime  = new ElapsedTime();
    private final ElapsedTime aimTimer = new ElapsedTime();

    // Tag 24 trigger memory
    private boolean prevSaw24 = false;

    // Burst state (shared for auto)
    private boolean catBurstActive = false;
    private double  catBurstEndMs  = 0.0;
    private double  nextAllowedFireMs = 0.0;

    // Mode toggles
    private boolean autoFireEnabled = false; // start MANUAL
    private boolean prevY = false;

    private boolean aimAssistEnabled = true; // dpad_right toggles
    private boolean prevAimToggleBtn = false;

    // PD state
    private double prevErrDeg = 0.0;

    // ===== Vision-only “stay put” hold (NO IMU/encoders) =====
    private static final boolean VISION_HOLD_ENABLED = true;
    private static final double  VH_TX_KP  = 0.035;
    private static final double  VH_TY_KP  = 0.000;
    private static final double  VH_MAX    = 0.30;
    private static final double  STICK_DEAD = 0.07;

    private boolean holdActive = false;
    private Double  holdTxBaseline = null;
    private Double  holdTyBaseline = null;

    // ===== Intake (gamepad2) =====
    private DcMotor intake;
    private static final double INTAKE_MAX = 1.0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {


        // ---- Drive ----
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        DigitalChannel leftSwitch  = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        DigitalChannel midSwitch   = hardwareMap.get(DigitalChannel.class, "midSwitch");
        DigitalChannel rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");

        // ---- Three separate tilters ----
        Servo tiltRight = hardwareMap.get(Servo.class, "tiltRight");
        Servo tiltMid   = hardwareMap.get(Servo.class, "tiltMid");
        Servo tiltLeft  = hardwareMap.get(Servo.class, "tiltLeft");
        Servo flipper  = hardwareMap.get(Servo.class, "flipper");

        // ---- Three pull servos (catapults) ----
        CRServo pullRight = hardwareMap.get(CRServo.class,"pullRight");
        CRServo pullMid   = hardwareMap.get(CRServo.class,"pullMid");
        CRServo pullLeft  = hardwareMap.get(CRServo.class,"pullLeft");

        // ---- Intake motor (NEW) ----
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0.0);



        // ---- Limelight ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        // ---- Drive setup ----
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // starting tilts

        double  tiltLeftD  = 0.420,
                tiltMidD   = 0.380,
                tiltRightD = 0.410;
        boolean lswitchPressed = false;
        boolean mswitchPressed = false;
        boolean rswitchPressed = false;
        boolean prevLB = false, prevRB = false;
        boolean prevDU = false, prevDD = false;
        boolean prevLS = false, prevRS = false;
        boolean midReverseHold = false;
        ElapsedTime midBtnTimer = new ElapsedTime();
        final long MID_HOLD_MS = 300;
        final double STEP = 0.01;

        // --- TOUCHPAD STATE MACHINE ---
        // 0 = normal (bumps work)
        // 1 = flat  (all 0.03)
        // 2 = preset (L=.440, M=.430, R=.370)
        int tiltMode = 0;
        boolean tpressed = false;
        boolean prevTouchpad = false;
        double savedTiltLeft  = tiltLeftD;
        double savedTiltMid   = tiltMidD;
        double savedTiltRight = tiltRightD;

        // Idle catapults
        //pullRight.setPower(CAT_IDLE_POWER);
        //pullMid.setPower(CAT_IDLE_POWER);
        //pullLeft.setPower(CAT_IDLE_POWER);

        // apply initial tilts
        //tiltRight.setPosition(tiltRightD);
        //tiltMid.setPosition(tiltMidD);
        //tiltLeft.setPosition(tiltLeftD);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        runtime.reset();
        aimTimer.reset();

        while (opModeIsActive()) {
            // ================== MODE TOGGLES ==================
            boolean yBtn = false; // you had this false in your version
            if (yBtn && !prevY) {
                autoFireEnabled = !autoFireEnabled;
                if (!autoFireEnabled) {
                    catBurstActive = false;
                    pullRight.setPower(CAT_IDLE_POWER);
                    pullMid.setPower(CAT_IDLE_POWER);
                    pullLeft.setPower(CAT_IDLE_POWER);
                }
            }
            prevY = yBtn;

            boolean aimToggleBtn = gamepad1.dpad_right;
            if (aimToggleBtn && !prevAimToggleBtn) {
                aimAssistEnabled = !aimAssistEnabled;
            }
            prevAimToggleBtn = aimToggleBtn;

            // ===== TOUCHPAD STATE MACHINE (1 or 2 pads) =====


            // ================== PIPELINE TOGGLE (dpad-left) ==================
            boolean dpadLeft = gamepad1.dpad_left;
            if (dpadLeft && !prevDpadLeft) {
                currentPipeline = (currentPipeline == 0) ? 1 : 0;
                limelight.pipelineSwitch(currentPipeline);

                double turnDir = (currentPipeline == 1) ? 1.0 : -1.0;
                double endMs = runtime.milliseconds() + TOGGLE_TURN_MS;

                while (opModeIsActive() && runtime.milliseconds() < endMs) {
                    double yawSpin = TOGGLE_TURN_POWER * turnDir;
                    frontLeft.setPower( yawSpin);
                    frontRight.setPower(-yawSpin);
                    backLeft.setPower(  -yawSpin);
                    backRight.setPower(yawSpin);
                    telemetry.addData("ToggleTurn", "Spinning %s", turnDir > 0 ? "+45°" : "-45°");
                    telemetry.update();
                }
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            prevDpadLeft = dpadLeft;

            // ================== LIMELIGHT ==================
            LLStatus st = limelight.getStatus();
            telemetry.addData("Mode", autoFireEnabled ? "AUTO" : "MANUAL");
            telemetry.addData("AimAssist", aimAssistEnabled ? "ON" : "OFF");
            telemetry.addData("LL Pipeline (req/actual)",
                    "%d / %d (%s)", currentPipeline, st.getPipelineIndex(), st.getPipelineType());

            LLResult result = limelight.getLatestResult();
            boolean saw24 = false;
            boolean haveTag = false;

            boolean distLaunch = false;
            double  lastDistIn = Double.NaN;

            double aimErrorDeg = 0.0;

            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    haveTag = true;

                    List<Integer> ids = new ArrayList<>(tags.size());
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult fr : tags) {
                        int id = fr.getFiducialId();
                        ids.add(id);
                        if (id == 24) saw24 = true;
                        if (fr.getTargetArea() > best.getTargetArea()) best = fr;
                    }
                    telemetry.addData("Tag IDs", ids);
                    telemetry.addData("Primary Tag ID", best.getFiducialId());

                    double tyDeg;
                    try { tyDeg = best.getTargetYDegrees(); }
                    catch (Throwable t) { tyDeg = result.getTy(); }

                    double cameraPitchRad = Math.toRadians(CAMERA_PITCH_DEG);
                    double tyRad          = Math.toRadians(tyDeg);
                    double deltaH = TAG_HEIGHT_M - CAMERA_HEIGHT_M;
                    double denom  = Math.tan(cameraPitchRad + tyRad);

                    Double distPlanarM = null;
                    if (Math.abs(denom) > 1e-6) distPlanarM = deltaH / denom;

                    if (distPlanarM != null && distPlanarM > 0 && distPlanarM < 10) {
                        double lastDistM = distPlanarM;
                        lastDistIn = lastDistM * M_TO_IN;
                        telemetry.addData("Distance XY (ty)", "%.2f m  (%.1f in)", lastDistM, lastDistIn);

                        if (Math.abs(lastDistIn - TRG1_IN) <= DIST_TOL_IN
                                || Math.abs(lastDistIn - TRG2_IN) <= DIST_TOL_IN) {
                            distLaunch = true;
                        }
                    } else {
                        telemetry.addLine("Distance (ty): out of range or unavailable — check heights/pitch.");
                    }

                    double txDeg;
                    try { txDeg = best.getTargetXDegrees(); }
                    catch (Throwable t) { txDeg = result.getTx(); }
                    aimErrorDeg = txDeg;
                    telemetry.addData("tx (deg)", "%.2f", txDeg);
                    telemetry.addData("ty (deg)", "%.2f", tyDeg);
                } else {
                    telemetry.addLine("No AprilTags detected.");
                }
            } else {
                telemetry.addLine("No Limelight result yet…");
            }

            // --- Auto FIRE triggers ---
            double nowMs = runtime.milliseconds();
            if (autoFireEnabled) {
                boolean saw24Edge = saw24 && !prevSaw24;

                if (!catBurstActive && nowMs >= nextAllowedFireMs) {
                    if (saw24Edge || distLaunch) {
                        catBurstActive = true;
                        catBurstEndMs  = nowMs + CAT_BURST_MS;
                        nextAllowedFireMs = nowMs + CAT_COOLDOWN_MS;
                        telemetry.addLine("** TRIGGER -> Triple Catapult BURST START (AUTO)**");
                    }
                }
            }
            prevSaw24 = saw24;

            // ================== MECANUM DRIVE (auto-aim yaw) ==================
            double axial     =  gamepad1.left_stick_y;
            double lateral   = -gamepad1.left_stick_x;
            double yawManual = -gamepad1.right_stick_x;

            double yawCmd = yawManual;

            final double AIM_OFFSET_DEG_LOCAL = 1.0;

            if (aimAssistEnabled && haveTag) {
                double dt = Math.max(aimTimer.seconds(), 1e-3);
                aimTimer.reset();

                double err  = (aimErrorDeg - AIM_OFFSET_DEG_LOCAL);
                double derr = (err - prevErrDeg) / dt;

                double yawAuto = -(AIM_KP * err + AIM_KD * derr);

                if (Math.abs(err) <= AIM_TOL_DEG) {
                    yawAuto = 0.0;
                } else {
                    double sgn = Math.signum(yawAuto);
                    if (Math.abs(yawAuto) < AIM_MIN_CMD) yawAuto = sgn * AIM_MIN_CMD;
                }

                yawAuto = Range.clip(yawAuto, -AIM_YAW_MAX, AIM_YAW_MAX);

                if (Math.abs(yawManual) < 0.15) {
                    yawCmd = yawAuto;
                }

                prevErrDeg = err;
                telemetry.addData("Aim err(tx)-off", "%.2f", err);
                telemetry.addData("Aim yawAuto", "%.2f", yawAuto);
            } else {
                prevErrDeg *= 0.9;
                aimTimer.reset();
            }

            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max; fr /= max; bl /= max; br /= max;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ================== INTAKE CONTROL (gamepad1 + gamepad2) ==================
            double rt = gamepad1.right_trigger + gamepad2.right_trigger;   // combined
            double lt = gamepad1.left_trigger + gamepad2.left_trigger;

            double intakeCmd = rt - lt;

            // BUMPER OVERRIDES (either controller)
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = 1.0;
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = -1.0;
            }

            // final clip + apply
            intakeCmd = Range.clip(intakeCmd, -INTAKE_MAX, INTAKE_MAX);
            intake.setPower(intakeCmd);
            // ================== TILTER BUMPS (gamepad1) ==================
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;
            if (tiltMode == 0) {
                if (lb && !prevLB && !rb) {
                    tiltLeftD = Range.clip(tiltLeftD + STEP, 0.0, 1.0);
                } else if (rb && !prevRB && !lb) {
                    tiltLeftD = Range.clip(tiltLeftD - STEP, 0.0, 1.0);
                }
            }
            prevLB = lb; prevRB = rb;

            boolean dUp = gamepad1.dpad_up;
            boolean dDn = gamepad1.dpad_down;
            if (tiltMode == 0) {
                if (dUp && !prevDU && !dDn) {
                    tiltRightD = Range.clip(tiltRightD + STEP, 0.0, 1.0);
                } else if (dDn && !prevDD && !dUp) {
                    tiltRightD = Range.clip(tiltRightD - STEP, 0.0, 1.0);
                }
            }
            prevDU = dUp; prevDD = dDn;

            boolean ls = gamepad1.left_stick_button;
            boolean rs = gamepad1.right_stick_button;

            if (ls && !prevLS) {
                midBtnTimer.reset();
                midReverseHold = false;
            }
            if (ls && !midReverseHold && midBtnTimer.milliseconds() > MID_HOLD_MS) {
                midReverseHold = true;
            }
            if (tiltMode == 0) {
                if (rs && !prevRS && !ls) {
                    tiltMidD = Range.clip(tiltMidD + STEP, 0.0, 1.0);
                }
                if (!ls && prevLS && !midReverseHold) {
                    tiltMidD = Range.clip(tiltMidD - STEP, 0.0, 1.0);
                }
            }
            prevLS = ls; prevRS = rs;

            // Push positions each loop
            tiltRight.setPosition(tiltRightD);
            tiltMid.setPosition(tiltMidD);
            tiltLeft.setPosition(tiltLeftD);

            boolean lswitch = leftSwitch.getState();
            boolean mswitch = midSwitch.getState();
            boolean rswitch = rightSwitch.getState();


            boolean touchpad = gamepad1.touchpad || gamepad2.touchpad;
            if (touchpad && !prevTouchpad) {
                tiltMode = (tiltMode + 1) % 2;   // 0 -> 1 -> 2 -> 0
                tpressed = true;
            } else if (!touchpad) {
                tpressed = false;
            }

            switch (tiltMode) {
                /*case 0:
                    //flipper.setPosition(.38);
                    tiltLeftD  = 0.290;
                    tiltMidD   = 0.320;
                    tiltRightD = 0.310;
                    break;*/
                
                case 0:
                    // flipper.setPosition(.48);
                    // Preset ONCE when you enter state 1 (your “tilt mode 2”), then allow bumps to change
                    if (tpressed) {
                        tiltLeftD  = 0.420;
                        tiltMidD   = 0.290;
                        tiltRightD = 0.420;
                    }
                    break;

                case 1:
                    // going FLAT (unchanged from your version; still presets every loop)
                    tiltLeftD  = 0.03;
                    tiltMidD   = 0.03;
                    tiltRightD = 0.03;
                    break;
            }


            prevTouchpad = touchpad;
            // ================== CATAPULT CONTROL (SEPARATE; 3x) ==================
            if (catBurstActive && autoFireEnabled) {
                if (nowMs < catBurstEndMs) {
                    pullRight.setPower(CAT_BURST_POWER);
                    pullMid.setPower(CAT_BURST_POWER);
                    pullLeft.setPower(CAT_BURST_POWER);
                    telemetry.addData("Catapults", "AUTO BURST (%.0f ms left)", catBurstEndMs - nowMs);
                } else {
                    catBurstActive = false;
                    pullRight.setPower(CAT_IDLE_POWER);
                    pullMid.setPower(CAT_IDLE_POWER);
                    pullLeft.setPower(CAT_IDLE_POWER);
                }
            } else {

                // 1) Handle the "any switch pressed" case (exactly like before)
                boolean blockedBySwitches = false;


                if (tiltMode == 1 & lswitch) {
                    flipper.setPosition(.3);
                } else {
                    flipper.setPosition(.18 );
                }

                if (lswitch & tiltMode == 1) {
                    blockedBySwitches = true;
                    lswitchPressed = true;
                    pullLeft.setPower(CAT_IDLE_POWER);
                } else if (tiltMode == 1 & !lswitchPressed) {
                    pullLeft.setPower(1.0);

                }
                if (mswitch & tiltMode == 1) {
                    blockedBySwitches = true;
                    mswitchPressed = true;
                    pullMid.setPower(CAT_IDLE_POWER);
                } else if (tiltMode == 1 & !mswitchPressed) {
                    pullMid.setPower(1.0);

                }

                if (rswitch & tiltMode == 1) {
                    blockedBySwitches = true;
                    rswitchPressed = true;
                    pullRight.setPower(CAT_IDLE_POWER);
                } else  if (tiltMode == 1 & !rswitchPressed ) {
                    pullRight.setPower(1.0);

                }

                // 2) Independently update the latched flags (don’t block buttons)
                if (!lswitch) lswitchPressed = false;
                if (!mswitch) mswitchPressed = false;
                if (!rswitch) rswitchPressed = false;

                // 3) If not blocked by the switch hold, run your button logic
                if (!blockedBySwitches && tiltMode != 1) {
                    // ----- RIGHT side -----
                    if (gamepad2.dpad_up & tiltMode != 1) {
                        pullRight.setPower(-1.0);
                    } else if (gamepad1.b & tiltMode != 1) {
                        pullRight.setPower(1.0);

                        // ----- MID side -----
                    } else if (gamepad2.dpad_right & tiltMode != 1) {
                        pullMid.setPower(-1.0);
                    } else if (gamepad1.a & tiltMode != 1) {
                        pullMid.setPower(1.0);

                        // ----- LEFT side -----
                    } else if (gamepad2.dpad_left & tiltMode != 1) {
                        pullLeft.setPower(-1.0);
                    } else if (gamepad1.x & tiltMode != 1) {
                        pullLeft.setPower(1.0);

                        // ----- Fire all 3 (respect per-side inhibits) -----
                    } else if (gamepad1.y & tiltMode != 1) {
                        pullLeft .setPower(1.0);
                        pullMid  .setPower(1.0);
                        pullRight.setPower(1.0);

                        // ----- Flat-mode safety (unchanged logic) -----
                    } else {
                        pullRight.setPower(CAT_IDLE_POWER);
                        pullMid.setPower(CAT_IDLE_POWER);
                        pullLeft.setPower(CAT_IDLE_POWER);
                    }




                }



            }

            // ================== TELEMETRY ==================
            telemetry.addData("Drive FL/FR", "%.2f / %.2f", fl, fr);
            telemetry.addData("Drive BL/BR", "%.2f / %.2f", bl, br);
            telemetry.addData("tiltMode", tiltMode); // 0=normal,1=flat,2=preset
            telemetry.addData("tiltLeft",  "%.3f", tiltLeft.getPosition());
            telemetry.addData("tiltMid",   "%.3f", tiltMid.getPosition());
            telemetry.addData("tiltRight", "%.3f", tiltRight.getPosition());
            telemetry.addData("IntakeCmd", "%.2f", intakeCmd);
            telemetry.addData("limit switch l", leftSwitch.getState());
            telemetry.addData("limit switch m", midSwitch.getState());
            telemetry.addData("limit switch r", rightSwitch.getState());
            telemetry.addData("AimAssist", aimAssistEnabled ? "ON" : "OFF");
            telemetry.addData("AimErr(tx)", "%.2f", aimErrorDeg);
            telemetry.update();
        }

        limelight.stop();
    }
}
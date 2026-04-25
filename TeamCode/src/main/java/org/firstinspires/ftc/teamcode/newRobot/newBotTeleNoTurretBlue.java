package org.firstinspires.ftc.teamcode.newRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.ArrayList;
import java.util.List;
@Disabled
@Config
@TeleOp(name = "novus umbra No turret Blue", group = "TeleOp")
/* opmode*/
public class newBotTeleNoTurretBlue extends LinearOpMode {

    // ================== DASH TUNABLES ==================
    public static boolean REVERSE_TURRET = false;

    // Manual turret controls
    public static double MANUAL_DEADBAND = 0.15;
    public static double TURRET_MANUAL_SCALE = 1.0;

    // ---- Hood ----
    public static double HOOD_START_POS = .8;
    public static double HOOD_STEP = 0.10;

    // ---- Flywheel ----
    public static double TARGET_RPM = 3600;
    public static double RPM_TO_TPS = (TARGET_RPM * 28) / 60;
    public static double VEL_START = (RPM_TO_TPS);
    public static double VEL_STEP = 46.6666666667;
    public static double VEL_MAX = 100000;

    // ---- gecko ----
    public static double GECKO_INTAKE_POWER = 1.0;
    public static double GECKO_REVERSE_POWER = -1.0;
    public static double GECKO_IDLE_POWER = 0.0;

    // ---- Intake ----
    public static double INTAKE_MAX = 1.0;

    // ================== TURRET (MANUAL ONLY) ==================
    public static double TURRET_TICKS_PER_REV = 418.557;

    // Hard turret limits (robot-relative). Encoder must be zeroed at center for these to be meaningful.
    public static double TURRET_MIN_DEG = -180.0;
    public static double TURRET_MAX_DEG = 180.0;

    // ================== HELPERS ==================
    private double ticksToDeg(double ticks) {
        return (ticks * 360.0) / TURRET_TICKS_PER_REV;
    }

    private double applyHardLimits(double currentTurretDegRaw, double pwr) {
        if (currentTurretDegRaw >= TURRET_MAX_DEG && pwr > 0) return 0.0;
        if (currentTurretDegRaw <= TURRET_MIN_DEG && pwr < 0) return 0.0;
        return pwr;
    }

    private static final int APRILTAG_PIPELINE = 1;

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
    private static final double AIM_KP = 0.01;
    private static final double AIM_KD = 0.002;
    private static final double AIM_TOL_DEG = 3.0;
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

    // Tag 20 trigger memory
    private boolean prevSaw20 = false;

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
    GoBildaPrismDriver prism;

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
    PrismAnimations.Rainbow rainbow = new PrismAnimations.Rainbow();
    PrismAnimations.PoliceLights policeLights = new PrismAnimations.PoliceLights();
    PrismAnimations.Sparkle sparkle = new PrismAnimations.Sparkle();
    PrismAnimations.Random random = new PrismAnimations.Random();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {


        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood = hardwareMap.get(Servo.class, "hood");
        DcMotorEx gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        Servo   light      = hardwareMap.get(Servo.class, "light");
        Servo   turret      = hardwareMap.get(Servo.class, "turret");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");

        random.setBrightness(100);
        random.setSpeed(1f);

        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);


        //turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //turret.setDirection(REVERSE_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        //turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double hoodpos = HOOD_START_POS;
        double turretpos = .5;
        double vel = VEL_START;

        boolean prevLB = false, prevRB = false;
        boolean prevxx = false, prevbb = false;
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        boolean saw20 = false;
        boolean haveTag = false;

        boolean distLaunch = false;
        double aimErrorDeg = 0.0;

        waitForStart();

        while (opModeIsActive()) {
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));

            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, random);

            haveTag = false;
            saw20 = false;
            distLaunch = false;

            LLResult llResult = limelight.getLatestResult();
            double distanceFromLimelightToGoalInches = 0;

            /* aim */ {
            if (llResult != null && llResult.isValid()) {

                double targetOffsetAngle_Vertical = llResult.getTy();

                // how many degrees back is your limelight rotated from perfectly vertical?
                double limelightMountAngleDegrees = 20;

                // distance from the center of the Limelight lens to the floor
                double limelightLensHeightInches = 13.84425;

                // distance from the target to the floor
                double goalHeightInches = 29.5;

                double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
                double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                //calculate distance
                distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            }



            boolean aimToggleBtn = gamepad1.dpad_right;
            if (aimToggleBtn && !prevAimToggleBtn) {
                aimAssistEnabled = !aimAssistEnabled;
            }
            prevAimToggleBtn = aimToggleBtn; }

            // cc these have to be out cuz brackets ¯\_(ツ)_/¯
            double goodvelfar = (962.1592 + 25.87152 * distanceFromLimelightToGoalInches - 0.3023253 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches + 0.001353474 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches);
            double goodvelclose = 7.926 * distanceFromLimelightToGoalInches + 1129.12;

            /* pipeline toggle      */ {
                boolean dpadLeft = gamepad1.dpad_left;
                if (dpadLeft && !prevDpadLeft) {
                    currentPipeline = (currentPipeline == 1) ? 0 : 1;
                    limelight.pipelineSwitch(currentPipeline);

                    double turnDir = (currentPipeline == 0) ? -1 : 1;
                    double endMs = runtime.milliseconds() + TOGGLE_TURN_MS;

                    while (opModeIsActive() && runtime.milliseconds() < endMs) {
                        double yawSpin = TOGGLE_TURN_POWER * turnDir;
                        frontLeft.setPower(yawSpin);
                        frontRight.setPower(-yawSpin);
                        backLeft.setPower(-yawSpin);
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
            }

            /* majority of ll stuff */ {
            LLStatus st = limelight.getStatus();
            telemetry.addData("Mode", autoFireEnabled ? "AUTO" : "MANUAL");
            telemetry.addData("AimAssist", aimAssistEnabled ? "ON" : "OFF");
            telemetry.addData("LL Pipeline (req/actual)",
                    "%d / %d (%s)", currentPipeline, st.getPipelineIndex(), st.getPipelineType());

            LLResult result = limelight.getLatestResult();

            double lastDistIn = Double.NaN;



            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {
                    haveTag = true;

                    List<Integer> ids = new ArrayList<>(tags.size());
                    LLResultTypes.FiducialResult best = tags.get(0);
                    for (LLResultTypes.FiducialResult fr : tags) {
                        int id = fr.getFiducialId();
                        ids.add(id);
                        if (id == 20) saw20 = true;
                        if (fr.getTargetArea() > best.getTargetArea()) best = fr;
                    }
                    telemetry.addData("Tag IDs", ids);
                    telemetry.addData("Primary Tag ID", best.getFiducialId());

                    double tyDeg;
                    try {
                        tyDeg = best.getTargetYDegrees();
                    } catch (Throwable t) {
                        tyDeg = result.getTy();
                    }

                    double cameraPitchRad = Math.toRadians(CAMERA_PITCH_DEG);
                    double tyRad = Math.toRadians(tyDeg);
                    double deltaH = TAG_HEIGHT_M - CAMERA_HEIGHT_M;
                    double denom = Math.tan(cameraPitchRad + tyRad);

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
                    try {
                        txDeg = best.getTargetXDegrees();
                    } catch (Throwable t) {
                        txDeg = result.getTx();
                    }
                    aimErrorDeg = txDeg;
                    telemetry.addData("tx (deg)", "%.2f", txDeg);
                    telemetry.addData("ty (deg)", "%.2f", tyDeg);
                } else {
                    telemetry.addLine("No AprilTags detected.");
                }
            } else {
                telemetry.addLine("No Limelight result yet…");
            } }

            /* auto FIRE not AIM    */ {
            double nowMs = runtime.milliseconds();
            if (autoFireEnabled) {
                boolean saw20Edge = saw20 && !prevSaw20;

                if (!catBurstActive && nowMs >= nextAllowedFireMs) {
                    if (saw20Edge || distLaunch) {
                        catBurstActive = true;
                        catBurstEndMs = nowMs + CAT_BURST_MS;
                        nextAllowedFireMs = nowMs + CAT_COOLDOWN_MS;
                        telemetry.addLine("** TRIGGER -> Triple Catapult BURST START (AUTO)**");
                    }
                }
            }
            prevSaw20 = saw20; }

            /* drive and aim        */ {
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawManual = -gamepad1.right_stick_x;

            double yawCmd = yawManual;


            if (aimAssistEnabled && haveTag) {
                double dt = Math.max(aimTimer.seconds(), 1e-3);
                aimTimer.reset();

                double err = (aimErrorDeg - AIM_OFFSET_DEG_LOCAL);
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

                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;

                frontLeft.setPower(fl);
                frontRight.setPower(fr);
                backLeft.setPower(bl);
                backRight.setPower(br);
            }

            /* flywheel             */ {

            if (gamepad1.x) {
                var = 1;
            } else if (gamepad1.y) {
                var = 0;
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            }


            if (var == 1 && distanceFromLimelightToGoalInches < 90 && saw20) {
                flywheel1.setVelocity(goodvelclose);
                flywheel2.setVelocity(goodvelclose);
                AIM_OFFSET_DEG_LOCAL = -2;
            } else if (var == 1 && distanceFromLimelightToGoalInches > 90 && saw20) {
                flywheel1.setVelocity(goodvelfar);
                flywheel2.setVelocity(goodvelfar);
                AIM_OFFSET_DEG_LOCAL = -5;
            }
            /*if (xx && !prevxx && !bb) {
                vel = Range.clip(vel + VEL_STEP, 0.0, VEL_MAX);
            }

            if (bb && !prevbb && !xx) {
                vel = Range.clip(vel - 46.6666666667, 0.0, VEL_MAX);
            }

            prevxx = xx;
            prevbb = bb;
             boolean xx = gamepad1.x;
            boolean bb = gamepad1.b;
            */
        }

            /* light                */ {
            if (Math.abs(goodvelclose - flywheel1.getVelocity()) < 40 && distanceFromLimelightToGoalInches >= 90){
                light.setPosition(.5);
            } else if (Math.abs(goodvelfar - flywheel1.getVelocity()) < 40 && distanceFromLimelightToGoalInches < 90){
                light.setPosition(.5);
            } else {
                light.setPosition(.686);
            }}

            /* hood                 */ {
                hood.setPosition(hoodpos);
            /*
            boolean lb = gamepad1.dpad_down;
            boolean rb = gamepad1.dpad_up;

            if (lb && !prevLB && !rb) {
                hoodpos = Range.clip(hoodpos + .01, 0.0, 1.0);
            } else if (rb && !prevRB && !lb) {
                hoodpos = Range.clip(hoodpos - .01, 0.0, 1.0);
            }

            prevLB = lb;
            prevRB = rb;*/

                hood.setPosition(hoodpos);

                // hoodpos = (-0.1800692 + 0.04033266 * distanceFromLimelightToGoalInches - 0.0004901035 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches + 0.000002004714 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches);
                hoodpos = 0.0050505050505 * distanceFromLimelightToGoalInches + 0.394;
                hoodpos = Range.clip(hoodpos, .5, 1.0);
            }

            /* intake               */ {
            double rt = gamepad1.right_trigger + gamepad2.right_trigger;
            double lt = gamepad1.left_trigger + gamepad2.left_trigger;
            double intakeCmd = rt - lt;

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;
            }

            intakeCmd = Range.clip(intakeCmd, -INTAKE_MAX, INTAKE_MAX);

            if (intakeCmd > 0 && intakeCmd <= 1) {
                if (gamepad1.left_bumper) {
                    gecko.setPower(GECKO_REVERSE_POWER);
                } else {
                    gecko.setPower(GECKO_INTAKE_POWER);
                }
            } else if (intakeCmd < 0 && intakeCmd >= -1) {
                if (gamepad1.right_bumper) {
                    gecko.setPower(GECKO_INTAKE_POWER);
                } else {
                    gecko.setPower(GECKO_REVERSE_POWER);
                }
            } else {
                gecko.setPower(GECKO_IDLE_POWER);
            }

            intake.setPower(intakeCmd); }

            /* turret               */ {
           /* turret.setDirection(REVERSE_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            double turretRobotDegRaw = ticksToDeg(turret.getCurrentPosition());

            // right stick: right = +power (you can flip sign if you want)
            double turretManual = -gamepad2.right_stick_x * TURRET_MANUAL_SCALE;

            double turretPower = 0.0;
            if (Math.abs(turretManual) >= MANUAL_DEADBAND) {
                turretPower = Range.clip(turretManual, -1.0, 1.0);
                turretPower = applyHardLimits(turretRobotDegRaw, turretPower);
            }
            turret.setPower(turretPower);*/

                turret.setPosition(turretpos);


                boolean lb = gamepad1.dpad_down;
                boolean rb = gamepad1.dpad_up;

                if (lb && !prevLB && !rb) {
                    turretpos = Range.clip(turretpos + .1, 0.0, 1.0);
                } else if (rb && !prevRB && !lb) {
                    turretpos = Range.clip(turretpos - .1, 0.0, 1.0);
                }

                prevLB = lb;
                prevRB = rb;}

            /* telemetry            */ {
            // commented cuz no turret --> telemetry.addData("turretDegRaw", "%.2f", turretRobotDegRaw);
            // commented cuz no turret --> telemetry.addData("turretManual", "%.2f", turretManual);
            // commented cuz no turret --> telemetry.addData("turretPower", "%.2f", turretPower);
            telemetry.addData("hood", "%.3f", hood.getPosition());
            telemetry.addData("hoodpos", "%.3f", hoodpos);
            telemetry.addData("turret", "%.3f", turret.getPosition());
            telemetry.addData("turretpos", "%.3f", turretpos);
            telemetry.addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
            telemetry.addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
            telemetry.addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 28));
            telemetry.addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 28));
            telemetry.addData("target vel", "%.3f", vel);
            telemetry.addData("Distance inches", "%.3f", distanceFromLimelightToGoalInches);
            telemetry.addData("Target y", llResult.getTy());
            telemetry.update();}

            /* dashboard            */ {
            dashboard.getTelemetry().addData("REVERSE_TURRET", REVERSE_TURRET);
            // commented cuz no turret --> dashboard.getTelemetry().addData("turretDegRaw", turretRobotDegRaw);
            // commented cuz no turret --> dashboard.getTelemetry().addData("turretPower", turretPower);
            dashboard.getTelemetry().update(); }
        }

       // turret.setPower(0);
    }
}
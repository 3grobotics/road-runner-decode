package org.firstinspires.ftc.teamcode.current;

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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Subsystems.turret;

import java.util.ArrayList;
import java.util.List;
@Disabled

@Config
@TeleOp(name = "teleop Red", group = "TeleOp")
/* opmode*/
public class teleopRed extends LinearOpMode {

    // ================== DASH TUNABLES ==================
    public static boolean REVERSE_TURRET = false;

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

    // Tag 24 trigger memory
    private boolean prevSaw24 = false;

    // Burst state (shared for auto)
    private boolean catBurstActive = false;
    private double  catBurstEndMs  = 0.0;
    private double  nextAllowedFireMs = 0.0;

    // Mode toggles
    private boolean autoFireEnabled = false; // start MANUAL

    private boolean aimAssistEnabled = true; // dpad_right toggles
    private boolean prevAimToggleBtn = false;

    // ===== Intake (gamepad2) =====
    GoBildaPrismDriver prism;

    PrismAnimations.Random random = new PrismAnimations.Random();

    private turret turret;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        turret = new turret(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood = hardwareMap.get(Servo.class, "hood");

        Servo   light      = hardwareMap.get(Servo.class, "light");
        Servo   turret      = hardwareMap.get(Servo.class, "turret");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        hood.setDirection(Servo.Direction.REVERSE);

        random.setBrightness(100);
        random.setSpeed(.001f);

        //DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        double oldTime = 0;
        double target_y = 72;
        double target_x = -72;

        //turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //turret.setDirection(REVERSE_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        //turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(-42, -90, DistanceUnit.MM);

        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        odo.setOffsets(42, -33.55, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        turret.setDirection(Servo.Direction.REVERSE);

        double hoodposFar = HOOD_START_POS;
        double hoodposClose = HOOD_START_POS;
        double turretpos = .5;
        double vel = VEL_START;

        boolean prevLB = false, prevRB = false;
        boolean prevxx = false, prevbb = false;
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        boolean saw24 = false;

        boolean distLaunch = false;
        double aimErrorDeg = 0.0;
        int twistState = 0;
        boolean dPressed = false;
        waitForStart();

        while (opModeIsActive()) {
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));

            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, random);

            saw24 = false;
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
                prevAimToggleBtn = aimToggleBtn;
            }

            // 1. Get targets relative to the robot's current position on the field
            double xleg = target_x - odo.getPosX(DistanceUnit.INCH);
            double yleg = target_y - odo.getPosY(DistanceUnit.INCH);

            // 2. Calculate the "Field Centric" angle to the target
            // Math.atan2 returns -180 to +180 degrees
            double targetFieldHeading = Math.toDegrees(Math.atan2(yleg, xleg));

            // 3. Calculate angle relative to the robot
            // We subtract the robot's heading to make 0 degrees "Straight Forward"
            double robotHeading = odo.getHeading(AngleUnit.DEGREES);
            double relativeAngle = targetFieldHeading - robotHeading;

            // 4. Handle wrapping so relativeAngle is strictly between -180 and 180
            // This ensures the turret takes the shortest path
            while (relativeAngle > 180)  relativeAngle -= 360;
            while (relativeAngle < -180) relativeAngle += 360;

            // 5. Offset for Servo Alignment
            // Your servo is centered (facing forward) at 0.5.
            // 0.5 * 303 degrees = 197.5 degrees.
            // So, when relativeAngle is 0 (forward), servoDegrees should be 197.5.
            double servoDegrees = relativeAngle + 151.5;

            // 6. Clamp safety for the 0-303 range
            // If the target is directly behind you, the math might output < 0 or > 303.
            // We clamp it so the servo doesn't try to go past its physical stops.
            if (servoDegrees < 0) servoDegrees = 0;
            if (servoDegrees > 303) servoDegrees = 303;

            // 7. Set Position
            turret.setPosition(servoDegrees / 303.0);

            //cc FIX THIS GAGE -Gage
            /*if (Math.abs(odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)) > 250){
                //turret.setPosition((servoDegrees / 303.0) - .2);
                target_x = target_x - 10;
                target_y = target_y - 10;
            } else {
                target_x = -72;
                target_y =  72;
            }*/

            double hypot = Math.sqrt((xleg * xleg) + (yleg * yleg));

            // cc these have to be out cuz brackets ¯\_(ツ)_/¯
            double goodvelfar = (12.8014 * hypot + 219.62);
            double goodvelclose = 5.46849 * hypot + 1123.13;

            /* pipeline toggle      */ {
                boolean dpadLeft = gamepad1.dpad_left;
                if (dpadLeft && !prevDpadLeft) {
                    currentPipeline = (currentPipeline == 0) ? 1 : 0;
                    limelight.pipelineSwitch(currentPipeline);

                    double turnDir = (currentPipeline == 1) ? 1.0 : -1.0;
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

                if (result != null) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (tags != null && !tags.isEmpty()) {

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
                            double lastDistIn = lastDistM * M_TO_IN;
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
                }
            }

            /* auto FIRE not AIM    */ {
                double nowMs = runtime.milliseconds();
                if (autoFireEnabled) {
                    boolean saw24Edge = saw24 && !prevSaw24;

                    if (!catBurstActive && nowMs >= nextAllowedFireMs) {
                        if (saw24Edge || distLaunch) {
                            catBurstActive = true;
                            catBurstEndMs = nowMs + CAT_BURST_MS;
                            nextAllowedFireMs = nowMs + CAT_COOLDOWN_MS;
                            telemetry.addLine("** TRIGGER -> Triple Catapult BURST START (AUTO)**");
                        }
                    }
                }
                prevSaw24 = saw24;
            }

            /* drive and aim        */ {
                double axial = gamepad1.left_stick_y;
                double lateral = -gamepad1.left_stick_x;
                double yawManual = -gamepad1.right_stick_x;

                double yawCmd = yawManual;

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

                boolean xx = gamepad1.x;
                boolean bb = gamepad1.b;

                if (gamepad1.x) {
                    var = 1;
                } else if (gamepad1.y) {
                    var = 0;
                    flywheel1.setVelocity(0);
                    flywheel2.setVelocity(0);
                }
                if (gamepad1.right_stick_button && gamepad1.left_stick_button && !dPressed) {
                    twistState = (twistState + 1) % 2;
                    dPressed = true;
                } else if (!gamepad2.dpad_up) {
                    dPressed = false;
                }
                switch (twistState){
                    case 0:


                        if (gamepad1.x) {
                            var = 1;
                        } else if (gamepad1.y) {
                            var = 0;
                            flywheel1.setVelocity(0);
                            flywheel2.setVelocity(0);
                        }


                        if (var == 1 && hypot < 90 && saw24) {
                            flywheel1.setVelocity(goodvelclose);
                            flywheel2.setVelocity(goodvelclose);
                            AIM_OFFSET_DEG_LOCAL = -2;
                        } else if (var == 1 && hypot > 90 && saw24) {
                            flywheel1.setVelocity(goodvelfar);
                            flywheel2.setVelocity(goodvelfar);
                            AIM_OFFSET_DEG_LOCAL = -5;
                        }
                        break;
                    case 1:

                        flywheel1.setVelocity(vel);
                        flywheel2.setVelocity(vel);
                        if (xx && !prevxx && !bb) {
                            vel = Range.clip(vel + VEL_STEP, 0.0, VEL_MAX);
                        }

                        if (bb && !prevbb && !xx) {
                            vel = Range.clip(vel - 46.6666666667, 0.0, VEL_MAX);
                        }
                        break;
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
                }
            }

            /* hood                 */ {

                if (hypot < 100) {
                    hood.setPosition(hoodposClose);
                } else if (hypot > 100) {
                    hood.setPosition(hoodposFar);
                }
                // hoodpos = (-0.1800692 + 0.04033266 * distanceFromLimelightToGoalInches - 0.0004901035 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches + 0.000002004714 * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches * distanceFromLimelightToGoalInches);
                hoodposClose = 0.0054099 * hypot + 0.226097;
                hoodposFar = 0.00399002 * hypot + 0.345731;
                hoodposClose = Range.clip(hoodposClose, .5, 1.0);
                hoodposFar = Range.clip(hoodposFar, .5, 1.0);
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

                intake.setPower(intakeCmd);
            }

            odo.update();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            /* telemetry            */ {
                telemetry.addData("hood", "%.3f", hood.getPosition());
                telemetry.addData("hoodposClose", "%.3f", hoodposClose);
                telemetry.addData("hoodposFar", "%.3f", hoodposFar);
                telemetry.addData("turret", "%.3f", turret.getPosition());
                telemetry.addData("turretpos", "%.3f", turretpos);
                telemetry.addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
                telemetry.addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
                telemetry.addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 28));
                telemetry.addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 28));
                telemetry.addData("target vel", "%.3f", vel);
                telemetry.addData("Distance inches", "%.3f", distanceFromLimelightToGoalInches);
                telemetry.addData("Target y", llResult.getTy());
                telemetry.addData("Distance hypot", hypot);
                telemetry.addData("x leg", xleg);
                telemetry.addData("y leg", yleg);
                telemetry.addData("odo x in", odo.getPosX(DistanceUnit.INCH));
                telemetry.addData("heading vel", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                telemetry.addData("odo y in", odo.getPosY(DistanceUnit.INCH));
                telemetry.update();
            }

            /* dashboard            */ {
                dashboard.getTelemetry().addData("REVERSE_TURRET", REVERSE_TURRET);
                dashboard.getTelemetry().update();
            }
        }

        //turret.setPower(0);
    }
}
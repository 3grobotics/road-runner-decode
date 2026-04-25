package org.firstinspires.ftc.teamcode.newRobot;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.Register;

import java.util.List;
@Disabled
@Config
@TeleOp(name = "novus umbra", group = "TeleOp")
public class limelight2new extends LinearOpMode {

    // ================== DASH TUNABLES ==================
    // ---- Turret / Limelight ----
    public static boolean AIM_ENABLED_DEFAULT = true;
    public static boolean REVERSE_TURRET = false;
    public static int PIPELINE = 0;

    // Manual override / controls
    public static double MANUAL_DEADBAND = 0.15;
    public static double TURRET_MANUAL_SCALE = 1.0;

    // ---- Keep these (not used for turret anymore, but you had them on dash) ----
    public static double ALIGN_KP = 0.030;
    public static double ALIGN_MIN_POWER = 0.12;
    public static double ALIGN_MAX_POWER = 0.60;
    public static double HOLD_TOL_DEG = 1.0;
    public static double AIM_OFFSET_DEG = 0.0;

    // tx smoothing (now used as measurement smoothing)
    public static boolean USE_TX_FILTER = true;
    public static double TX_ALPHA = 0.35;

    public static boolean SEARCH_WHEN_LOST = false;
    public static double SEARCH_POWER = 0.12;

    // reduce dashboard spam (can help loop rate)
    public static int DASH_DECIM = 3;

    // ---- Hood ----
    public static double HOOD_START_POS = .8;
    public static double HOOD_STEP = 0.10;

    // ---- Flywheel ----
    public static double TARGET_RPM = 3600;
    public static double RPM_TO_TPS = (TARGET_RPM * 28) / 60;
    public static double VEL_START = (RPM_TO_TPS);
    public static double VEL_STEP = 46.6666666667;
    public static double VEL_MAX = 100000;

    // ---- intakeServo1 ----
    public static double GECKO_INTAKE_POWER = 1.0;
    public static double GECKO_REVERSE_POWER = -1.0;
    public static double GECKO_IDLE_POWER = 0.0;

    // ---- Intake ----
    public static double INTAKE_MAX = 1.0;

    // ================== TURRET (PINPOINT FIELD HOLD + ACQUIRE/HOLD) ==================
    public static double TURRET_TICKS_PER_REV = 418.557;

    // Hard turret limits (robot-relative). Encoder must be zeroed at center for these to be meaningful.
    public static double TURRET_MIN_DEG = -180.0;
    public static double TURRET_MAX_DEG = 180.0;

    // Turret position PD (robot-relative degrees -> power)
    public static double POS_KP = 0.018;
    public static double POS_KD = 0.0015;

    // Jitter killers (updated)
    public static double POS_TOL_DEG = 2.0;
    public static double MIN_POWER = 0.03;
    public static double MAX_POWER = 0.35;

    // Tag debounce (frame-based)
    public static int SEEN_FRAMES_ON = 2;
    public static int LOST_FRAMES_OFF = 6;

    // Limelight sign
    public static boolean TX_SIGN_NEGATIVE = true;

    // Pinpoint config
    public static double PINPOINT_X_OFFSET_MM = -84.0;
    public static double PINPOINT_Y_OFFSET_MM = -168.0;

    // --- Acquire & Hold anti-jitter ---
    public static double ACQUIRE_TX_MAX = 12.0;
    public static double ACQUIRE_TX_DEADBAND = 1.0;
    public static double REACQUIRE_IF_TX_OVER = 6.0;
    public static double ACQUIRE_COOLDOWN_SEC = 0.35;

    // ================== DISTANCE VIA TY (ADDED) ==================
    public static boolean ENABLE_DISTANCE = true;

    public static double CAMERA_HEIGHT_M  = 0.22;
    public static double TAG_HEIGHT_M     = 0.75;
    public static double CAMERA_PITCH_DEG = 76.75;

    public static double DIST_MIN_M = 0.05;
    public static double DIST_MAX_M = 10.0;

    public static double TRG1_IN = 7.7;
    public static double TRG2_IN = 8.0;
    public static double DIST_TOL_IN = 0.2;

    // ================== STATE ==================
    private GoBildaPinpointDriver odo;

    private double txFilt = 0.0;
    private boolean txFiltInit = false;

    private int seenCount = 0;
    private int lostCount = 0;
    private boolean stableSeen = false;

    private boolean prevStableSeen = false;
    private boolean haveLock = false;
    private final ElapsedTime acquireTimer = new ElapsedTime();

    private double holdFieldDeg = 0.0;
    private boolean holdInitialized = false;

    private double prevPosErr = 0.0;
    private final ElapsedTime turretLoopTimer = new ElapsedTime();

    private boolean distLaunch = false;

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

    // ================== HELPERS ==================
    private double ticksToDeg(double ticks) {
        return (ticks * 360.0) / TURRET_TICKS_PER_REV;
    }

    private double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg <= -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    private double applyHardLimits(double currentTurretDegRaw, double pwr) {
        if (currentTurretDegRaw >= TURRET_MAX_DEG && pwr > 0) return 0.0;
        if (currentTurretDegRaw <= TURRET_MIN_DEG && pwr < 0) return 0.0;
        return pwr;
    }

    private double metersToInches(double m) {
        return m * 39.3700787;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood     = hardwareMap.get(Servo.class, "hood");

        DcMotorEx gecko = hardwareMap.get(DcMotorEx.class, "gecko");

        Servo   turret      = hardwareMap.get(Servo.class, "turret");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        hood.setDirection(Servo.Direction.REVERSE);

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

        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        odo.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        double hoodpos = HOOD_START_POS;
        double vel = VEL_START;

        boolean prevLB = false, prevRB = false;
        boolean prevxx = false, prevbb = false;

        boolean aimEnabled = AIM_ENABLED_DEFAULT;
        boolean prevAimToggle = false;

        int lastPipeline = PIPELINE;
        int dashCount = 0;

        boolean bPressed = false;
        boolean dPressed = false;
         int twistState = 0;

        turretLoopTimer.reset();
        acquireTimer.reset();






        waitForStart();



        while (opModeIsActive()) {
            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, .07, .1, .01));


            double distanceFromLimelightToGoalInches = 0;
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose();

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
                telemetry.addData("distance inches", distanceFromLimelightToGoalInches);

            }

            if (PIPELINE != lastPipeline) {
                limelight.pipelineSwitch(PIPELINE);
                lastPipeline = PIPELINE;
            }

            //turret.setDirection(REVERSE_TURRET ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

            /* driving */ {
                double axial = gamepad1.left_stick_y;
                double lateral = -gamepad1.left_stick_x;
                double yaw = -gamepad1.right_stick_x;

                double fl = axial + lateral + yaw;
                double fr = axial - lateral - yaw;
                double bl = axial - lateral + yaw;
                double br = axial + lateral - yaw;

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

            /* hood stuff */ {
                hood.setPosition(hoodpos);

                boolean lb = gamepad1.dpad_down;
                boolean rb = gamepad1.dpad_up;

            if (lb && !prevLB && !rb) {
                hoodpos = Range.clip(hoodpos + .1, 0.0, 1.0);
            } else if (rb && !prevRB && !lb) {
                hoodpos = Range.clip(hoodpos - .1, 0.0, 1.0);
            }


                /*if (gamepad2.dpad_down && !bPressed) {
                    twistState = (twistState + 1) % 2;
                    bPressed = true;
                } else if (!gamepad2.dpad_down) {
                    bPressed = false;
                }
                switch (twistState) {
                    case 0:
                        hoodpos = .5;
                        break;
                    case 1:
                        hoodpos = 1;
                        break;
                }*/

                prevLB = lb;
                prevRB = rb;
            }

            /* flywheel and velocity stuff */ {
            if (gamepad1.x) {
                flywheel1.setVelocity(vel);
                flywheel2.setVelocity(vel);
            } else if (gamepad1.y) {
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            }

            boolean xx = gamepad1.x;
            boolean bb = gamepad1.b;

            if (xx && !prevxx && !bb) {
                vel = Range.clip(vel + VEL_STEP, 0.0, VEL_MAX);
            }

            if (bb && !prevbb && !xx) {
                vel = Range.clip(vel - 46.6666666667, 0.0, VEL_MAX);
            }

            prevxx = xx;
            prevbb = bb; }

            /* intake stuff */{
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


            boolean aimToggle = gamepad2.dpad_right;
            if (aimToggle && !prevAimToggle) {
                aimEnabled = !aimEnabled;
                txFiltInit = false;
                haveLock = false;
                acquireTimer.reset();
            }
            prevAimToggle = aimToggle;

            odo.update();

            double dt = turretLoopTimer.seconds();
            if (dt < 0.001) dt = 0.001;
            turretLoopTimer.reset();

            double robotHeadingDeg = odo.getPosition().getHeading(AngleUnit.DEGREES);
            //double turretRobotDegRaw = ticksToDeg(turret.getCurrentPosition());
            //double turretFieldDeg = wrapDeg(robotHeadingDeg + turretRobotDegRaw);

            //if (!holdInitialized) {
            //    holdFieldDeg = turretFieldDeg;
            //    holdInitialized = true;
            //}

            // ================== LIMELIGHT READ (TX + TY distance ADDED) ==================
            LLResult result = limelight.getLatestResult();

            boolean tagSeen = false;
            double tx = 0.0;

            distLaunch = false;

            if (result != null) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null && !tags.isEmpty()) {

                    LLResultTypes.FiducialResult best = tags.get(0);

                    // ✅ FIXED: don't use "fr" here (you already have drive "fr")
                    for (LLResultTypes.FiducialResult fid : tags) {
                        if (fid.getTargetArea() > best.getTargetArea()) best = fid;
                    }

                    tagSeen = true;

                    double txDeg;
                    try { txDeg = best.getTargetXDegrees(); }
                    catch (Throwable t) { txDeg = result.getTx(); }
                    tx = txDeg - AIM_OFFSET_DEG;




                }
            }
            Pose3D tz = result.getBotpose();
            double usedTx = tx;
            if (USE_TX_FILTER) {
                if (!txFiltInit) {
                    txFilt = tx;
                    txFiltInit = true;
                } else {
                    txFilt = (TX_ALPHA * tx) + ((1.0 - TX_ALPHA) * txFilt);
                }
                usedTx = txFilt;
            } else {
                txFiltInit = false;
            }



            if (tagSeen) { seenCount++; lostCount = 0; }
            else { lostCount++; seenCount = 0; }

            if (!stableSeen && seenCount >= SEEN_FRAMES_ON) stableSeen = true;
            if (stableSeen && lostCount >= LOST_FRAMES_OFF) stableSeen = false;

            double turretManual = -gamepad2.right_stick_x * TURRET_MANUAL_SCALE;

            double turretPower;
            double autoPower = 0.0;

            /*if (Math.abs(turretManual) >= MANUAL_DEADBAND) {
                turretPower = Range.clip(turretManual, -1.0, 1.0);
                turretPower = applyHardLimits(turretRobotDegRaw, turretPower);

                holdFieldDeg = turretFieldDeg;
                haveLock = false;
                acquireTimer.reset();

            } else {

                boolean risingSeen = stableSeen && !prevStableSeen;
                prevStableSeen = stableSeen;

                if (!stableSeen) {
                    haveLock = false;
                }

                if (aimEnabled && stableSeen) {

                    double txUsed = TX_SIGN_NEGATIVE ? (-usedTx) : (usedTx);

                    boolean wantAcquire =
                            risingSeen ||
                                    (!haveLock && acquireTimer.seconds() >= ACQUIRE_COOLDOWN_SEC) ||
                                    (haveLock && Math.abs(usedTx) > REACQUIRE_IF_TX_OVER
                                            && acquireTimer.seconds() >= ACQUIRE_COOLDOWN_SEC);

                    if (wantAcquire) {
                        if (Math.abs(usedTx) > ACQUIRE_TX_DEADBAND && Math.abs(usedTx) < ACQUIRE_TX_MAX) {
                            double targetFieldDeg = wrapDeg(turretFieldDeg + txUsed);
                            holdFieldDeg = targetFieldDeg;
                            haveLock = true;
                            acquireTimer.reset();
                        }
                    }
                }

                double targetTurretRobotDeg = wrapDeg(holdFieldDeg - robotHeadingDeg);
                targetTurretRobotDeg = Range.clip(targetTurretRobotDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

                double posErr = targetTurretRobotDeg - turretRobotDegRaw;
                double derr = (posErr - prevPosErr) / dt;
                prevPosErr = posErr;

                autoPower = (POS_KP * posErr) + (POS_KD * derr);

                if (Math.abs(posErr) <= POS_TOL_DEG) {
                    autoPower = 0.0;
                } else {
                    if (Math.abs(autoPower) < MIN_POWER) autoPower = (autoPower > 0) ? MIN_POWER : -MIN_POWER;
                }

                autoPower = Range.clip(autoPower, -MAX_POWER, MAX_POWER);
                autoPower = applyHardLimits(turretRobotDegRaw, autoPower);

                turretPower = autoPower;
            }*/

            turret.setPosition(.5);

            telemetry.addData("AimEnabled", aimEnabled);
            telemetry.addData("TagSeenRaw", tagSeen);
            telemetry.addData("TagSeenStable", stableSeen);
            telemetry.addData("haveLock", haveLock);
            telemetry.addData("tx raw", "%.2f", tx);
            telemetry.addData("tx used", "%.2f", usedTx);

            if (ENABLE_DISTANCE) {
                telemetry.addData("DistLaunch", distLaunch);
            }

            telemetry.addData("robotHeadingDeg", "%.2f", robotHeadingDeg);
            //telemetry.addData("turretRobotDegRaw", "%.2f", turretRobotDegRaw);
            //telemetry.addData("turretFieldDeg", "%.2f", turretFieldDeg);
            telemetry.addData("holdFieldDeg", "%.2f", holdFieldDeg);
            telemetry.addData("Distance in inches", "%.3f", distanceFromLimelightToGoalInches);

            // telemetry.addData("tz", "%.2f", botpose);

            telemetry.addData("TurretManual", "%.2f", turretManual);
            telemetry.addData("TurretAuto", "%.2f", autoPower);
            //telemetry.addData("TurretPower", "%.2f", turretPower);
            telemetry.addData("hood", "%.3f", hood.getPosition());
            telemetry.addData("hoodpos", "%.3f", hoodpos);
            telemetry.addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
            telemetry.addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
            telemetry.addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 28));
            telemetry.addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 28));
            telemetry.addData("target vel", "%.3f", vel);

            telemetry.update();

            dashCount++;
            if (DASH_DECIM < 1) DASH_DECIM = 1;



            if (dashCount >= DASH_DECIM) {
                dashCount = 0;

                dashboard.getTelemetry().addData("PIPELINE", PIPELINE);
                dashboard.getTelemetry().addData("REVERSE_TURRET", REVERSE_TURRET);
                dashboard.getTelemetry().addData("aimEnabled", aimEnabled);
                dashboard.getTelemetry().addData("tagSeenStable", stableSeen);
                dashboard.getTelemetry().addData("haveLock", haveLock);
                dashboard.getTelemetry().addData("tx_raw", tx);
                dashboard.getTelemetry().addData("tx_used", usedTx);

                if (ENABLE_DISTANCE) {
                    dashboard.getTelemetry().addData("distLaunch", distLaunch);
                }

                //dashboard.getTelemetry().addData("turretRobotDegRaw", turretRobotDegRaw);
                dashboard.getTelemetry().addData("holdFieldDeg", holdFieldDeg);
                dashboard.getTelemetry().addData("autoPower", autoPower);
                //dashboard.getTelemetry().addData("turretPower", turretPower);

                dashboard.getTelemetry().update();
            }
        }

        //turret.setPower(0);
        limelight.stop();
    }
}}
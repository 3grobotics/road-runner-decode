package org.firstinspires.ftc.teamcode.newRobot;

import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.Register;
@Disabled
@TeleOp(name="goBILDA® Pinpoint Odometry Example MINE NOT THE ORIGINAL", group="Linear OpMode")
public class SensorGoBildaPinpointExampleMINE extends LinearOpMode {

    GoBildaPinpointDriver odo;
    Limelight3A limelight;
    double oldTime = 0;

    // ----------------- TARGET: RED GOAL (FROM MEEPMEEP, INCHES) -----------------
    // MeepMeep coords are inches. Convert to mm to match Pinpoint.
    static final double INCH_TO_MM = 25.4;

    static final double TARGET_X_IN = 55.0;
    static final double TARGET_Y_IN = -60.0;

    static final double TARGET_X_MM_FIXED = TARGET_X_IN * INCH_TO_MM;
    static final double TARGET_Y_MM_FIXED = TARGET_Y_IN * INCH_TO_MM;

    // ----------------- OPTIONAL: LIVE RECORDED TARGET (PRESS X) -----------------
    // If you want to override the fixed MeepMeep point during testing, keep this.
    // Press X once to set the target to your current position.
    static double TARGET_X_MM = TARGET_X_MM_FIXED;
    static double TARGET_Y_MM = TARGET_Y_MM_FIXED;

    boolean lastX = false;

    Register[] defaultRegisters = {
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

    Register[] onlyPosition = {
            Register.DEVICE_STATUS,
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
    };

    private static double normalizeRadians(double r) {
        while (r > Math.PI) r -= 2.0 * Math.PI;
        while (r < -Math.PI) r += 2.0 * Math.PI;
        return r;
    }

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);

        odo.setOffsets(42, -33.55, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("MeepMeep Target (in)", "%.1f, %.1f", TARGET_X_IN, TARGET_Y_IN);
        telemetry.addData("Fixed Target (mm)", "%.1f, %.1f", TARGET_X_MM_FIXED, TARGET_Y_MM_FIXED);
        telemetry.addLine("Press X to OVERRIDE target to your current position");
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            odo.update();

            if (gamepad1.a) odo.resetPosAndIMU();
            if (gamepad1.b) odo.recalibrateIMU();
            if (gamepad1.dpad_down) odo.setBulkReadScope(onlyPosition);

            // Record target at current position when X is pressed (edge) - overrides fixed target
            boolean xNow = gamepad1.x;
            if (xNow && !lastX) {
                Pose2D p = odo.getPosition();
                TARGET_X_MM = p.getX(DistanceUnit.MM);
                TARGET_Y_MM = p.getY(DistanceUnit.MM);
            }
            lastX = xNow;

            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            double frequency = 1 / loopTime;
            oldTime = newTime;

            Pose2D pose = odo.getPosition();
            double robotX = pose.getX(DistanceUnit.MM);
            double robotY = pose.getY(DistanceUnit.MM);
            double robotHeadingRad = pose.getHeading(AngleUnit.RADIANS);

            double dx = TARGET_X_MM - robotX;
            double dy = TARGET_Y_MM - robotY;

            double distanceToTargetMM = Math.hypot(dx, dy);
            double angleToTargetRad = Math.atan2(dy, dx);
            double relativeAngleRad = normalizeRadians(angleToTargetRad - robotHeadingRad);
            LLResult llResult = limelight.getLatestResult();
            //Pose3D botpose = llResult.getBotpose();
            limelight.start();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose_MT2();
                Position thePosition = botpose.getPosition();
                telemetry.addData("Pos.X", thePosition.x * 1000);
                telemetry.addData("Pos.Y", thePosition.y * 1000);
                telemetry.addData("Pos.Z", thePosition.z);
            }


                telemetry.addData("Position", pose.toString());


            telemetry.addData("Robot (mm)", "%.1f, %.1f", robotX, robotY);
            telemetry.addData("Robot (in)", "%.1f, %.1f", robotX / INCH_TO_MM, robotY / INCH_TO_MM);
            telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));

            telemetry.addData("Target (mm)", "%.1f, %.1f", TARGET_X_MM, TARGET_Y_MM);
            telemetry.addData("Target (in)", "%.1f, %.1f", TARGET_X_MM / INCH_TO_MM, TARGET_Y_MM / INCH_TO_MM);

            telemetry.addData("dX (mm)", dx);
            telemetry.addData("dY (mm)", dy);

            telemetry.addData("Distance->Target (mm)", distanceToTargetMM);
            telemetry.addData("Angle->Target field (deg)", AngleUnit.DEGREES.fromRadians(angleToTargetRad));
            telemetry.addData("Turn->Target robot (deg)", AngleUnit.DEGREES.fromRadians(relativeAngleRad));

            telemetry.addData("roll degrees", odo.getRoll(AngleUnit.DEGREES));
            telemetry.addData("pitch degrees", odo.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Heading Velocity degrees", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
            telemetry.addData("REV Hub Frequency", frequency);
            telemetry.update();
        }
    }
}
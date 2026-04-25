package org.firstinspires.ftc.teamcode.current.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterLookupTable;

@TeleOp(name = "state TeleOp with table", group = "linear equations test")
public class stateTeleOpWithTable extends LinearOpMode {

    Servo hood, turret1, turret2;
    DcMotorEx f1, f2, gecko;
    DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pip;

    @Override
    public void runOpMode() {
        // Hardware Mapping
        hood = hardwareMap.get(Servo.class, "hood");
        turret1 = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        f1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        f2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        pip = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Directions
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

        // Pinpoint Config
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();

        double tx = -72; // Target X
        double ty = 72;  // Target Y
        double var = 0;
        double AIM_OFFSET_DEG_LOCAL = 1;
        double visionOffsetDeg = 0.0; // Mocked for this test file since there's no Limelight mapped
        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

        ShooterLookupTable shooterTable = new ShooterLookupTable();
        // ADD YOUR DATA HERE (Examples based on your current code estimates)
        // Format: .add(Distance, HoodPos, Velocity);
        shooterTable.add(48,  0.5,  1350); // Close zone start
        shooterTable.add(72,  .7,  1500); // Close zone end
        shooterTable.add(96, .8, 1610); // The "Gap" (Guessing here, you must test this!)
        shooterTable.add(120, .8,  1800); // Far zone start
        shooterTable.add(144, .9,  1890); // Far zone end

        waitForStart();

        while (opModeIsActive()) {
            pip.update(); // Update position first!

            double robotX = pip.getPosX(DistanceUnit.INCH);
            double robotY = pip.getPosY(DistanceUnit.INCH);

            double xl = tx - robotX;
            double yl = ty - robotY;
            double hypot = Math.sqrt((xl * xl) + (yl * yl));

            // =========================================================================
            //  TURRET APPLICATION (EXACTLY LIKE OLDER CODE)
            // =========================================================================
            double angleToGoal = Math.atan2(yl, xl);
            double robotHeading = pip.getHeading(AngleUnit.RADIANS);

            // Base Turret Angle Calculation
            double calculatedTurretRad = angleToGoal - robotHeading;

            // 1. Wrap angle (-180 to 180)
            while (calculatedTurretRad > Math.PI) calculatedTurretRad -= 2 * Math.PI;
            while (calculatedTurretRad < -Math.PI) calculatedTurretRad += 2 * Math.PI;

            // 2. Convert to Degrees and apply center offset (159 deg from comment)
            double finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

            // 4. Combine Physics + Vision
            finalServoDegrees += visionOffsetDeg + AIM_OFFSET_DEG_LOCAL;

            // 5. Clamp and Set
            finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);
            if (gamepad1.ps || gamepad2.ps) {
                turret1.setPosition(.5);
                turret2.setPosition(.5);
            } else {
                turret1.setPosition(finalServoDegrees / 303);
                turret2.setPosition(finalServoDegrees / 303);
            }

            // =========================================================================
            //  HOOD LINEAR REGRESSION
            // =========================================================================


            double[] settings = shooterTable.get(hypot);

            // Extract values
            double targetHood = settings[0];
            double targetVel = settings[1];

            // Safety Clamps (Good practice to keep these)
            targetHood = Range.clip(targetHood, 0.5, 1.0);
            targetVel = Range.clip(targetVel, 0, 2500);

            // Apply to hardware
            hood.setPosition(targetHood);

            if (gamepad1.x) {
                var = 1;
            }
            else if (gamepad1.y) {
                var = 0;
            }

            if (var == 1) {
                f1.setVelocity(targetVel);
                f2.setVelocity(targetVel);
            } else {
                f1.setVelocity(0);
                f2.setVelocity(0);
            }





            // =========================================================================
            //  INTAKE LOGIC
            // =========================================================================
            double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
            }
            else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;
            }

            intake.setPower(intakeCmd);

            if (intakeCmd != 0) {
                gecko.setPower(gamepad1.left_bumper || gamepad1.right_bumper ? -1.0 : 1.0);
            } else {
                gecko.setPower(0);
            }

            // =========================================================================
            //  DRIVE LOGIC
            // =========================================================================
            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawCmd = -gamepad1.right_stick_x;

            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            // =========================================================================
            //  ODOMETRY RESET (EXACTLY LIKE OLDER CODE)
            // =========================================================================
            if (gamepad1.dpad_up) {
                pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, 28.72, AngleUnit.DEGREES, 91.2));
            } else if (gamepad1.dpad_down) {
                pip.resetPosAndIMU();
            }

            // =========================================================================
            //  TELEMETRY
            // =========================================================================
            telemetry.addData("Distance (Hypot)", hypot);
            telemetry.addData("Target Hood", targetHood);
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Turret Angle (Deg)", finalServoDegrees);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.update();
        }
    }
}
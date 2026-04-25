package org.firstinspires.ftc.teamcode.current;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Config
@TeleOp(name = "shoot while move test", group = "TeleOp")
public class redTeleopnew extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // --- Hardware Init ---
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood = hardwareMap.get(Servo.class, "hood");
        Servo turretServo1 = hardwareMap.get(Servo.class, "turret1");
        Servo turretServo2 = hardwareMap.get(Servo.class, "turret2");

        // --- Configurations ---
        hood.setDirection(Servo.Direction.REVERSE);

        // IMPORTANT: If servos are on opposite sides, reverse one!
        turretServo2.setDirection(Servo.Direction.REVERSE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        // --- Pinpoint Odometry Init ---
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setBulkReadScope(GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        odo.setOffsets(-42, -90, DistanceUnit.MM);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            odo.update(); // IMPORTANT: Update Odometry every loop

            // --- STEP 1: GATHER DATA ---
            double robotX = odo.getPosX(DistanceUnit.INCH);
            double robotY = odo.getPosY(DistanceUnit.INCH);
            double robotHeading = odo.getHeading(AngleUnit.DEGREES);
            double robotVx = odo.getVelX(DistanceUnit.INCH); // Note: Pinpoint X velocity is usually forward/back relative to pod setup
            double robotVy = odo.getVelY(DistanceUnit.INCH);

            // --- STEP 2: CALCULATE PHYSICS ---
            ShooterSolution solution = calculateShooterSolution(robotX, robotY, robotVx, robotVy);

            // --- STEP 3: CALCULATE TURRET ANGLE ---
            // solution.turretAngle is already the Field-Centric target angle in degrees
            double turretTargetAngle = solution.turretAngle - robotHeading;

            // Normalize angle to -180 to 180 to find shortest path
            while (turretTargetAngle > 180) turretTargetAngle -= 360;
            while (turretTargetAngle <= -180) turretTargetAngle += 360;

            // --- STEP 4: CONVERT TO SERVO POSITION ---
            double positionOffset = turretTargetAngle / ShooterConstants.TURRET_RANGE_DEGREES;
            double servoPosition = ShooterConstants.TURRET_HOME_POSITION + positionOffset;

            // --- STEP 5: SAFETY CLAMP ---
            servoPosition = Math.max(ShooterConstants.TURRET_MIN_POS,
                    Math.min(ShooterConstants.TURRET_MAX_POS, servoPosition));

            // --- STEP 6: COMMAND SERVOS & MOTORS ---
            turretServo1.setPosition(servoPosition);
            turretServo2.setPosition(servoPosition);

            // Set Flywheel Velocity (Convert linear in/s to motor ticks/s if needed, assuming direct mapping for now)
            // You might need a conversion factor here like: velocity * TICKS_PER_INCH
            flywheel1.setVelocity(solution.flywheelVelocity);
            flywheel2.setVelocity(solution.flywheelVelocity);

            // Set Hood Position
            // Map degrees to servo position (Example: 0.0 is min angle, 1.0 is max angle)
            double hoodServoPos = (solution.hoodAngle - ShooterConstants.HOOD_MIN_ANGLE) / (ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE);
            hood.setPosition(hoodServoPos);

            // Telemetry
            telemetry.addData("Turret Target (Deg)", turretTargetAngle);
            telemetry.addData("Servo Pos", servoPosition);
            telemetry.addData("Flywheel Vel", solution.flywheelVelocity);
            telemetry.addData("Hood Angle", solution.hoodAngle);
            telemetry.update();
        }
    }

    public ShooterSolution calculateShooterSolution(double robotX, double robotY, double robotVx, double robotVy) {
        // --- 1. Calculate Vector to Goal ---
        double deltaX = ShooterConstants.GOAL_X - robotX;
        double deltaY = ShooterConstants.GOAL_Y - robotY;
        double distanceToGoal = Math.hypot(deltaX, deltaY);
        double angleToGoal = Math.atan2(deltaY, deltaX); // Field-centric angle to goal

        // --- 2. Decompose Robot Velocity ---
        double radialVelocity = (robotVx * Math.cos(angleToGoal)) + (robotVy * Math.sin(angleToGoal));
        double tangentialVelocity = (-robotVx * Math.sin(angleToGoal)) + (robotVy * Math.cos(angleToGoal));

        // --- 3. Stationary Physics (The "Ideal" Shot) ---
        ShooterState idealShot = ShooterMath.calculateStationaryShot(distanceToGoal);

        double horizontalBallSpeed = idealShot.velocity * Math.cos(idealShot.angle);
        double flightTime = distanceToGoal / horizontalBallSpeed;

        // --- 4. Velocity Compensation ---
        double requiredHorizontalSpeed = (distanceToGoal / flightTime) - radialVelocity;
        double newTotalHorizontalSpeed = Math.hypot(requiredHorizontalSpeed, tangentialVelocity);

        // Calculate Turret Offset (If drifting right, aim left)
        double turretOffsetAngle = Math.atan2(tangentialVelocity, requiredHorizontalSpeed);
        double finalTurretFieldAngle = Math.toDegrees(angleToGoal - turretOffsetAngle);

        // --- 5. Final Recalculation ---
        double virtualDistance = newTotalHorizontalSpeed * flightTime;
        ShooterState finalShot = ShooterMath.calculateStationaryShot(virtualDistance);

        return new ShooterSolution(
                Math.toDegrees(finalShot.angle), // Hood Angle (degrees)
                finalShot.velocity,              // Flywheel Velocity
                finalTurretFieldAngle            // Turret Angle (degrees)
        );
    }

    // --- INNER CLASSES FOR CALCULATIONS ---

    public static class ShooterConstants {
        // Field Constants (Measure these on your field)
        public static final double GOAL_X               = -70.5               ; // inches
        public static final double GOAL_Y               =  70.5               ; // inches
        // Tuning Parameters (Adjust these during testing)
        public static double       SCORE_HEIGHT         =  16.0               ; // Height of point above the launcher
        public static double       SCORE_ANGLE          =  Math.toRadians(-45); // Downward angle at entry
        public static double       POINT_RADIUS         =  4.0                ; // Distance from goal center to the point
        // Physics Constants
        public static final double GRAVITY              =  386.1              ; // in/s^2 (approx 9.8 m/s^2)

        // Turret & Hood Hardware Constants
        public static final double TURRET_RANGE_DEGREES = 303.5               ;
        public static final double TURRET_HOME_POSITION = 0.5                 ;
        public static final double TURRET_MIN_POS       = 0.02                ;
        public static final double TURRET_MAX_POS       = 0.98                ;
        public static final double HOOD_MIN_ANGLE       = 20.0                ; // Example values, change these!
        public static final double HOOD_MAX_ANGLE       = 60.0                ;
    }

    public static class ShooterState {
        public double angle;    // radians
        public double velocity; // in/s

        public ShooterState(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
        }
    }

    public static class ShooterSolution {
        public double hoodAngle;       // degrees
        public double flywheelVelocity; // in/s
        public double turretAngle;     // degrees

        public ShooterSolution(double hoodAngle, double flywheelVelocity, double turretAngle) {
            this.hoodAngle = hoodAngle;
            this.flywheelVelocity = flywheelVelocity;
            this.turretAngle = turretAngle;
        }
    }

    public static class ShooterMath {
        public static ShooterState calculateStationaryShot(double distanceToPoint) {
            // Apply Point Radius correction roughly (simple subtraction for tuning)
            double x = distanceToPoint - ShooterConstants.POINT_RADIUS;
            double y = ShooterConstants.SCORE_HEIGHT;
            double theta = ShooterConstants.SCORE_ANGLE;
            double g = ShooterConstants.GRAVITY;

            double term1 = (2 * y) / x;
            double term2 = Math.tan(theta);
            double launchAngle = Math.atan(term1 - term2);

            double numerator = g * Math.pow(x, 2);
            double denominator = 2 * Math.pow(Math.cos(launchAngle), 2) * ((x * Math.tan(launchAngle)) - y);
            // Protect against negative numbers in sqrt
            double launchVelocity = (denominator > 0) ? Math.sqrt(numerator / denominator) : 0;

            return new ShooterState(launchAngle, launchVelocity);
        }
    }
}
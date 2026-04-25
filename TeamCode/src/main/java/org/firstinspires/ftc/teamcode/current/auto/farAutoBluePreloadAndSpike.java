package org.firstinspires.ftc.teamcode.current.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumDrive2;

@Autonomous(name = "far Auto blue preload and spike", group = "competition")
public class farAutoBluePreloadAndSpike extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, gecko;
    public Servo hood, turret1, turret2;

    public class Robot {
        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                gecko.setPower(1);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class fire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                    intake.setPower(1);
                    gecko.setPower(-1);
                return false;
            }
        }
        public Action fire() { return new fire(); }

        private class stopFire implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(0);
                gecko.setPower(0);
                return false;
            }
        }
        public Action stopFire() { return new stopFire(); }

        private class flywheelUp implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                hood.setPosition(1);
                turret1.setPosition(.51);
                turret2.setPosition(.51);
                flywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel1.setVelocity(1900);
                flywheel2.setVelocity(1900);
                return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /* ---- initialize hardware ---- */
        intake     = hardwareMap.get(DcMotor.class, "intake");
        flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");
        gecko      = hardwareMap.get(DcMotorEx.class, "gecko");
        hood       = hardwareMap.get(Servo.class, "hood");
        turret1    = hardwareMap.get(Servo.class, "turret");
        turret2    = hardwareMap.get(Servo.class, "turret2");

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);

        /* ---- S Selection (Toggle) ---- */
        int S = 1; // 1 = Red, -1 = Blue
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.b) S = 1;  // B for RED
            if (gamepad1.x) S = -1; // X for BLUE

            telemetry.addLine("--- S SELECTION ---");
            telemetry.addData("SELECTED S", S == 1 ? "RED (B)" : "BLUE (X)");
            telemetry.update();
        }

        /* ---- Adaptive Braking Constraint ---- */
        VelConstraint adaptiveBrake = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45.0;
            double slowVel = 90.0;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            }
            return cruiseVel;
        };

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(62, -6.5, Math.toRadians(-180));
        MecanumDrive2 drive = new MecanumDrive2(hardwareMap, initialPose);

        // Pinpoint Initialization
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(84, -90, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                initialPose.position.x, initialPose.position.y,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                initialPose.heading.toDouble()
        ));

        Robot robot = new Robot();

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.flywheelUp())
                .setTangent(Math.toRadians(-115))
                .splineToLinearHeading(new Pose2d(48, -10 , Math.toRadians(-155)), Math.toRadians(-180), adaptiveBrake)
                .waitSeconds(3)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.1)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.2)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())

                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(32, -54, Math.toRadians(-90)), Math.toRadians(-90))

                .strafeToLinearHeading(new Vector2d(48, -10 ), Math.toRadians(-155))
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.1)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.3)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())

                /*// loading zone
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(60, 65, Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(48, 50 ), Math.toRadians(90))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(48, 10 ), Math.toRadians(155))
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.1)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.2)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())


                //first gate pickup
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(60, 63, Math.toRadians(100)), Math.toRadians(90))

                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(26, 63, Math.toRadians(140)), Math.toRadians(180))

                .setTangent(Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(48, 10 ), Math.toRadians(155))
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.1)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())
                .waitSeconds(.2)
                .stopAndAdd( robot.fire())
                .waitSeconds(.2)
                .stopAndAdd( robot.stopFire())*/

                .strafeToLinearHeading(new Vector2d(48, -16 ), Math.toRadians(-180))



               /* //second gate pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(46, 63, Math.toRadians(140)), Math.toRadians(90))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(26, 63, Math.toRadians(140)), Math.toRadians(180))

                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(60.5, 6.5, Math.toRadians(150)), Math.toRadians(315))
                .afterDisp(0, robot.fire())
                .waitSeconds(1)
                .stopAndAdd( robot.stopFire())


                //third gate pickup
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(46, 63, Math.toRadians(140)), Math.toRadians(90))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(26, 63, Math.toRadians(140)), Math.toRadians(180))

                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(60.5, 6.5, Math.toRadians(150)), Math.toRadians(315))
                .afterDisp(0, robot.fire())
                .waitSeconds(1)
                .stopAndAdd( robot.stopFire())*/
                .build();

        Action PRELOAD = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.flywheelUp())
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(315 * S), adaptiveBrake)
                .waitSeconds(4)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action PGP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180 * S))
                .splineToLinearHeading(new Pose2d(13, 55 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action FLUSH = drive.actionBuilder(new Pose2d(13, 55 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToLinearHeading(new Pose2d(13, 40 * S, Math.toRadians(90 * S)), Math.toRadians(270 * S), adaptiveBrake)
                .setTangent(Math.toRadians(270 * S))
                .splineToLinearHeading(new Pose2d(1, 40 * S, Math.toRadians(90 * S)), Math.toRadians(180 * S), adaptiveBrake)
                .setTangent(Math.toRadians(90 * S))
                .splineToLinearHeading(new Pose2d(1, 55 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action RETURN_AND_SHOOT = drive.actionBuilder(new Pose2d(1, 55 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action GPP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180 * S))
                .splineToSplineHeading(new Pose2d(37, 53 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action RETURN_AND_SHOOT_GPP = drive.actionBuilder(new Pose2d(37, 47 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action LAST_STEP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(65, 65 * S, Math.toRadians(90 * S)), Math.toRadians(45 * S), adaptiveBrake)
                .build();

        Action LAST_RETURN = drive.actionBuilder(new Pose2d(60, 65 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action PARK = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(5, 30 * S, Math.toRadians(180 * S)), Math.toRadians(115 * S), adaptiveBrake)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                TEST
               /* PRELOAD,
                PGP,
                FLUSH,
                RETURN_AND_SHOOT,
                GPP,
                RETURN_AND_SHOOT_GPP,
                LAST_STEP,
                LAST_RETURN,
                PARK*/
        ));


        // Save Pose for TeleOp
        if (drive.localizer.getPose() != null) {
            org.firstinspires.ftc.teamcode.Subsystems.PoseStorage.currentPose = drive.localizer.getPose();
        }
    }
}
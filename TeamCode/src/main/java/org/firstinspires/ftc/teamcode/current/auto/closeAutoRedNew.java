package org.firstinspires.ftc.teamcode.current.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive2;

@Autonomous(name = "Competition Auto - Turret Bot", group = "competition")
public class closeAutoRedNew extends LinearOpMode {

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

                hood.setPosition(.5);
                turret1.setPosition(.5);
                turret2.setPosition(.5);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel1.setVelocity((double) (3300 * 28) / 60);
                flywheel2.setVelocity((double) (3300 * 28) / 60);
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
        turret1     = hardwareMap.get(Servo.class, "turret");
        turret2     = hardwareMap.get(Servo.class, "turret2");

        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

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
            double slowVel = 12.0;
            double brakeZone = 10.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            }
            return cruiseVel;
        };

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(-46.5, 52.5 * S, Math.toRadians(127 * S));
        Pose2d afterPreload = new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S));

        MecanumDrive2 drive = new MecanumDrive2(hardwareMap, initialPose);
        Robot robot = new Robot();

        Action PRELOAD = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.flywheelUp())
                .setTangent(Math.toRadians(315 * S))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)), Math.toRadians(315 * S), adaptiveBrake)
                .waitSeconds(1)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action PPG = drive.actionBuilder(afterPreload)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90 * S))
                // Added adaptiveBrake here
                .setTangent(Math.toRadians(0 * S))
                .splineToSplineHeading(new Pose2d(-11, 20 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .splineToSplineHeading(new Pose2d(-11, 60 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action FLUSH = drive.actionBuilder(new Pose2d(-11, 60 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToLinearHeading(new Pose2d(1, 38 * S, Math.toRadians(180 * S)), Math.toRadians(90 * S))
                .setTangent(Math.toRadians(90 * S))
                .splineToSplineHeading(new Pose2d(1, 54 * S, Math.toRadians(180 * S)), Math.toRadians(90 * S))
                .build();

        Action RETURN_AND_SHOOT = drive.actionBuilder(new Pose2d(1, 54 * S, Math.toRadians(180 * S)))
                .setTangent(Math.toRadians(90 * S))
                .splineToSplineHeading(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)), Math.toRadians(225 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action PGP = drive.actionBuilder(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0 * S))
                .splineToSplineHeading(new Pose2d(13, 60 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action RETURN_AND_SHOOT_PGP = drive.actionBuilder(new Pose2d(13, 60 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)), Math.toRadians(225 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action GPP = drive.actionBuilder(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0 * S))
                .splineToLinearHeading(new Pose2d(20, 15.5 * S, Math.toRadians(90 * S)), Math.toRadians(0 * S), adaptiveBrake)
                .setTangent(Math.toRadians(0 * S))
                .splineToSplineHeading(new Pose2d(37, 60 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S), adaptiveBrake)
                .build();

        Action RETURN_AND_SHOOT_GPP = drive.actionBuilder(new Pose2d(37, 60 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)), Math.toRadians(225 * S), adaptiveBrake)
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action PARK = drive.actionBuilder(new Pose2d(-14.5, 15.5 * S, Math.toRadians(145 * S)))
                .setTangent(Math.toRadians(180 * S))
                .splineToSplineHeading(new Pose2d(0, 38 * S, Math.toRadians(180 * S)), Math.toRadians(180 * S))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                PRELOAD,
                PPG,
                FLUSH,
                RETURN_AND_SHOOT,
                PGP,
                RETURN_AND_SHOOT_PGP,
                GPP,
                RETURN_AND_SHOOT_GPP,
                PARK
        ));
    }
}
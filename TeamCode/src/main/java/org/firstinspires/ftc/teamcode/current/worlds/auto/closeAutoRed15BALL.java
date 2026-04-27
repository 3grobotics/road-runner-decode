package org.firstinspires.ftc.teamcode.current.worlds.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.MecanumDrive2;

@Autonomous(name = "close Auto Red 15 Ball", group = "competition")
public class closeAutoRed15BALL extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */
    private DcMotor intake;
    public DcMotorEx flywheel1, flywheel2, gecko;
    public Servo hood, turret1, turret2;
    public GoBildaPinpointDriver pip;
    double vTarget;
    double hpos;
    double tx = -72; // Target X
    double ty = 72;  // Target Y
    double t = 1;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;
    double offset = 10;


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
                if (opModeIsActive()){
                pip.update();
                //^ this cant be in brackets ^


                vy = pip.getVelY(DistanceUnit.INCH);
                vx = pip.getVelX(DistanceUnit.INCH);


                tx = -72 - (vx * t); // 72 or tx maybe
                ty = 72 - (vy * t); // 72 or tx maybe

                robotX = pip.getPosX(DistanceUnit.INCH);
                robotY = pip.getPosY(DistanceUnit.INCH);

                xl = tx - robotX;
                yl = ty - robotY;
                hypot = Math.sqrt((xl * xl) + (yl * yl)) + offset;


                if (hypot < 80) {
                    t = 0.003 * hypot + 0.347;
                } else if (hypot < 100 && hypot > 80) {
                    t = 0.004 * hypot + 0.263;
                } else if (hypot > 100) {
                    t = 0.003 * hypot + 0.367;
                }


                    // Zone: Close
                    double d1h = 57.5, d2h = 97.3;
                    double v1h = 0.5, v2h = 0.7;
                    double slopeh = (v2h - v1h) / (d2h - d1h);
                    hpos = v1h + (slopeh * (hypot - d1h));


                hood.setPosition(Range.clip(hpos, 0.5, 1.0));

                turret1.setPosition(.485);
                turret2.setPosition(.485);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));

                // Zone: Close
                double d1 = 57.5, d2 = 97.3;
                double v1 = 1310, v2 = 1660;
                double slope = (v2 - v1) / (d2 - d1);
                vTarget = 1150 + (slope * (hypot - d1));

                vTarget = Range.clip(vTarget, 0, 2500); // Adjust max based on motor


                flywheel1.setVelocity(vTarget);
                flywheel2.setVelocity(vTarget);

                /*flywheel1.setVelocity((double) (2800 * 28) / 60);
                flywheel2.setVelocity((double) (2800 * 28) / 60);*/
                return true;}
                else return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }

        private class flywheelUpPre implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {

                hood.setPosition(.5);
                turret1.setPosition(.485);
                turret2.setPosition(.485);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
                flywheel1.setVelocity((double) (2000 * 28) / 60);
                flywheel2.setVelocity((double) (2000 * 28) / 60);
                return false;
            }
        }
        public Action flywheelUpPre() { return new flywheelUpPre(); }


        private class lowerVelocity implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                offset = 5;
                return false;
            }
        }
        public Action lowerVelocity() { return new lowerVelocity(); }
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
        hood.setDirection(Servo.Direction.REVERSE);

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
            double cruiseVel = 90;
            double slowVel = 45;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            } else return cruiseVel;
        };

        VelConstraint adaptiveBrakeSlow = (robotPose, path, pathPos) -> {
            double distLeft = path.length() - pathPos;
            double cruiseVel = 45;
            double slowVel = 20;
            double brakeZone = 20.0;

            if (distLeft < brakeZone) {
                return slowVel + (cruiseVel - slowVel) * (distLeft / brakeZone);
            } else return cruiseVel;
        };

        /* ---- Poses & Actions ---- */
        Pose2d initialPose = new Pose2d(-46.5, 52.5, Math.toRadians(127));
        MecanumDrive2 drive = new MecanumDrive2(hardwareMap, initialPose);

        // Pinpoint Initialization
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pip.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                DistanceUnit.INCH,
                initialPose.position.x, initialPose.position.y,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                initialPose.heading.toDouble()
        ));

        Robot robot = new Robot();

        // Pathing Definitions
        Action TEST = drive.actionBuilder(initialPose)
                //.afterDisp(0, robot.flywheelUp())
                .waitSeconds(.5)
                .afterDisp(20, robot.fire())
                //.afterDisp(40, robot.stopFire())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(315))


                //cc middle spike
                .afterDisp(0, robot.intake())
                .afterDisp(0, robot.lowerVelocity())
                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(12, 50, Math.toRadians(90)), Math.toRadians(90))



                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(180))
                //.waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())

                //cc flush cycle
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(5, 55, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(25, 55, Math.toRadians(135)), Math.toRadians(45))
                .waitSeconds(1)

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(180))
                //.waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                //cc goal spike
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-12, 30, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 53, Math.toRadians(90)), Math.toRadians(90))

                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(245))
                //.waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())


                //cc flush cycle
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(5, 55, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(25, 55, Math.toRadians(135)), Math.toRadians(45))
                .waitSeconds(1)

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(180))
                //.waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(.5)
                .stopAndAdd(robot.stopFire())


                // park
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(1, 40, Math.toRadians(90)), Math.toRadians(90))






               /* // closest to loading zone spike
                //.turnTo(Math.toRadians(90))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(37, 27, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(37, 57, Math.toRadians(90)), Math.toRadians(90))

                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(245))
                //.waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())


                // park
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(1, 40, Math.toRadians(90)), Math.toRadians(90))
*/
                /*.afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, 38, Math.toRadians(100)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, 55, Math.toRadians(100)), Math.toRadians(90))

                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(20, 60, Math.toRadians(140)), Math.toRadians(45))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(140)), Math.toRadians(0))
                .waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())

                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, 38, Math.toRadians(100)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(6, 55, Math.toRadians(100)), Math.toRadians(90))

                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(20, 60, Math.toRadians(140)), Math.toRadians(45))*/




                /*.setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(245))
                .waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())

                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, 38, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, 55, Math.toRadians(90)), Math.toRadians(90))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(40, 60, Math.toRadians(140)), Math.toRadians(90))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(16, 60, Math.toRadians(140)), Math.toRadians(180))

                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(-14.5, 15.5, Math.toRadians(145)), Math.toRadians(245))
                .waitSeconds(2)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())

                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, 38, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(2, 55, Math.toRadians(90)), Math.toRadians(90))*/
                .build();

        Action PRELOAD = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.flywheelUp())
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(315 * S))
                .waitSeconds(4)
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action PGP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180 * S))
                .splineToLinearHeading(new Pose2d(13, 55 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S))
                .build();

        Action FLUSH = drive.actionBuilder(new Pose2d(13, 55 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToLinearHeading(new Pose2d(13, 40 * S, Math.toRadians(90 * S)), Math.toRadians(270 * S))
                .setTangent(Math.toRadians(270 * S))
                .splineToLinearHeading(new Pose2d(1, 40 * S, Math.toRadians(90 * S)), Math.toRadians(180 * S))
                .setTangent(Math.toRadians(90 * S))
                .splineToLinearHeading(new Pose2d(1, 55 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S))
                .build();

        Action RETURN_AND_SHOOT = drive.actionBuilder(new Pose2d(1, 55 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S))
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action GPP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(180 * S))
                .splineToSplineHeading(new Pose2d(37, 53 * S, Math.toRadians(90 * S)), Math.toRadians(90 * S))
                .build();

        Action RETURN_AND_SHOOT_GPP = drive.actionBuilder(new Pose2d(37, 47 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S))
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action LAST_STEP = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(65, 65 * S, Math.toRadians(90 * S)), Math.toRadians(45 * S))
                .build();

        Action LAST_RETURN = drive.actionBuilder(new Pose2d(60, 65 * S, Math.toRadians(90 * S)))
                .setTangent(Math.toRadians(270 * S))
                .splineToSplineHeading(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)), Math.toRadians(-45 * S))
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();

        Action PARK = drive.actionBuilder(new Pose2d(60.5, 5.5 * S, Math.toRadians(155 * S)))
                .setTangent(Math.toRadians(115 * S))
                .splineToLinearHeading(new Pose2d(5, 30 * S, Math.toRadians(180 * S)), Math.toRadians(115 * S))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                TEST,
                robot.flywheelUp()
                )
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
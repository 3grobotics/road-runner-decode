package org.firstinspires.ftc.teamcode.current.auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled

@Autonomous(name = "close auto blue new turret bot ", group = "competition")
public class closeAutoBlueNew extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */

    private DcMotor intake;
    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;
    public DcMotorEx gecko;
    public Servo hood;
    public Servo turret;


    public class Robot {


        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
                gecko.setPower(1);
                return false;
            }
        }
        public Action intake() { return new intake(); }

        private class outtake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(-1);
                return false;
            }
        }
        public Action outtake() { return new outtake(); }

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
                hood.setPosition(.7);
                turret.setPosition(.5);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
                flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
                flywheel1.setVelocity((5000 * 28) / 60);
                flywheel2.setVelocity((5000 * 28) / 60);
                return false;
            }
        }
        public Action flywheelUp() { return new flywheelUp(); }

        private class flywheelUpFAR implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                hood.setPosition(.94);
                turret.setPosition(.5);
                flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 20));
                flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
                flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
                flywheel1.setVelocity((6000 * 28) / 60);
                flywheel2.setVelocity((6000 * 28) / 60);
                return false;
            }
        }
        public Action flywheelUpFAR() { return new flywheelUpFAR(); }





//(TARGET_RPM * 28) / 60

    }


    /* ──────────────── main opmode ──────────────── */

    @Override
    public void runOpMode() throws InterruptedException {


        /* ---- initialize hardware ---- */
        intake     = hardwareMap.get(DcMotor.class, "intake");
        flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");
        gecko = hardwareMap.get(DcMotorEx.class, "gecko");
        hood = hardwareMap.get(Servo.class, "hood");
        turret = hardwareMap.get(Servo.class, "turret");


        /* ---- traj / action setup ---- */
        Pose2d initialPose = new Pose2d(-46.5, -52.5, Math.toRadians(-127));
        Pose2d afterPreload = new Pose2d(-14.5, -15.5, Math.toRadians(-135));


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot();



        Action PRELOAD = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.flywheelUp())
                .afterDisp(20, robot.fire())
                .setTangent(Math.toRadians(-315))
                .splineToLinearHeading(new Pose2d(-14.5, -15.5, Math.toRadians(-135)), Math.toRadians(-315))
                //.stopAndAdd(robot.fire())
                //.waitSeconds(1)
                .stopAndAdd(robot.stopFire())

                .build();
        /// pickups
        Action PPG = drive.actionBuilder(afterPreload)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-11, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Action PPG_BACKWARDS = drive.actionBuilder(afterPreload)
                .setTangent(Math.toRadians(-315 - 180))
                .splineToLinearHeading(new Pose2d(-21, -53, Math.toRadians(-340)), Math.toRadians(-45))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(-11, -53, Math.toRadians(-270)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(-11, -47, Math.toRadians(-270)), Math.toRadians(-90))
                .build();

        Action PGP = drive.actionBuilder(new Pose2d(-14.5, -15.5, Math.toRadians(-135)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToSplineHeading(new Pose2d(13, 55, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Action GPP = drive.actionBuilder(new Pose2d(-14.5, -15.5, Math.toRadians(-135)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-0))
                .splineToSplineHeading(new Pose2d(-11 + 48, -53, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Action GPP_BACKWARDS = drive.actionBuilder(afterPreload)
                .setTangent(Math.toRadians(-315))
                .splineToSplineHeading(new Pose2d(24, -24, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-21 + 48, -53, Math.toRadians(-340)), Math.toRadians(-45))
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(-11 + 48, -53, Math.toRadians(-270)), Math.toRadians(-270))
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(-11 + 48, -47, Math.toRadians(-270)), Math.toRadians(-270))
                .build();

        /// utilities
        Action RETURN_AND_SHOOT_GPP_BACKWARDS = drive.actionBuilder(afterPreload)
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-14, -15.5, Math.toRadians(-132)), Math.toRadians(-225))
                .build();

        Action FLUSH = drive.actionBuilder(new Pose2d(-11, -55, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -38, Math.toRadians(-180)), Math.toRadians(-90))

                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -54, Math.toRadians(-180)), Math.toRadians(-90))
                .build();

        Action RETURN_AND_SHOOT = drive.actionBuilder(new Pose2d(-1.5, -54, Math.toRadians(-180)))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-14.5, -15.5, Math.toRadians(-135)), Math.toRadians(-225))
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action RETURN_AND_SHOOT_PGP = drive.actionBuilder(new Pose2d(12, -55, Math.toRadians(-120)))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-14.5, -15.5, Math.toRadians(-135)), Math.toRadians(-225))
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action RETURN_AND_SHOOT_GPP = drive.actionBuilder(new Pose2d(-11 + 48, -47, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(-14.5, -15.5, Math.toRadians(-135)), Math.toRadians(-225))
                .stopAndAdd(robot.fire())
                .waitSeconds(1)
                .build();

        Action PARK = drive.actionBuilder(new Pose2d(-14.5, -15.5, Math.toRadians(-135)))
                .setTangent(Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(0, -38, Math.toRadians(-180)), Math.toRadians(-180))
                .build();

        Action LAST_STEP = drive.actionBuilder(new Pose2d(-14.5, -15.5, Math.toRadians(-135)))
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(67, -60, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Action LAST_RETURN = drive.actionBuilder(new Pose2d(67, -60, Math.toRadians(-90)))
                .afterDisp(0, robot.flywheelUpFAR())
                .setTangent(Math.toRadians(-270))
                .splineToSplineHeading(new Pose2d(60.5, -5.5, Math.toRadians(-155)), Math.toRadians(45))
                .stopAndAdd(robot.fire())
                .waitSeconds(2)
                .stopAndAdd(robot.stopFire())
                .build();





        waitForStart();

        if (isStopRequested()) return;
        if (isStopRequested()){
        }


        /* ---- main autonomous ---- */
        Actions.runBlocking(new SequentialAction(
                PRELOAD,
                PPG,
                FLUSH,
                RETURN_AND_SHOOT,
                PGP,
                RETURN_AND_SHOOT_PGP,
                GPP,
                RETURN_AND_SHOOT_GPP,
                LAST_STEP,
                LAST_RETURN,
                PARK
                /*PGP,
                FLUSH,
                RETURN_AND_SHOOT,
                GPP,
                RETURN_AND_SHwOOT,
                PPG_BACKWARDS,
                RETURN_AND_SHOOT*/

        ));

        /* ---- cleanup ---- */

    }
}
package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.discoverfest.limelight2;
@Disabled
@Autonomous(name = "GOOD_GRID_SAMPLE_AUTO", group = "competition")
public class GOOD_GRID_SAMPLE_AUTO extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */

    private DcMotor intake;
    private Servo flipper;
    private Servo tiltLeft;
    private Servo tiltMid ;
    private Servo tiltRight;
    private CRServo pullRight;
    private CRServo pullMid;
    private CRServo pullLeft;
    boolean lswitchPressed = false;
    boolean mswitchPressed = false;
    boolean rswitchPressed = false;
    DigitalChannel leftSwitch;
    DigitalChannel midSwitch;
    DigitalChannel rightSwitch;





    public class Robot {


        private class posservo implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                tiltLeft.setPosition(.290);
                tiltMid.setPosition(.320);
                tiltRight.setPosition(.310);
                return false;
            }
        }
        public Action posservo() { return new posservo(); }

        private class posservo2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                tiltLeft.setPosition(.390);
                tiltMid.setPosition(.420);
                tiltRight.setPosition(.410);
                return false;
            }
        }
        public Action posservo2() { return new  posservo2(); }

        private class posservo3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                tiltLeft.setPosition(0.03);
                tiltMid.setPosition(0.03);
                tiltRight.setPosition(0.03);
                if (leftSwitch.getState()) {
                    lswitchPressed = true;
                    pullLeft.setPower(.2);
                } else if (!lswitchPressed) {
                    pullLeft.setPower(1.0);

                }
                if (rightSwitch.getState()) {
                    rswitchPressed = true;
                    pullRight.setPower(.2);
                } else if (!rswitchPressed) {
                    pullRight.setPower(1.0);

                }
                if (midSwitch.getState()) {
                    mswitchPressed = true;
                    pullMid.setPower(.2);
                } else if (!mswitchPressed) {
                    pullMid.setPower(1.0);

                }

                return false;
            }
        }
        public Action posservo3() { return new  posservo3(); }

        private class thing1 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.38);
                return false;
            }
        }
        public Action thing1() { return new  thing1(); }

        private class thing2 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.48);
                return false;
            }
        }
        public Action thing2() { return new  thing2(); }

        private class thing3 implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.75);
                return false;
            }
        }
        public Action thing3() { return new  thing3(); }


        private class intake implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                intake.setPower(1);
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


    }


    /* ──────────────── main opmode ──────────────── */

    @Override
    public void runOpMode() throws InterruptedException {

        /* ---- initialize hardware ---- */
        intake = hardwareMap.get(DcMotor.class, "intake");

        /* ---- traj / action setup ---- */
        Pose2d initialPose = new Pose2d(-50, 50, Math.toRadians(315));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot();

//.afterDisp(0, robot.thing3())
        Action INTAKE = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.intake())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-14, 14, Math.toRadians(315)), Math.toRadians(315))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-25, 36, Math.toRadians(45)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-11, 45), Math.toRadians(45))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-14, 14, Math.toRadians(315)), Math.toRadians(315))
                .afterDisp(0, robot.outtake())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(0, 38, Math.toRadians(45)), Math.toRadians(90))
                .afterDisp(0, robot.intake())
                .strafeToLinearHeading(new Vector2d(12, 45), Math.toRadians(45))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-14, 14, Math.toRadians(315)), Math.toRadians(135))
                .afterDisp(0, robot.outtake())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(24, 34, Math.toRadians(45)), Math.toRadians(45))
                .afterDisp(0, robot.intake())
                .strafeToLinearHeading(new Vector2d(37, 47), Math.toRadians(45))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(-14, 14, Math.toRadians(315)), Math.toRadians(135))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(38, 33, Math.toRadians(90)), Math.toRadians(90))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        if (isStopRequested()){
        }


        /* ---- main autonomous ---- */
        Actions.runBlocking(new SequentialAction(
                INTAKE
        ));

        /* ---- cleanup ---- */
    }
}
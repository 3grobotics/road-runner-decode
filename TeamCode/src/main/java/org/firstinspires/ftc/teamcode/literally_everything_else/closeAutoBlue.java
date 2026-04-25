package org.firstinspires.ftc.teamcode.literally_everything_else;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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
@Disabled
@Autonomous(name = "closeAutoBlue", group = "competition")
public class closeAutoBlue extends LinearOpMode {

    /* ──────────────── hardware ──────────────── */

    private DcMotor intake;
    public Servo flipper;
    public Servo tiltLeft;
    public Servo tiltMid ;
    public Servo tiltRight;
    public CRServo pullRight;
    public CRServo pullMid;
    public CRServo pullLeft;
    public boolean lswitchPressed = false;
    public boolean mswitchPressed = false;
    public boolean rswitchPressed = false;
    public DigitalChannel leftSwitch;
    public DigitalChannel midSwitch;
    public DigitalChannel rightSwitch;





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
                flipper.setPosition(.38);
                tiltLeft.setPosition(0.380);
                tiltMid.setPosition(0.350);
                tiltRight.setPosition(0.400);
                return false;
            }
        }
        public Action posservo2() { return new  posservo2(); }

        private class posservo3 implements Action {
            private boolean leftDone = false;
            private boolean midDone = false;
            private boolean rightDone = false;
            @Override public boolean run(@NonNull TelemetryPacket p) {
                flipper.setPosition(.75);
                // In your Robot class

                        // Set tilting servos to the correct position for winching.
                        tiltLeft.setPosition(0.03);
                        tiltMid.setPosition(0.03);
                        tiltRight.setPosition(0.03);

                        // --- Left Catapult ---
                        // getState() is false when the switch is pressed.
                        if (leftSwitch.getState() == true) {
                            leftDone = true;
                            pullLeft.setPower(.2); // Stop the motor
                        } else {
                            pullLeft.setPower(1.0); // Run motor backwards to winch
                        }

                        // --- Middle Catapult ---
                        if (midSwitch.getState() == true) {
                            midDone = true;
                            pullMid.setPower(.2); // Stop the motor
                        } else {
                            pullMid.setPower(1.0); // Run motor backwards to winch
                        }

                        // --- Right Catapult ---
                        if (rightSwitch.getState() == true) {
                            rightDone = true;
                            pullRight.setPower(.2); // Stop the motor
                        } else {
                            pullRight.setPower(1.0); // Run motor backwards to winch
                        }

                        // The action is complete only when all three switches have been pressed.
                        // Return 'true' to continue running, 'false' when done.
                        boolean isRunning = !(leftDone && midDone && rightDone);
                        return isRunning;
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

        private class fireM implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullMid.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireM() { return new fireM(); }

        private class fireR implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullRight.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireR() { return new fireR(); }

        private class fireL implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullLeft.setPower(1);
                sleep(500);
                return false;
            }
        }
        public Action fireL() { return new fireL(); }

        private class stopM implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullMid.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopM() { return new stopM(); }

        private class stopR implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullRight.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopR() { return new stopR(); }

        private class stopL implements Action {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                pullLeft.setPower(0);
                sleep(500);
                return false;
            }
        }
        public Action stopL() { return new stopL(); }


    }


    /* ──────────────── main opmode ──────────────── */

    @Override
    public void runOpMode() throws InterruptedException {


        /* ---- initialize hardware ---- */
        intake = hardwareMap.get(DcMotor.class, "intake");
        tiltRight = hardwareMap.get(Servo.class, "tiltRight");
        tiltMid   = hardwareMap.get(Servo.class, "tiltMid");
        tiltLeft  = hardwareMap.get(Servo.class, "tiltLeft");
        flipper  = hardwareMap.get(Servo.class, "flipper");

        // ---- Three pull servos (catapults) ----
        pullRight = hardwareMap.get(CRServo.class,"pullRight");
        pullMid   = hardwareMap.get(CRServo.class,"pullMid");
        pullLeft  = hardwareMap.get(CRServo.class,"pullLeft");
        leftSwitch  = hardwareMap.get(DigitalChannel.class, "leftSwitch");
        midSwitch   = hardwareMap.get(DigitalChannel.class, "midSwitch");
        rightSwitch = hardwareMap.get(DigitalChannel.class, "rightSwitch");

        /* ---- traj / action setup ---- */
        Pose2d initialPose = new Pose2d(-50, -50, Math.toRadians(45));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot();

//.afterDisp(0, robot.thing3())
        Action INTAKE = drive.actionBuilder(initialPose)
                .afterDisp(0, robot.posservo2())
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(45)), Math.toRadians(45))
                .afterDisp(0, robot.posservo3())
                .setTangent(Math.toRadians(235))
                .splineToLinearHeading(new Pose2d(-25, -36, Math.toRadians(315)), Math.toRadians(315))
                .afterDisp(0, robot.intake())
                .strafeToLinearHeading(new Vector2d(-6, -50), Math.toRadians(315))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(45)), Math.toRadians(135))
                .afterDisp(0, robot.posservo2())
                .waitSeconds(.5)
                .stopAndAdd(robot.fireL())
                .stopAndAdd(robot.fireM())
                .stopAndAdd(robot.fireR())
                .waitSeconds(.1)
                .stopAndAdd(robot.stopL())
                .stopAndAdd(robot.stopM())
                .stopAndAdd(robot.stopR())
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(0, -38, Math.toRadians(315)), Math.toRadians(315))
                .afterDisp(0, robot.stopL())
                .afterDisp(0, robot.stopM())
                .afterDisp(0, robot.stopR())
                /* .strafeToLinearHeading(new Vector2d(12, -45), Math.toRadians(315))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(45)), Math.toRadians(135))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(24, -34, Math.toRadians(315)), Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(37, -47), Math.toRadians(315))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-14, -14, Math.toRadians(45)), Math.toRadians(135))
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(new Pose2d(38, -33, Math.toRadians(90)), Math.toRadians(0))
                .turn(20.43)*/
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
package org.firstinspires.ftc.teamcode.current.worlds.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.teamcode.Subsystems.PoseStorage;

@Config
@TeleOp(name = "worlds teleop blue", group = "worlds")
public class worldsTeleopBlue extends LinearOpMode {

    Servo hood, turret1, turret2, kr, kl;
    DcMotorEx f1, f2, gecko;
    DcMotor intake, frontLeft, frontRight, backLeft, backRight;
    GoBildaPinpointDriver pip;

    boolean bPressed = false;
    int kickState   = 0;
    boolean dPressed = false;

    public static double farSlope =  1950 ;
    double tx = -72; // Target X
    double ty = -72;  // Target Y
    double t = 1;

    double samOffset = 5;
    double var = 0;
    double axial;
    double lateral;
    double yawCmd;

    double robotX;
    double robotY;
    double xl;
    double yl;
    double hypot;

    double vx;
    double vy;

    double vTarget;
    double hpos;

    double angleToGoal;
    double robotHeading;
    double calculatedTurretRad;
    double finalServoDegrees;
    double samOffsetv = 0;
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
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class,  "backRight ");
        backLeft = hardwareMap.get(DcMotor.class,   "backLeft  ");
        frontLeft = hardwareMap.get(DcMotor.class,  "frontLeft ");

        kr = hardwareMap.get(Servo.class,"kickstandRight");
        kl = hardwareMap.get(Servo.class,"kickstandLeft");

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        boolean prevUp = up;
        boolean prevDown = down;


        // Directions
        frontLeft.setDirection(DcMotorSimple.Direction .FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction  .FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction .REVERSE);

        f1.setDirection(DcMotorSimple.Direction.REVERSE);
        f2.setDirection(DcMotorSimple.Direction.REVERSE);
        hood.setDirection(Servo.Direction.REVERSE);
        turret1.setDirection(Servo.Direction.REVERSE);

        // Pinpoint Config
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();



        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean prevleft = left;
        boolean prevright = right;

        f1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        f2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 0, 200));
        PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Pose2d savedPose = PoseStorage.currentPose;
        if (savedPose.position.x == 0 && savedPose.position.y == 0 && savedPose.heading.toDouble() == 0) {
            // We didn't run Auto (or it crashed), so we assume it is the start of the match
            // We might want to change this to the known start pose if testing teleop alone
            // odo.resetPosAndIMU();
        } else {
            // We came from Auto so load the data.
            pip.resetPosAndIMU();

        }
        // ^ idk if this does anything honestly ^



        waitForStart();
        pip.setPosition(new Pose2D(
                DistanceUnit.INCH,
                savedPose.position.x,
                savedPose.position.y,
                AngleUnit.RADIANS,
                savedPose.heading.toDouble()
        ));
        telemetry.addLine(">>> AUTO POSITION LOADED <<<");
        while (opModeIsActive()) {
            pip.update();
            //^ this cant be in brackets ^

            /* shooting while moving stuff */ {
                vy = pip.getVelY(DistanceUnit.INCH);
                vx = pip.getVelX(DistanceUnit.INCH);


                tx = -72 - (vx * t); // 72 or tx maybe
                ty = -72 - (vy * t); // 72 or tx maybe

                robotX = pip.getPosX(DistanceUnit.INCH);
                robotY = pip.getPosY(DistanceUnit.INCH);

                xl = tx - robotX;
                yl = ty - robotY;
                hypot = Math.sqrt((xl * xl) + (yl * yl));


                if (hypot < 80) {
                    t = 0.003 * hypot + 0.347;
                } else if (hypot < 100 && hypot > 80) {
                    t = 0.004 * hypot + 0.263;
                } else if (hypot > 100) {
                    t = 0.003 * hypot + 0.367;
                }
            }

            /* turret stuff */ {
                angleToGoal = Math.atan2(yl, xl);
                robotHeading = pip.getHeading(AngleUnit.RADIANS);

                calculatedTurretRad = angleToGoal - robotHeading;

                while (calculatedTurretRad > Math.PI) {
                    calculatedTurretRad -= 2 * Math.PI;
                }
                while (calculatedTurretRad < -Math.PI) {
                    calculatedTurretRad += 2 * Math.PI;
                }

                finalServoDegrees = Math.toDegrees(calculatedTurretRad) + 151.5;

                left = gamepad2.dpad_left;
                right = gamepad2.dpad_right;


                if (left && !prevleft && !right) {
                    samOffset = Range.clip(samOffset + 2.5, -40, 40);
                }
                if (right && !prevright && !left) {
                    samOffset = Range.clip(samOffset - 2.5, -40, 40);
                }

                // leftdate prev AFTER using them
                prevleft = left;
                prevright = right;

                finalServoDegrees += samOffset;

                // 5. Clamp and Set
                finalServoDegrees = Range.clip(finalServoDegrees, 0, 303);
                if (gamepad1.ps || gamepad2.ps) {
                    turret1.setPosition(.5);
                    turret2.setPosition(.5);
                } else {
                    turret1.setPosition(finalServoDegrees / 303);
                    turret2.setPosition(finalServoDegrees / 303);
                }
            }

            /* hood stuff */ {
                if (hypot < 130) {
                    // Zone: Close
                    double d1 = 57.5, d2 = 97.3;
                    double v1 = 0.5, v2 = 0.7;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));
                } else {
                    // Zone: Far
                    double d1 = 136.5, d2 = 158.1;
                    double v1 = 0.7, v2 = 1.0;
                    double slope = (v2 - v1) / (d2 - d1);
                    hpos = v1 + (slope * (hypot - d1));
                }
                hood.setPosition(Range.clip(hpos, 0.5, 1.0));
            }

            /* flywheel stuff */ {
                if (hypot < 130) {
                    // Zone: Close
                    double d1 = 57.5, d2 = 97.3;
                    double v1 = 1310, v2 = 1660;
                    double slope = (v2 - v1) / (d2 - d1);
                    vTarget = 1150 + (slope * (hypot - d1));
                } else {
                    // Zone: Far
                    double d1 = 136.5, d2 = 158.1;
                    double v1 = 1900, v2 = 1940;
                    double slope = (v2 - v1) / (d2 - d1);
                    vTarget = farSlope + (slope * (hypot - d1));
                }
                vTarget = Range.clip(vTarget, 0, 2500); // Adjust max based on motor

                up = gamepad2.dpad_up;
                down = gamepad2.dpad_down;


                if (up && !prevUp && !down) {
                    samOffsetv = Range.clip(samOffsetv + 50, -2000, 2000);
                }
                if (down && !prevDown && !up) {
                    samOffsetv = Range.clip(samOffsetv - 50, -2000, 2000);
                }

                prevUp = up;
                prevDown = down;


                if (gamepad1.x) {
                    var = 1;
                } else if (gamepad1.y) {
                    var = 0;
                }

                if (var == 1) {
                    f1.setVelocity(vTarget + samOffsetv);
                    f2.setVelocity(vTarget + samOffsetv);
                } else {
                    f1.setVelocity(0);
                    f2.setVelocity(0);
                }
            }

            /* intake stuff */ {

                double intakeCmd = (gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad1.left_trigger + gamepad2.left_trigger);
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    intakeCmd = -1.0;
                } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    intakeCmd = 1.0;
                }

                intake.setPower(intakeCmd);

                if (intakeCmd != 0) {
                    gecko.setPower(gamepad1.left_bumper || gamepad1.right_bumper ? -1.0 : 1.0);
                } else {
                    gecko.setPower(0);
                }
            }

            /* kickstand stuff */ {
                if (gamepad1.b && !bPressed) {
                    kickState = (kickState + 1) % 2;
                    bPressed = true;
                } else if (!gamepad1.b) {
                    bPressed = false;
                }

                if (gamepad2.left_stick_button && !dPressed) {
                    kickState = (kickState + 1) % 2;
                    dPressed = true;
                } else if (!gamepad2.left_stick_button) {
                    dPressed = false;
                }
                switch (kickState) {
                    case 0:
                        kr.setPosition(.6);
                        kl.setPosition(.4);
                        break;
                    case 1:
                        kr.setPosition(.5);
                        kl.setPosition(.5);
                        break;
                }
            }

            /* driving stuff */ {
                axial = gamepad1.left_stick_y;
                lateral = -gamepad1.left_stick_x;
                yawCmd = -gamepad1.right_stick_x;

                double fl = axial + lateral + yawCmd;
                double fr = axial - lateral - yawCmd;
                double bl = axial - lateral + yawCmd;
                double br = axial + lateral - yawCmd;

                double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
                frontLeft.setPower(fl / max);
                frontRight.setPower(fr / max);
                backLeft.setPower(bl / max);
                backRight.setPower(br / max);
            }

            /* odometry reset stuff */ {
                if (gamepad1.dpad_up) {
                    pip.setPosition(new Pose2D(DistanceUnit.INCH, -68.02, -28.72, AngleUnit.DEGREES, -91.2));
                    samOffset = 0;
                } else if (gamepad1.dpad_down) {
                    pip.resetPosAndIMU();
                    samOffset = 0;
                } else if (gamepad1.dpad_left){
                    pip.setPosition(new Pose2D(DistanceUnit.INCH, 62, 65, AngleUnit.DEGREES, -0));
                    samOffset = 0;
                }
            }

            /* telemetry */
            telemetry.addData("vel offset", samOffsetv);
            telemetry.addData("Distance (Hypot)", hypot);
            telemetry.addData("Target Velocity", vTarget);
            telemetry.addData("Hood Position", hpos);
            telemetry.addData("Turret Angle (Deg)", finalServoDegrees);
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.update();
        }
    }
}
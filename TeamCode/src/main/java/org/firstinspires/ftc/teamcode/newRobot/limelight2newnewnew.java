
package org.firstinspires.ftc.teamcode.newRobot;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name = "novus umbra old no turret", group = "TeleOp")
public class limelight2newnewnew extends LinearOpMode {

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {

        // ---- Drive ----
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor intake     = hardwareMap.get(DcMotor.class, "intake");

        DcMotorEx flywheel1  = hardwareMap.get(DcMotorEx.class, "flywheel1");
        DcMotorEx flywheel2  = hardwareMap.get(DcMotorEx.class, "flywheel2");

        Servo hood     = hardwareMap.get(Servo.class, "hood");
        CRServo closer = hardwareMap.get(CRServo.class, "closer");

        // ---- Drive setup ----
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        double hoodpos = .6;
        double vel = 1000;

        boolean prevLB = false, prevRB = false;
        boolean prevxx = false, prevbb = false;

        waitForStart();

        while (opModeIsActive()) {

            // ================== MECANUM DRIVE (auto-aim yaw) ==================
            double axial     =  gamepad1.left_stick_y;
            double lateral   = -gamepad1.left_stick_x;
            double yaw       = -gamepad1.right_stick_x;

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max; fr /= max; bl /= max; br /= max;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            hood.setPosition(hoodpos);

            boolean lb = gamepad1.dpad_down;
            boolean rb = gamepad1.dpad_up;

            if (lb && !prevLB && !rb) {
                hoodpos = Range.clip(hoodpos + .1, 0.0, 1.0);
            } else if (rb && !prevRB && !lb) {
                hoodpos = Range.clip(hoodpos - .1, 0.0, 1.0);
            }

            prevLB = lb;
            prevRB = rb;

            // ================== INTAKE CONTROL (gamepad1 + gamepad2) ==================
            double rt = gamepad1.right_trigger + gamepad2.right_trigger;   // combined
            double lt = gamepad1.left_trigger + gamepad2.left_trigger;

            double intakeCmd = rt - lt;

            if (gamepad1.x) {
                flywheel1.setVelocity((vel * 28) / 60);
                flywheel2.setVelocity((vel * 28) / 60);
            } else if (gamepad1.y) {
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
            } else if (gamepad1.a) {
                flywheel1.setVelocity(((4000 * 28) / 60) * -.25);
                flywheel2.setVelocity(((4000 * 28) / 60) * -.25);
                closer.setPower(1);
            }

            boolean xx = gamepad1.x;
            boolean bb = gamepad1.b;

            if (xx && !prevxx && !bb) {
                vel = Range.clip(vel + 100, 0.0, 100000);
            } else if (bb && !prevbb && !xx) {
                vel = Range.clip(vel - 100, 0.0, 100000);
            }

            prevxx = xx;
            prevbb = bb;

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                intakeCmd = 1.0;
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                intakeCmd = -1.0;
            }

            if (intakeCmd > 0 && intakeCmd <= 1) {

                if (gamepad1.left_bumper == true) {
                    closer.setPower(-1);
                } else {
                    closer.setPower(1);
                }

            } else if (intakeCmd < 0 && intakeCmd >= -1) {

                if (gamepad1.right_bumper == true) {
                    closer.setPower(1);
                }

            } else {
                closer.setPower(0);
            }

            // BUMPER OVERRIDES (either controller)
            intake.setPower(intakeCmd);

            telemetry.addData("hood", "%.3f", hood.getPosition());
            telemetry.addData("hoodpos", "%.3f", hoodpos);
            telemetry.addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
            telemetry.addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
            telemetry.addData("target vel", "%.3f", vel);
            telemetry.update();
        }
    }
}
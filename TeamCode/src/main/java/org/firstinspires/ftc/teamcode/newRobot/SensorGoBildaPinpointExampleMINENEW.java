/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.newRobot;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.Register;
@Disabled
@TeleOp(name="goBILDA® Pinpoint Odometry Example MINE NEW", group="Linear OpMode")
//@Disabled
public class SensorGoBildaPinpointExampleMINENEW extends LinearOpMode {

    GoBildaPinpointDriver odo;
    Servo turret;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    double oldTime = 0;
    static final double target_x = -57.0;
    static final double target_y = 58.0;

    double pi = Math.PI;

    Register[] defaultRegisters = {
            Register.DEVICE_STATUS,
            Register.LOOP_TIME,
            Register.X_ENCODER_VALUE,
            Register.Y_ENCODER_VALUE,
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
            Register.H_VELOCITY,
    };

    Register[] onlyPosition = {
            Register.DEVICE_STATUS,
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
    };

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = hardwareMap.get(Servo.class,"turret");
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);


        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        odo.setOffsets(42, -33.55, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawManual = -gamepad1.right_stick_x;

            double yawCmd = yawManual;

            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            fl /= max; fr /= max; bl /= max; br /= max;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            odo.update();

            if (gamepad1.a){
                odo.resetPosAndIMU();
            }

            if (gamepad1.b){
                odo.recalibrateIMU();
            }

            if (gamepad1.dpad_down){
                odo.setBulkReadScope(onlyPosition);
            }

            /* turret */
            double rx = odo.getPosX(DistanceUnit.INCH);
            double ry = odo.getPosY(DistanceUnit.INCH);
            double heading = odo.getHeading(AngleUnit.RADIANS);

            double dx = target_x - rx;
            double dy = target_y - ry;

            double angleToGoal = Math.atan2(dy, dx);

            // Robot-relative target angle (this is the thing you want the turret to match)
            final double HEADING_COMP_GAIN = 1.1; // try 1.02–1.10
            double rel = angleToGoal - (heading * HEADING_COMP_GAIN);

            // Wrap robot-relative angle first
            while (rel > Math.PI)  rel -= 2.0 * Math.PI;
            while (rel < -Math.PI) rel += 2.0 * Math.PI;

            // If your physical turret "forward" is 180° off from the math zero, apply constant offset
            final double TURRET_FORWARD_OFFSET = Math.PI;   // your “backwards” fix
            double turretAngle = rel + TURRET_FORWARD_OFFSET;

            // Wrap again after offset
            while (turretAngle > Math.PI)  turretAngle -= 2.0 * Math.PI;
            while (turretAngle < -Math.PI) turretAngle += 2.0 * Math.PI;

            // ===== GEAR + SERVO TRAVEL (Axon Max 355°) =====
            final double SERVO_TRAVEL_REV = 355.0 / 360.0;
            final double SERVO_CENTER = 0.5;

            // keep your ratio (your “correct how it was”)
            // ^ im with idiot -Gage^
            final double SERVO_REV_PER_TURRET_REV = (152.0 / 50.0) * (1.0 / 3.0);

            double turretRev = turretAngle / (2.0 * Math.PI);
            double servoRev  = turretRev * SERVO_REV_PER_TURRET_REV;

// raw can go outside [0..1] — this is key to removing the 180° bump
            double servoPosRaw = SERVO_CENTER + (servoRev /*/ SERVO_TRAVEL_REV*/);

// stops (wire-safe window)
            double SERVO_LOWER_STOP = 0.10;
            double SERVO_UPPER_STOP = 0.90;

// pick the equivalent position inside the window closest to current -> no jump at wrap
            double currentPos = turret.getPosition();
            double servoPos = wrapToLimitsClosest(servoPosRaw, currentPos, SERVO_LOWER_STOP, SERVO_UPPER_STOP);

// optional tiny deadband to kill 1-tick chatter
            if (Math.abs(servoPos - currentPos) < 0.002) servoPos = currentPos;

            turret.setPosition(servoPos);

            telemetry.addData("dx", dx);
            telemetry.addData("dy", dy);
            telemetry.addData("angleToGoal deg", Math.toDegrees(angleToGoal));
            telemetry.addData("heading deg", Math.toDegrees(heading));
            telemetry.addData("rel deg", Math.toDegrees(rel));
            telemetry.addData("turretAngle deg", Math.toDegrees(turretAngle));
            telemetry.addData("turretRev rev", turretRev);
            telemetry.addData("servoRev rev", servoRev);
            telemetry.addData("currentTurret rev", currentPos);
            telemetry.addData("servoPosRaw rev", servoPosRaw);
            telemetry.addData("servoPosCmd rev", servoPos);
            //telemetry.addData("servoPos", servoPos);
            telemetry.addData("actual servo Position", turret.getPosition());

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("Position", odo.getPosition().toString());
            telemetry.addData("roll degrees", odo.getRoll(AngleUnit.DEGREES));
            telemetry.addData("pitch degrees", odo.getPitch(AngleUnit.DEGREES));

            telemetry.addData("x", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading Velocity degrees", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
            telemetry.addData("REV Hub Frequency: ", frequency);
            telemetry.update();
        }
    }
    private static double wrapRad(double a) {
        while (a > Math.PI)  a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    private static double wrapToLimitsClosest(double desired, double current, double lower, double upper) {
        double best = Double.NaN;
        double bestDist = Double.POSITIVE_INFINITY;

        for (int k = -2; k <= 2; k++) {
            double cand = desired + k; // shift by whole revs
            if (cand < lower || cand > upper) continue;

            double dist = Math.abs(cand - current);
            if (dist < bestDist) {
                bestDist = dist;
                best = cand;
            }
        }

        if (Double.isNaN(best)) {
            // fallback clamp
            if (desired < lower) return lower;
            if (desired > upper) return upper;
            return desired;
        }

        return best;
    }

}
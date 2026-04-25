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

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver.Register;
@Disabled
@TeleOp(name="turret Servo Test", group="Linear OpMode")
//@Disabled
public class turretServoTest extends LinearOpMode {

    GoBildaPinpointDriver odo;
    Servo turret;
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    double oldTime = 0;
    static final double target_y = 72;
    static final double target_x = -72;


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

        odo.setOffsets(-42, -90, DistanceUnit.MM);

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



            if (gamepad1.a){
                odo.resetPosAndIMU();
            }

            if (gamepad1.b){
                odo.recalibrateIMU();
            }

            if (gamepad1.dpad_down){
                odo.setBulkReadScope(onlyPosition);
            }

            // 1. Get targets relative to the robot's current position on the field
            double xleg = target_x - odo.getPosX(DistanceUnit.INCH);
            double yleg = target_y - odo.getPosY(DistanceUnit.INCH);

            double hypot = Math.sqrt(xleg * xleg + yleg * yleg);

            // 2. Calculate the "Field Centric" angle to the target
            // Math.atan2 returns -180 to +180 degrees
            double targetFieldHeading = Math.toDegrees(Math.atan2(yleg, xleg));

            // 3. Calculate angle relative to the robot
            // We subtract the robot's heading to make 0 degrees "Straight Forward"
            double robotHeading = odo.getHeading(AngleUnit.DEGREES);
            double relativeAngle = targetFieldHeading - robotHeading;

            // 4. Handle wrapping so relativeAngle is strictly between -180 and 180
            // This ensures the turret takes the shortest path
            while (relativeAngle > 180)  relativeAngle -= 360;
            while (relativeAngle < -180) relativeAngle += 360;

            // 5. Offset for Servo Alignment
            // Your servo is centered (facing forward) at 0.5.
            // 0.5 * 320 degrees = 197.5 degrees.
            // So, when relativeAngle is 0 (forward), servoDegrees should be 197.5.
            double servoDegrees = relativeAngle + 160;

            // 6. Clamp safety for the 0-320 range
            // If the target is directly behind you, the math might output < 0 or > 320.
            // We clamp it so the servo doesn't try to go past its physical stops.
            if (servoDegrees < 0) servoDegrees = 0;
            if (servoDegrees > 320) servoDegrees = 320;

            // 7. Set Position
            turret.setPosition(servoDegrees / 320.0);




            // Telemetry for debugging
            telemetry.addData("Debug: Delta X", xleg);
            telemetry.addData("Debug: Delta Y", yleg);
            telemetry.addData("Debug: Target Field Angle", targetFieldHeading);
            telemetry.addData("Debug: Relative Angle", relativeAngle);
            telemetry.addData("Debug: Servo Degrees", servoDegrees);


            odo.update();
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            telemetry.addData("Position", odo.getPosition().toString());
            telemetry.addData("roll degrees", odo.getRoll(AngleUnit.DEGREES));
            telemetry.addData("pitch degrees", odo.getPitch(AngleUnit.DEGREES));

            telemetry.addData("x", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("y", odo.getPosY(DistanceUnit.INCH));

            //telemetry.addData("x leg", xLeg);
            //telemetry.addData("y leg", yLeg);
            //telemetry.addData("hypot", hypot);
            //telemetry.addData("arctan", arcTan);
            telemetry.addData("turret servo pos", turret.getPosition());

            telemetry.addData("heading", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Heading Velocity degrees", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("Pinpoint Frequency", odo.getFrequency());
            telemetry.addData("REV Hub Frequency: ", frequency);
            telemetry.update();
        }
    }

}
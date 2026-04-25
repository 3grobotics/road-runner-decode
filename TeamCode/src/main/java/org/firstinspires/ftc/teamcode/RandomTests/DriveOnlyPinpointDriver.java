package org.firstinspires.ftc.teamcode.RandomTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Config
@TeleOp(name = "Drive Only (PinpointDriver)", group = "tests")
public class DriveOnlyPinpointDriver extends LinearOpMode {

    // Tunables
    public static double DRIVE_SCALE = 1.0;

    // Pinpoint offsets (MM) – matches your teleop style
    public static double ODO_X_OFFSET_MM = 42;
    public static double ODO_Y_OFFSET_MM = -33.55;

    @Override
    public void runOpMode() {

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Matches your teleopRedMatts directions exactly
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pinpoint init (trimmed but consistent with your teleop)
        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);

        odo.setOffsets(ODO_X_OFFSET_MM, ODO_Y_OFFSET_MM, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                 GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addLine("Drive Only (PinpointDriver) ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update odometry first (so telemetry is current)
            odo.update();

            // Same drive math structure as your teleopRedMatts block
            double axial   = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yawCmd  = -gamepad1.right_stick_x;

            double fl = axial + lateral + yawCmd;
            double fr = axial - lateral - yawCmd;
            double bl = axial - lateral + yawCmd;
            double br = axial + lateral - yawCmd;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(fr),
                                    Math.max(Math.abs(bl), Math.abs(br)))));

            fl = (fl / max) * DRIVE_SCALE;
            fr = (fr / max) * DRIVE_SCALE;
            bl = (bl / max) * DRIVE_SCALE;
            br = (br / max) * DRIVE_SCALE;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            telemetry.addData("odo x (in)", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("odo y (in)", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("heading (deg)", odo.getHeading(AngleUnit.DEGREES));
            telemetry.addData("heading vel (deg/s)", odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.update();
        }
    }
}

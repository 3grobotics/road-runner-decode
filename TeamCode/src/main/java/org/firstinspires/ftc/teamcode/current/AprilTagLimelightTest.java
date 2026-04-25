package org.firstinspires.ftc.teamcode.current;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// import frc.robot.LimelightHelpers;
@Disabled
@TeleOp
public class AprilTagLimelightTest extends OpMode {
    double distance;
    private Limelight3A limelight;
    private IMU imu;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        boolean success;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        success = limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose();
            Position thePosition = botpose.getPosition();

            double targetOffsetAngle_Vertical = llResult.getTy();

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 20;

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 13.84425;

            // distance from the target to the floor
            double goalHeightInches = 29.5;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            //Pose3D botpose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance area", distance);
            telemetry.addData("Distance better math inches", distanceFromLimelightToGoalInches);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Success", success);
            telemetry.addData("BotPose", botpose.toString());
            telemetry.addData("Pos.X", thePosition.x);
            telemetry.addData("Pos.Y", thePosition.y);
            telemetry.addData("Pos.Z", thePosition.z);
            telemetry.addData("Yaw", botpose.getOrientation().getYaw());
            telemetry.addData("IMU Yaw", orientation.getYaw());
            telemetry.addData("IMU roll", orientation.getRoll());
            telemetry.addData("IMU pitch", orientation.getPitch());
            telemetry.update();

        }

    }

    public double getDistanceFromTag(double ta) {
        double scale = 7800.179;
        double distance = (Math.pow(scale / ta, 1.0 / 1.737));
        return distance;
    }


}

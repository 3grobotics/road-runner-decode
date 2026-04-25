package org.firstinspires.ftc.teamcode.current;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

//import frc.robot.LimelightHelpers;
@Disabled
@TeleOp(name = "megatag2 test")
public class Megatag2Test extends OpMode {
     public Limelight3A limelight;
    private GoBildaPinpointDriver pip;


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


    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        pip = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pip.setOffsets(-42, -90, DistanceUnit.MM);
        pip.setBulkReadScope(defaultRegisters);
        pip.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        pip.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pip.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pip.resetPosAndIMU();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();


        // First, tell Limelight which way your robot is facing
        double robotYaw = pip.getHeading(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                telemetry.addData("heading", pip.getHeading(AngleUnit.DEGREES));
                telemetry.update();
            }
        }

    }
 }


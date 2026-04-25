package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class turret {

    private final GoBildaPinpointDriver odo;
    private final Servo turret;
    static final double target_y = 72;
    static final double target_x = -72;
    static double servoDegrees;
    Pose2D initialPose = new Pose2D(DistanceUnit.INCH, -46.5, 52.5, AngleUnit.DEGREES, 127);
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

    GoBildaPinpointDriver.Register[] onlyPosition = {
            GoBildaPinpointDriver.Register.DEVICE_STATUS,
            GoBildaPinpointDriver.Register.X_POSITION,
            GoBildaPinpointDriver.Register.Y_POSITION,
            GoBildaPinpointDriver.Register.H_ORIENTATION,
        };

    public boolean loopActive = false;




    public turret(HardwareMap hardwareMap) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = hardwareMap.get(Servo.class,"turret");



        odo.setBulkReadScope(defaultRegisters);
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
        odo.setOffsets(42, -33.55, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.setPosition(initialPose);


        //odo.setOffsets(-42, -90, DistanceUnit.MM);

    }

    /* ───────────────────────────────────────────────────────────────────────
     * Control loop for the main arm (call inside OpMode loop)               */
    public void loop() {
        if (!loopActive) return;

        // 1. Always update the hardware first to get fresh data
        odo.update();

        // 2. Calculate distances to target
        double xleg = target_x - odo.getPosX(DistanceUnit.INCH);
        double yleg = target_y - odo.getPosY(DistanceUnit.INCH);

        // 3. Calculate the "Field Centric" angle to the target
        double targetFieldHeading = Math.toDegrees(Math.atan2(yleg, xleg));

        // 4. Calculate angle relative to the robot
        double robotHeading = odo.getHeading(AngleUnit.DEGREES);
        double relativeAngle = targetFieldHeading - robotHeading;

        // 5. Handle wrapping so relativeAngle is strictly between -180 and 180
        while (relativeAngle > 180)  relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        // 6. Calculate Servo Degrees (Center point + offset)
        servoDegrees = relativeAngle + 160;

        // 7. Clamp safety for the 0-320 range
        if (servoDegrees < 0)   servoDegrees = 0;
        if (servoDegrees > 320) servoDegrees = 320;

        // 8. Set the actual position
        turret.setPosition((servoDegrees / 320.0) + -1);
    }


    public double getPos() {
        return turret.getPosition();
    }




    public void update() {
        odo.update();
    }

    public void reset() {
        odo.resetPosAndIMU();
    }


    public void setPosRedAutoClose() {
        odo.setPosition(initialPose);
    }



    public void setAuto() {
        loopActive = true;
        turret.setPosition((servoDegrees / 320.0) + -1);
    }


    }

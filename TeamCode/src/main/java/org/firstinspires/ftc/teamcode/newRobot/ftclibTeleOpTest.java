package org.firstinspires.ftc.teamcode.newRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
@Disabled
@TeleOp(name="FTCLib Potentiometer PID Example", group="Examples")
public class ftclibTeleOpTest extends OpMode {

    // Hardware
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;


    //-------------------------------------------------------------------------
    // These are now STATIC so that the FTC Dashboard can modify them live
    //-------------------------------------------------------------------------
    public static double vels;
    public static double kP = 1;
    public static double kI = 0.001;
    public static double kD = 0.0001;
    public static double kF = 0.001;
    //public static double kF = 0.06;      // If you have a known feed-forward, put it here
    public static double target = 2000;
    // public static double setpointVoltage = 1;
    public final double ticks_in_degrees = 0.2525;
    // FTCLib PIDFController
    private PIDController armController;

    // Dashboard instance (optional, only if you want telemetry on the Dashboard)
    private FtcDashboard dashboard;

    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);

        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        armController = new PIDController(kP, kI, kD);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
        // Runs repeatedly after init() but before play
    }

    @Override
    public void start() {
        // Called once when driver hits PLAY
    }

    @Override
    public void loop() {

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        flywheel1.setVelocity((target * 28) / 60);
        flywheel2.setVelocity((target * 28) / 60);

        /*// 1) Update the PIDFController with possibly changed parameters from the Dashboard
        armController.setPID(kP, kI, kD/*, kF);
        vels = ((flywheel1.getVelocity() * 28) / 60);
        // 2) Read the potentiometer voltage


        // 3) Compute the motor output
        //double output = armController.calculate(currentVoltage, setpointVoltage);
        double pid = armController.calculate(vels, targetVels);
        double target = Math.cos(Math.toRadians((targetVels * 28) / 60));

        double power = pid + target;
        // 4) Clip and set motor power
        power = Range.clip(power, -1.0, 1.0);
        flywheel1.setPower(power); //set to output if reverting
        flywheel2.setPower(power); //set to output if reverting
        */

        // ------------------------------
        // Phone Telemetry (Driver Station)
        // ------------------------------
        telemetry.addData("Target Vels", target);
        telemetry.addData("current vels", vels);
        //telemetry.addData("Output (Motor Power)", power);
        telemetry.addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 28));
        telemetry.addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 28));
        telemetry.addData("target tps", "%.3f",(target * 28) / 60);
        telemetry.addData("target rpm", "%.3f", target);
        telemetry.addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
        telemetry.addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
        telemetry.update();

        // ------------------------------
        // Dashboard Telemetry (optional)
        // ------------------------------
        dashboard.getTelemetry().addData("Target tps", target);
        dashboard.getTelemetry().addData("current vels", vels);
        dashboard.getTelemetry().addData("RPM flywheel 1", "%.3f", ((flywheel1.getVelocity() * 60) / 28));
        dashboard.getTelemetry().addData("RPM flywheel 2", "%.3f", ((flywheel2.getVelocity() * 60) / 28));
        dashboard.getTelemetry().addData("target tps", "%.3f",(target * 28) / 60);
        dashboard.getTelemetry().addData("target rpm", "%.3f", target);
        dashboard.getTelemetry().addData("actual vel flywheel 1", "%.3f", flywheel1.getVelocity());
        dashboard.getTelemetry().addData("actual vel flywheel 2", "%.3f", flywheel2.getVelocity());
        //dashboard.getTelemetry().addData("PID Output", power);
        dashboard.getTelemetry().update();
    }

    @Override
    public void stop() {
        // Called once when driver hits STOP
        flywheel1.setPower(0.0);
        flywheel2.setPower(0.0);
    }
}
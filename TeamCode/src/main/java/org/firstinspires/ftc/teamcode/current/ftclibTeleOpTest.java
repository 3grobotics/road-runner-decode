package org.firstinspires.ftc.teamcode.current;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config

@TeleOp(name="custom velocity pidf", group="Examples")
public class ftclibTeleOpTest extends OpMode {

    // Hardware
    private AnalogInput armPotentiometer;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    //-------------------------------------------------------------------------
    // These are now STATIC so that the FTC Dashboard can modify them live
    //-------------------------------------------------------------------------
    public static double kP = -3;
    public static double kI = 0.0000004;
    public static double kD = 0.0001;
    public static double kF = 0.06;      // If you have a known feed-forward, put it here
    public static double target = .9;
    // public static double setpointVelocity = 1;
    public final double ticks_in_degrees = 0.2525;
    // FTCLib PIDFController
    private PIDFController flywheelController;

    // Dashboard instance (optional, only if you want telemetry on the Dashboard)
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Map hardware
        armPotentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        
//        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize the PID controller with current static values
        flywheelController = new PIDFController(kP, kI, kD, kF);

        // Optional: get the FTC Dashboard instance so we can send telemetry there
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        // 1) Update the PIDFController with possibly changed parameters from the Dashboard
        flywheelController.setPIDF(kP, kI, kD, kF);

        // 2) Read the potentiometer velocity
        double currentVelocity1 = flywheel1.getVelocity();
        double currentVelocity2 = flywheel2.getVelocity();

        // 3) Compute the motor output
        //double output = flywheelController.calculate(currentVelocity1, setpointVelocity);
        double pid1 = flywheelController.calculate(currentVelocity1, target);
        double pid2 = flywheelController.calculate(currentVelocity2, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * kF;

        double power1 = pid1 + ff;
        double power2 = pid2 + ff;
        // 4) Clip and set motor power
        //output = Range.clip(output, -1.0, 1.0);
        flywheel1.setVelocity(power1); //set to output if reverting
        flywheel2.setVelocity(power2);

        // ------------------------------
        // Phone Telemetry (Driver Station)
        // ------------------------------
        telemetry.addData("Target Velocity", target);
        telemetry.addData("Current Velocity", currentVelocity1);
        telemetry.addData("Output (Motor Power)", power1);
        telemetry.update();

        // ------------------------------
        // Dashboard Telemetry (optional)
        // ------------------------------
        dashboard.getTelemetry().addData("Target Velocity", target);
        dashboard.getTelemetry().addData("Current Velocity1", currentVelocity1);
        dashboard.getTelemetry().addData("Current Velocity2", currentVelocity2);
        dashboard.getTelemetry().addData("PID Output1", power1);
        dashboard.getTelemetry().addData("PID Output2", power2);
        dashboard.getTelemetry().update();
    }

    @Override
    public void stop() {
        // Called once when driver hits STOP
        flywheel1.setPower(0.0);
    }
}
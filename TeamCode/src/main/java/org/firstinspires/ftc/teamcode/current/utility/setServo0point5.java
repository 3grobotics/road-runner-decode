
package org.firstinspires.ftc.teamcode.current.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="set servo 0.5", group="Linear OpMode")
//@Disabled
public class setServo0point5 extends LinearOpMode {

    Servo h, t1, t2, kr, kl;
    boolean bPressed = false;
    int twistState = 0;
    boolean dPressed = false;

    @Override
    public void runOpMode() {

        t1 = hardwareMap.get(Servo.class,"turret");
        t2 = hardwareMap.get(Servo.class,"turret2");
        kr = hardwareMap.get(Servo.class,"kickstandRight");
        kl = hardwareMap.get(Servo.class,"kickstandLeft");
        h = hardwareMap.get(Servo.class,"hood");
        h.setDirection(Servo.Direction.REVERSE);
        t1.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
        t1.setPosition(0.5);
        t2.setPosition(0.5);
        h.setPosition(0.5);

            if (gamepad1.b && !bPressed) {
                twistState = (twistState + 1) % 2;
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad2.dpad_up && !dPressed) {
                twistState = (twistState + 1) % 2;
                dPressed = true;
            } else if (!gamepad2.dpad_up) {
                dPressed = false;
            }
            switch (twistState){
                case 0:
                    kr.setPosition(.45);
                    kl.setPosition(.45);
                    break;
                case 1:
                    kr.setPosition(.5);
                    kl.setPosition(.5);
                    break;
            }
        telemetry.addData("hood position", h.getPosition());
        telemetry.update();
        }
    }
}
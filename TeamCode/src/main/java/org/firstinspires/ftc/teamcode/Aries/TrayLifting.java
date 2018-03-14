package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Khue on 3/10/18.
 */

public class TrayLifting extends LinearOpMode{

    public Servo TraySlide;
    public DcMotor TrayDumper;
    int TraySlideLength;
    public void runOpMode() {
        while (opModeIsActive()) {

            if (gamepad2.x) {
                TrayDumper.setTargetPosition(90);
            }
            if (gamepad2.dpad_up) {
                TrayDumper.setTargetPosition(10);
                TraySlide.setPosition(1);
            }
            if (gamepad2.dpad_down) {
                TraySlide.setPosition(TraySlideLength - 0.001);
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * Created by Khue on 3/7/18.
 */

public class RelicArmTest extends LinearOpMode {

    public DcMotor RelicExtendor;
    public Servo RelicClaw;
    public Servo RelicLifter;
    public int RelicClawCounter;
    public int RelicLifterCounter;
    public RelicExtendor = hardwareMap.dcMotor.get("RelicExtendor");
    public RelicClaw = hardwareMap.servo.get("RelicClaw");
    public RelicLifter = hardwareMap.servo.get("RelicLifter");

    public void runOpMode() {
        RelicExtendor.setPower(-gamepad2.right_stick_y);

        else if (RelicClawCounter == 0) {
            if (gamepad2.a) {
                RelicClaw.setPosition(100);
                RelicClawCounter++;

            }
        } else if (RelicLifterCounter == 0) {
            if (gamepad2.b) {
                RelicLifter.setPosition(90);
                RelicLifterCounter++;
            }
        }  else if (RelicClawCounter > 0) {
            if (gamepad2.a) {
                RelicClaw.setPosition(0);
                RelicClawCounter--;
            }
        } else if (RelicLifterCounter > 0) {
            if (gamepad2.b) {
                RelicLifter.setPosition(0);
            }
        }
    }
}
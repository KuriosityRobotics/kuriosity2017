<<<<<<< HEAD
package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
                RelicLifterCounter --;
            }
        }
    }
}
||||||| merged common ancestors
=======


package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 ** Created by Khue on 3/7/18.
 **/


@TeleOp(name="RelicArmTest", group="Linear Opmode")
//@Disabled
public class RelicArmTest extends LinearOpMode {

    // Declare OpMode members.
    public DcMotor RelicExtender;
    public Servo RelicClaw;
    public Servo RelicPivot;
    public int RelicLength = 0;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        RelicExtender = hardwareMap.dcMotor.get("relicSlide");
        RelicClaw = hardwareMap.servo.get("relicClaw");
        RelicPivot = hardwareMap.servo.get("relicPivot");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.dpad_right) {
                RelicExtender.setPower(1);
            } else if (gamepad2.dpad_left){
                RelicExtender.setPower(-1);
            }else{
                RelicExtender.setPower(0);
            }
            if (gamepad1.x) {
                RelicClaw.setPosition(0);
            } else if (gamepad1.y) {
                RelicPivot.setPosition(0.7);
            } else if (gamepad1.a) {
                RelicClaw.setPosition(0.6);
            } else if (gamepad1.b) {
                RelicPivot.setPosition(0.2);
            }
        }

        }
    }

>>>>>>> 0b2710147c53afacbc9aeace4f19ac6def50e533

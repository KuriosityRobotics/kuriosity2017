package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name="Aries OpMode", group="Linear Opmode")
//@Disabled
public class AriesMainTeleop extends LinearOpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Aries robot = new Aries(hardwareMap, telemetry, this);   //DO NOT DELETE

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            fLPower = (gamepad1.left_stick_y);
            bLPower = (gamepad1.left_stick_y);

            fRPower = (gamepad1.right_stick_y);
            bRPower = (gamepad1.right_stick_y);

            if (gamepad1.left_trigger != 0) {
                fLPower = gamepad1.left_trigger;
                bLPower = -gamepad1.left_trigger;
                fRPower = -gamepad1.left_trigger;
                bRPower = gamepad1.left_trigger;
            } else if (gamepad1.right_trigger != 0) {
                fLPower = -gamepad1.right_trigger;
                bLPower = gamepad1.right_trigger;
                fRPower = gamepad1.right_trigger;
                bRPower = -gamepad1.right_trigger;
            }


            if (gamepad1.dpad_up) {
                fLPower = (gamepad1.left_stick_y)-0.7;
                bLPower = (gamepad1.left_stick_y)-0.7;

                fRPower = (gamepad1.right_stick_y-0.7);
                bRPower = (gamepad1.right_stick_y)-0.7;

            } else if (gamepad1.dpad_down) {

                fLPower = (gamepad1.left_stick_y)+0.7;
                bLPower = (gamepad1.left_stick_y)+0.7;

                fRPower = (gamepad1.right_stick_y+0.7);
                bRPower = (gamepad1.right_stick_y)+0.7;

            } else if (gamepad1.dpad_right) {

                fRPower = (gamepad1.right_stick_y+0.7);
                bRPower = (gamepad1.right_stick_y)+0.7;
                fLPower = (gamepad1.left_stick_y)-0.7;
                bLPower = (gamepad1.left_stick_y)-0.7;

            } else if (gamepad1.dpad_left) {

                fLPower = (gamepad1.right_stick_y+0.7);
                bLPower = (gamepad1.right_stick_y)+0.7;
                fRPower = (gamepad1.left_stick_y)-0.7;
                bRPower = (gamepad1.left_stick_y)-0.7;

            }

                robot.fLeft.setPower(fLPower);
                robot.fRight.setPower(fRPower);
                robot.bLeft.setPower(bLPower);
                robot.bRight.setPower(bRPower);

        }

    }
}

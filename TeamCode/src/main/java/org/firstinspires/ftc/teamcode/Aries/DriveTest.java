package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="DriveTest", group="Linear Opmode")
//@Disabled
public class DriveTest extends LinearOpMode
{
    double fLeft;
    double fRright;
    double bLeft;
    double bRight;

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Aries robot = new Aries(hardwareMap, telemetry, this);   //DO NOT DELETE

        //Clears power values

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            //Strafe

                robot.fLeft.setPower(-gamepad1.left_stick_y);
                robot.bLeft.setPower(-gamepad1.left_stick_y);
                robot.fRight.setPower(-gamepad1.right_stick_y);
                robot.bRight.setPower(-gamepad1.right_stick_y);

//            if (gamepad1.right_trigger != 0) {
//                robot.fRight.setPower(-gamepad1.right_trigger);
//                robot.bLeft.setPower(-gamepad1.right_trigger);
//                robot.fLeft.setPower(gamepad1.right_trigger);
//                robot.bRight.setPower(gamepad1.right_trigger);
//            } else if (gamepad1.left_trigger != 0) {
//                robot.bRight.setPower(-gamepad1.left_trigger);
//                robot.fLeft.setPower(-gamepad1.left_trigger);
//                robot.fRight.setPower(gamepad1.left_trigger);
//                robot.bLeft.setPower(gamepad1.left_trigger);

            } if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_y > 0) {
                robot.fLeft.setPower(40);
                robot.bRight.setPower(40);
                robot.bLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                robot.fRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            } if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_x < 0) {
                robot.fLeft.setPower(-40);
                robot.bRight.setPower(-40);
                robot.bRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                robot.fLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            } if (gamepad1.left_stick_x < 0 && gamepad1.left_stick_x < 0) {
                robot.fRight.setPower(-40);
                robot.bLeft.setPower(-40);
                robot.bRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                robot.fLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            } if (gamepad1.left_stick_x > 0 && gamepad1.left_stick_x > 0) {
                robot.fRight.setPower(40);
                robot.bLeft.setPower(40);
                robot.bRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                robot.fLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            }   else {
                robot.fLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bLeft.setPower(0);
                robot.bRight.setPower(0);
            }

        }

    }
//}

package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="DriveTest", group="Linear Opmode")
//@Disabled
public class KhuesDriveTest extends LinearOpMode {
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
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            //Strafe
            if (gamepad1.left_stick_y != 0) {
                robot.fLeft.setPower(-gamepad1.left_stick_y);
                robot.bLeft.setPower(-gamepad1.left_stick_y);
            } if (gamepad1.right_stick_y != 0) {
                robot.fRight.setPower(-gamepad1.right_stick_y);
                robot.bRight.setPower(-gamepad1.right_stick_y);
            } if (gamepad1.left_trigger != 0) {
                robot.fRight.setPower(-gamepad1.left_trigger);
                robot.bLeft.setPower(-gamepad1.left_trigger);
                robot.fLeft.setPower(gamepad1.left_trigger);
                robot.bRight.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger != 0) {
                robot.bRight.setPower(-gamepad1.right_trigger);
                robot.fLeft.setPower(-gamepad1.right_trigger);
                robot.fRight.setPower(gamepad1.right_trigger);
                robot.bLeft.setPower(gamepad1.right_trigger);
            } else {
                robot.fLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bLeft.setPower(0);
                robot.bRight.setPower(0);
            } if (gamepad2.a) {
                robot.trayLeft.setPosition(90);
                robot.trayRight.setPosition(90);
            } else if (gamepad2.b) {
                robot.trayLeft.setPosition(0);
                robot.trayRight.setPosition(0);
            } if (gamepad2.dpad_up) {
                robot.linearSlideMotor.setPower(100);
            } if (gamepad2.dpad_down) {
                robot.linearSlideMotor.setPower(-100);
            } if (gamepad2.left_stick_y != 0) {
                robot.leftIntake.setPower(-50);
            } if (gamepad2.right_stick_y != 0) {
                robot.rightIntake.setPower(-50);
            } if (gamepad2.left_bumper) {
                robot.armServo.setPosition(100);
                robot.pivotServo.setPosition(120);
            } if (gamepad2.left_bumper) {
                robot.pivotServo.setPosition(0);
                robot.armServo.setPosition(0);
            } if (gamepad1.dpad_down) {
                robot.fLeft.setPower(100);
                robot.fRight.setPower(100);
                robot.bLeft.setPower(100);
                robot.bRight.setPower(100);
            } if (gamepad1.dpad_right) {
                robot.fRight.setPower(100);
                robot.bRight.setPower(100);
            } if (gamepad1.dpad_left) {
                robot.fLeft.setPower(100);
                robot.bLeft.setPower(100);
            } if (gamepad1.dpad_down) {
                robot.fRight.setPower(-100);
                robot.bRight.setPower(-100);
                robot.bLeft.setPower(-100);
                robot.fLeft.setPower(-100);
            }
        }
    }
}

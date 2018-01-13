package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name="Main TeleOp", group="Linear Opmode")
//@Disabled
public class MainTeleOp extends LinearOpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    long startTime = 0;
    boolean xActivated = false;
    boolean releaseRelic = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Kuro robot = new Kuro(hardwareMap, telemetry, this);   //DO NOT DELETE

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            try{
                //Calculate Power for motors (Non-meccanum drive)
                if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                    //When Sticks are not in resting position
                    fLPower = -gamepad1.left_stick_y;
                    bLPower = -gamepad1.left_stick_y;

                    fRPower = -gamepad1.right_stick_y;
                    bRPower = -gamepad1.right_stick_y;
                } else {
                    //When Sticks are in resting position, brake motors
                    fLPower = 0;
                    fRPower = 0;
                    bLPower = 0;
                    bRPower = 0;
                }

                //Calculate power for motors in Meccanum Drive
                if (gamepad1.left_trigger != 0) {
                    fLPower = -gamepad1.left_trigger;
                    bLPower = gamepad1.left_trigger;
                    fRPower = gamepad1.left_trigger;
                    bRPower = -gamepad1.left_trigger;
                } else if (gamepad1.right_trigger != 0) {
                    fLPower = gamepad1.right_trigger;
                    bLPower = -gamepad1.right_trigger;
                    fRPower = -gamepad1.right_trigger;
                    bRPower = gamepad1.right_trigger;
                }

                //slider power
                double sliderPower = (gamepad2.right_stick_y + gamepad2.left_stick_y) / 2;

                robot.left.setPower(sliderPower);
                robot.right.setPower(sliderPower);

                //dpad driving
                float left;
                float right;
                left = -gamepad1.left_stick_y + 3;
                right = -gamepad1.left_stick_y - 3;

                if (gamepad1.dpad_down) {
                        robot.fLeft.setPower(-left / 5);
                        robot.fRight.setPower(-left / 5);
                        robot.bLeft.setPower(right / 5);
                        robot.bRight.setPower(right / 5);

                } else if (gamepad1.dpad_up) {

                        robot.fLeft.setPower(left / 5);
                        robot.fRight.setPower(left / 5);
                        robot.bLeft.setPower(-right / 5);
                        robot.bRight.setPower(-right / 5);

                } else if (gamepad1.dpad_right) {

                        robot.fLeft.setPower(left / 5);
                        robot.fRight.setPower(-left / 5);
                        robot.bLeft.setPower(-right / 5);
                        robot.bRight.setPower(right / 5);

                } else if (gamepad1.dpad_left) {

                        robot.fLeft.setPower(-left / 5);
                        robot.fRight.setPower(left / 5);
                        robot.bLeft.setPower(right / 5);
                        robot.bRight.setPower(-right / 5);

                } else {

                    robot.fLeft.setPower(fLPower);
                    robot.fRight.setPower(fRPower);
                    robot.bLeft.setPower(bLPower);
                    robot.bRight.setPower(bRPower);
                }



                //claws
                if(gamepad2.x) {
                    xActivated = true;
                    startTime = SystemClock.elapsedRealtime();
                    robot.upRight.setPosition(0);
                    robot.upLeft.setPosition(1);
                }

                if (gamepad2.y) {
                    if (gamepad2.right_bumper) {
                        robot.upRight.setPosition(0.49);
                        robot.upLeft.setPosition(0.42);
                    } else {
                        robot.upRight.setPosition(0.63);
                        robot.upLeft.setPosition(0.3);
                    }
                }

                if(gamepad2.dpad_up){
                    robot.upLeft.setPosition(0.57);
                    robot.upRight.setPosition(0.38);

                    robot.downRight.setPosition(0.33);
                    robot.downLeft.setPosition(0.53);
                }

                if (gamepad2.a) {
                    robot.downRight.setPosition(1);
                    robot.downLeft.setPosition(0);
                } else if (gamepad2.b) {
                    if (gamepad2.right_bumper) {
                        robot.downRight.setPosition(0.2);
                        robot.downLeft.setPosition(0.67);
                    } else {
                        robot.downRight.setPosition(0.1);
                        robot.downLeft.setPosition(0.8);
                    }
                }


//                if(gamepad2.right_trigger>0){
//                    robot.grabRelic();
//                    releaseRelic = true;
//                }
//
//                if(gamepad2.left_trigger>0){
//                    robot.releaseRelic();
//                }
//
//                if(gamepad2.dpad_down){
//                    robot.relicSlide.setPower(1);
//                }else if(gamepad2.dpad_up){
//                    robot.relicSlide.setPower(-1);
//                    if((robot.relicSlide.getCurrentPosition() <= -190)){
//                        robot.relicClawRight.setPosition(0.5);
//                        robot.relicClawLeft.setPosition(0.5);
//                    }
//                }else{
//                    robot.relicSlide.setPower(0);
//                }

                if (gamepad1.a) {
                    robot.moveRobot(0.75, 800);
                }

                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // Elapsed Time
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Relic Slide Position", robot.relicSlide.getCurrentPosition());
                telemetry.addData("Relic Pivot Position", robot.relicPivot.getCurrentPosition());
                telemetry.update();
            }catch(Exception e){
                if(e instanceof InterruptedException){
                    continue;
                }
                else {
                    throw e;
                }
            }
        }
    }
}
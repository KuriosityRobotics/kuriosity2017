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
    double inputPower;
    boolean up = true;
    boolean down = true;
    long startTime = 0;
    int controlMode;
    boolean xActivated = false;
    boolean topIntakeAcitivated = false;
    boolean bottomIntakeActivated = false;
    boolean clawActivated;
    boolean grabberActivated;

    double aPressed = 0;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        final Kuro robot;


        //Init's robot
        robot = new Kuro(hardwareMap, telemetry,this);   //DO NOT DELETE

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;
        inputPower = 0.0;

        //Sets Control Mode to default
        controlMode = 0;

        //Sets claw and grabber to default
        clawActivated = false;
        grabberActivated = false;
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
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


            double sliderPower = (gamepad2.right_stick_y + gamepad2.left_stick_y) / 2;

            robot.left.setPower(sliderPower);
            robot.right.setPower(sliderPower);


                if (gamepad1.dpad_down) {
                    float left;
                    float right;
                    left = -gamepad1.left_stick_y + 3;
                    right = -gamepad1.left_stick_y - 3;

                    float max = Math.max(left, right);

                    if (gamepad1.left_bumper) {
                        robot.fLeft.setPower(-left / 12);
                        robot.fRight.setPower(-left / 12);
                        robot.bLeft.setPower(right / 12);
                        robot.bRight.setPower(right / 12);
                    } else {
                        robot.fLeft.setPower(-left / 5);
                        robot.fRight.setPower(-left / 5);
                        robot.bLeft.setPower(right / 5);
                        robot.bRight.setPower(right / 5);
                    }


                } else if (gamepad1.dpad_up) {
                    float left;
                    float right;
                    left = -gamepad1.left_stick_y + 3;
                    right = -gamepad1.left_stick_y - 3;

                    float max = Math.max(left, right);

                    if (gamepad1.left_bumper) {
                        robot.fLeft.setPower(left / 12);
                        robot.fRight.setPower(left / 12);
                        robot.bLeft.setPower(-right / 12);
                        robot.bRight.setPower(-right / 12);
                    } else {
                        robot.fLeft.setPower(left / 5);
                        robot.fRight.setPower(left / 5);
                        robot.bLeft.setPower(-right / 5);
                        robot.bRight.setPower(-right / 5);
                    }

                } else if (gamepad1.dpad_right) {
                    float left;
                    float right;
                    left = -gamepad1.left_stick_y + 3;
                    right = -gamepad1.left_stick_y - 3;

                    float max = Math.max(left, right);

                    if (gamepad1.left_bumper) {
                        robot.fLeft.setPower(left / 12);
                        robot.fRight.setPower(-left / 12);
                        robot.bLeft.setPower(-right / 12);
                        robot.bRight.setPower(right / 12);
                    } else {
                        robot.fLeft.setPower(left / 5);
                        robot.fRight.setPower(-left / 5);
                        robot.bLeft.setPower(-right / 5);
                        robot.bRight.setPower(right / 5);
                    }
                } else if (gamepad1.dpad_left) {
                    float left;
                    float right;
                    left = -gamepad1.left_stick_y + 3;
                    right = -gamepad1.left_stick_y - 3;

                    float max = Math.max(left, right);

                    if (gamepad1.left_bumper) {
                        robot.fLeft.setPower(-left / 12);
                        robot.fRight.setPower(left / 12);
                        robot.bLeft.setPower(right / 12);
                        robot.bRight.setPower(-right / 12);
                    } else {
                        robot.fLeft.setPower(-left / 5);
                        robot.fRight.setPower(left / 5);
                        robot.bLeft.setPower(right / 5);
                        robot.bRight.setPower(-right / 5);
                    }
                } else {
                    if (gamepad1.left_bumper) {
                        robot.fLeft.setPower(fLPower / 3);
                        robot.fRight.setPower(fRPower / 3);
                        robot.bLeft.setPower(bLPower / 3);
                        robot.bRight.setPower(bRPower / 3);
                    } else {
                        robot.fLeft.setPower(fLPower);
                        robot.fRight.setPower(fRPower);
                        robot.bLeft.setPower(bLPower);
                        robot.bRight.setPower(bRPower);
                    }
                }
            


            //claws
            if(gamepad2.x) {
                xActivated = true;
                startTime = SystemClock.elapsedRealtime();
                robot.upRight.setPosition(0);
                robot.upLeft.setPosition(1);
            }
//            }
//            if(SystemClock.elapsedRealtime()-startTime<=1000 && xActivated){
//                robot.inTopL.setPower(-1);
//                robot.inTopR.setPower(1);
//                sleep(100);
//            }else{
//                startTime = 0;
//                xActivated = false;
//            }





            if(gamepad2.left_trigger > 0.1) {
                if( !topIntakeAcitivated){
                    topIntakeAcitivated = true;

                    robot.upRight.setPosition(0.44);
                    robot.upLeft.setPosition(0.46);
                    robot.inTopL.setPower(-1);
                    robot.inTopR.setPower(1);
                }
            }else {
                if (topIntakeAcitivated) {
                    robot.upRight.setPosition(0);
                    robot.upLeft.setPosition(1);
                    robot.inTopL.setPower(0);
                    robot.inTopR.setPower(0);
                    topIntakeAcitivated = false;
                }
            }

            if(gamepad2.right_trigger > 0.1) {
                if( !bottomIntakeActivated){
                    bottomIntakeActivated = true;
                    robot.downRight.setPosition(0.33);
                    robot.downLeft.setPosition(0.67);

                    robot.inBottomL.setPower(1);
                    robot.inBottomR.setPower(-1);
                }
            }else {
                if (bottomIntakeActivated) {
                    robot.downRight.setPosition(0.6);
                    robot.downLeft.setPosition(0.4);
                    robot.inBottomL.setPower(0);
                    robot.inBottomR.setPower(0);
                    bottomIntakeActivated = false;
                }
            }



//                if(gamepad2.right_trigger==0) {
//                    robot.downRight.setPosition(0.45);
//                    robot.downLeft.setPosition(0.55);
//                }else {
//                    robot.inBottomL.setPower(gamepad2.right_trigger);
//                    robot.inBottomR.setPower(-gamepad2.right_trigger);
//                }
//
//


            if (gamepad2.y) {
                if (gamepad2.right_bumper) {
                    robot.upRight.setPosition(0.6);
                    robot.upLeft.setPosition(0.4);
                } else {
                    robot.upRight.setPosition(0.80);
                    robot.upLeft.setPosition(0.15);
                }
            }
            if (gamepad2.a) {
                robot.downRight.setPosition(1);
                robot.downLeft.setPosition(0);
            } else if (gamepad2.b) {
                if (gamepad2.right_bumper) {
                    robot.downRight.setPosition(0.35);
                    robot.downLeft.setPosition(0.65);
                } else {
                    robot.downRight.setPosition(0.2);
                    robot.downLeft.setPosition(0.8);
                }
            }


            if (gamepad1.a) {
                robot.moveRobot(0.75, 800);
                telemetry.addLine("Finished moving");
            }

            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            // Elapsed Time
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addLine();
            telemetry.addLine("Drive Motor Powers:");
            telemetry.addData("Front Left", fLPower);
            telemetry.addData("Front Right", fRPower);
            telemetry.addData("Back Left", bLPower);
            telemetry.addData("Back Right", bRPower);
            telemetry.addData("Angle", robot.angles.firstAngle);
        }
    }
}

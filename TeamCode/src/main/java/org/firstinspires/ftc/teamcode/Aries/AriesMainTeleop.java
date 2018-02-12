package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Aries: MainTeleOp", group="Linear Opmode")
//@Disabled
public class AriesMainTeleop extends LinearOpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    boolean trayIsDown;
    boolean isCorrected;

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Aries robot = new Aries(hardwareMap, telemetry, this);   //DO NOT DELETE

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.trayPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.trayPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

        //tray position values
        trayIsDown = true;
        isCorrected = true;

        //Makes tray stop at postion
        robot.trayPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            //tank drive
            fLPower = -(gamepad1.left_stick_y);
            bLPower = -(gamepad1.left_stick_y);
            fRPower = -(gamepad1.right_stick_y);
            bRPower = -(gamepad1.right_stick_y);

            //Strafe
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

            //Straight D-Pad move
            if (gamepad1.dpad_up) {
                fLPower = (gamepad1.left_stick_y)+0.7;
                bLPower = (gamepad1.left_stick_y)+0.7;
                fRPower = (gamepad1.right_stick_y+0.7);
                bRPower = (gamepad1.right_stick_y+0.7);

            } else if (gamepad1.dpad_down) {
                fLPower = (gamepad1.left_stick_y)-0.7;
                bLPower = (gamepad1.left_stick_y)-0.7;
                fRPower = (gamepad1.right_stick_y-0.7);
                bRPower = (gamepad1.right_stick_y)-0.7;

            } else if (gamepad1.dpad_right) {

                fLPower = (gamepad1.right_stick_y+0.7);
                bLPower = (gamepad1.right_stick_y)+0.7;
                fRPower = (gamepad1.left_stick_y)-0.7;
                bRPower = (gamepad1.left_stick_y)-0.7;

            } else if (gamepad1.dpad_left) {

                fRPower = (gamepad1.right_stick_y+0.7);
                bRPower = (gamepad1.right_stick_y)+0.7;
                fLPower = (gamepad1.left_stick_y)-0.7;
                bLPower = (gamepad1.left_stick_y)-0.7;

            }

            //Sets power of motor to set value
            robot.fLeft.setPower(fLPower);
            robot.fRight.setPower(fRPower);
            robot.bLeft.setPower(bLPower);
            robot.bRight.setPower(bRPower);

            //Intake
            double intakePowerLeft = -gamepad2.left_stick_y;
            double intakePowerRight = -gamepad2.right_stick_y;
            robot.leftIntake.setPower(intakePowerRight);
            robot.rightIntake.setPower(intakePowerLeft);

            //Linear slide

            //Tray Operation
            if(gamepad2.dpad_up){
                robot.trayPivot.setPower(1);
                robot.trayPivot.setTargetPosition(32);
                robot.fLeft.setPower(0);
                robot.bLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bRight.setPower(0);
                sleep(500);
                robot.linearSlideMotor.setPower(1);
                robot.linearSlideMotor.setTargetPosition(1550);
                robot.linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }else if(gamepad2.dpad_down){
                robot.linearSlideMotor.setPower(1);
                robot.linearSlideMotor.setTargetPosition(0);
                robot.fLeft.setPower(0);
                robot.bLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bRight.setPower(0);
                sleep(500);

                robot.trayPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.trayPivot.setPower(1);
                robot.trayPivot.setTargetPosition(3);

            }

            if(gamepad2.x){
                robot.trayPivot.setPower(1);
                robot.trayPivot.setTargetPosition(80);
                while(robot.trayPivot.isBusy() && this.opModeIsActive() && (!gamepad1.a||!gamepad2.a)) {
                    robot.trayPivot.setPower(1);
                    robot.trayPivot.setTargetPosition(80);
                }
            }


            if(gamepad2.y || gamepad1.y){
                robot.armServo.setPosition(0.65);
                robot.pivotServo.setPosition(1);
            }else if(gamepad2.b || gamepad1.b){
                robot.pivotServo.setPosition(0.45);
                robot.armServo.setPosition(0);
            }

            if(gamepad1.x){
                robot.moveRobotInches(0.8,16);
            }



            telemetry.addData("Linear slide motor", robot.linearSlideMotor.getCurrentPosition());
            telemetry.addData("Tray position", robot.trayPivot.getCurrentPosition());
            telemetry.update();
        }
    }
}

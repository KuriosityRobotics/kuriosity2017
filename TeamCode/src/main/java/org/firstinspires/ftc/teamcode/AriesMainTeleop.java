package org.firstinspires.ftc.teamcode;

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

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Aries robot = new Aries(hardwareMap, telemetry, this);   //DO NOT DELETE

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.trayPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.trayPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.trayPivot.setPower(0.5);

        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

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
            double intakePower = -gamepad2.left_stick_y;
            robot.leftIntake.setPower(intakePower);
            robot.rightIntake.setPower(intakePower);

            //Linear slide
            robot.linearSlideMotor.setPower(-gamepad2.right_stick_y);

            //Tray Operation
            if(gamepad2.y){
                robot.trayPivot.setTargetPosition(5);
            }else if(gamepad2.x){
                robot.trayPivot.setTargetPosition(0);
            }
        }
    }
}

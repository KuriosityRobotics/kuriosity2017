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


//        robot.trayPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.trayPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double origin = robot.relicSlide.getCurrentPosition();

        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.relicSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Clears power values
        fLPower = 0.0;
        fRPower = 0.0;
        bLPower = 0.0;
        bRPower = 0.0;

        //tray position values
        trayIsDown = true;
        isCorrected = true;

        //Makes tray stop at postion
//        robot.trayPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            robot.relicSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
// else if(gamepad1.left_stick_y!=0) {
//
//                fRPower = 0.03;
//                bLPower = 0.03;
//                fLPower = -gamepad1.left_stick_y;
//                bRPower = -gamepad1.left_stick_y;
//
//            }else if(gamepad1.right_stick_y!=0){
//
//                fLPower = 0.03;
//                bRPower = 0.03;
//                fRPower = -gamepad1.right_stick_y;
//                bLPower = -gamepad1.right_stick_y;
//
//            }
//
            //Sets power of motor to set value


            robot.fLeft.setPower(fLPower);
            robot.fRight.setPower(fRPower);
            robot.bLeft.setPower(bLPower);
            robot.bRight.setPower(bRPower);
            //Intake
            double intakePowerLeft = -gamepad2.left_stick_y;
            double intakePowerRight = -gamepad2.right_stick_y;
            robot.leftIntake.setPower(intakePowerLeft);
            robot.rightIntake.setPower(intakePowerRight);

            //Linear slide

            //Tray Operation
            if(gamepad2.dpad_up){
                robot.trayRight.setPosition(0.28);
                robot.trayLeft.setPosition(0.72);
                sleep(500);
                robot.fLeft.setPower(0);
                robot.bLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bRight.setPower(0);

                robot.linearSlideMotor.setPower(1);
                robot.linearSlideMotor.setTargetPosition(1500);
                robot.linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            }else if(gamepad2.dpad_down){

                robot.linearSlideMotor.setPower(1);
                robot.linearSlideMotor.setTargetPosition(0);
                robot.fLeft.setPower(0);
                robot.bLeft.setPower(0);
                robot.fRight.setPower(0);
                robot.bRight.setPower(0);
                sleep(500);
                robot.trayRight.setPosition(0.12);
                robot.trayLeft.setPosition(0.88);

            }

            if(gamepad2.x){
                robot.trayRight.setPosition(0.725);
                robot.trayLeft.setPosition(0.275);
            }

            if(gamepad2.right_trigger>0){
//                robot.trayPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.trayPivot.setPower(gamepad2.right_trigger/2);
            }


            if(gamepad1.y){
                robot.armServo.setPosition(0.625);
                robot.pivotServo.setPosition(1);
            }else if(gamepad1.b){
                robot.pivotServo.setPosition(0.45);
                robot.armServo.setPosition(0);
            }

            if(gamepad1.left_bumper){
                robot.moveRobotInches(0.8,16);
            }







            if(gamepad1.a){
                robot.relicClaw.setPosition(0.2);
            }else if(gamepad1.x && gamepad1.right_bumper){
                robot.relicClaw.setPosition(1);
            }else if(gamepad1.x){
                robot.relicClaw.setPosition(1);
                sleep(500);
                robot.relicSlide.setPower(-1);
                sleep(750);
            }
            // double currentPosition = Math.abs(Math.abs(robot.relicSlide.getCurrentPosition())-origin);
            // if(currentPosition >100 && currentPosition <300 && !gamepad2.b && !gamepad1.right_bumper) {
            //     robot.relicPivot.setPosition(0.1);
            // }else if(currentPosition > 300 && !gamepad2.y && !gamepad1.right_bumper){
            //     robot.relicPivot.setPosition(0.6);
            // }else{
            //     robot.relicSlide.setPower(0.05);
            // }

            if(gamepad2.y){
                robot.relicPivot.setPosition(0.4);

            }else if(gamepad2.b) {
                robot.relicPivot.setPosition(0.4575);
            }

            if(gamepad2.dpad_left){
                robot.relicSlide.setPower(1);
            }else if(gamepad2.dpad_right){
                robot.relicSlide.setPower(-1);
            }else{
                robot.relicSlide.setPower(0);
            }






            telemetry.addData("Linear slide motor", robot.linearSlideMotor.getCurrentPosition());
            telemetry.addData("trayRight", robot.trayRight.getPosition());
            telemetry.addData("trayLeft", robot.trayLeft.getPosition());
            telemetry.addData("relicSlide",robot.relicSlide.getCurrentPosition());
            telemetry.addData("relicPivot", robot.relicPivot.getPosition());
            telemetry.addData("relicPivot", robot.relicPivot.getPosition());
            telemetry.addData("cryptservo",robot.cryptoServo.getPosition());

//            telemetry.addData("Tray position", robot.trayPivot.getCurrentPosition());
            telemetry.update();
        }
    }
}
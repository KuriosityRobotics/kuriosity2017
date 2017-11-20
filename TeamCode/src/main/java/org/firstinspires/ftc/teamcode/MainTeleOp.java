package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Main TeleOp", group="Iterative Opmode")
//@Disabled
public class MainTeleOp extends OpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    double inputPower;
    boolean up = true;
    boolean down = true;
    int controlMode;

    boolean clawActivated;
    boolean grabberActivated;

    ElapsedTime runtime = new ElapsedTime();

    Kuro robot;


    @Override
    public void init() {
        //Init's robot
        robot = new Kuro(hardwareMap);   //DO NOT DELETE

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

        //Adds telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    @Override
    public void init_loop() {
    }



    @Override
    public void start() {
        runtime.reset();
    }



    @Override
    public void loop() {
        //Calculate Power for motors (Non-meccanum drive)
        if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0){
            //When Sticks are not in resting position
            fLPower = -gamepad1.left_stick_y;
            bLPower = -gamepad1.left_stick_y;

            fRPower = -gamepad1.right_stick_y;
            bRPower = -gamepad1.right_stick_y;
        }else{
            //When Sticks are in resting position, brake motors
            fLPower = 0;
            fRPower = 0;
            bLPower = 0;
            bRPower = 0;
        }

        //Calculate power for motors in Meccanum Drive
        if(!(gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0)){
            if(gamepad1.left_trigger != 0){
                fLPower = -gamepad1.left_trigger;
                bLPower = gamepad1.left_trigger;
                fRPower = gamepad1.left_trigger;
                bRPower = -gamepad1.left_trigger;
            }else if(gamepad1.right_trigger != 0){
                fLPower = gamepad1.right_trigger;
                bLPower = -gamepad1.right_trigger;
                fRPower = -gamepad1.right_trigger;
                bRPower = gamepad1.right_trigger;
            }
        }

        robot.left.setPower(-gamepad2.right_stick_y/2);
        robot.right.setPower(-gamepad2.right_stick_y/2);
        double triggerVal2 = gamepad1.left_trigger;
        triggerVal2 = Range.clip(triggerVal2, 0 , 0.75);
        //Set Power to the drive motors
        /**int mode = 0;
        if (gamepad1.dpad_left) {
            mode = 0;
        }else if (gamepad1.dpad_right) {
            mode = 1;
        }
        switch (mode){
            case 0:
                if(gamepad1.left_bumper) {
                    robot.fLeft.setPower(fLPower / 3);
                    robot.fRight.setPower(fRPower / 3);
                    robot.bLeft.setPower(bLPower / 3);
                    robot.bRight.setPower(bRPower / 3);
                }else{
                    robot.fLeft.setPower(fLPower);
                    robot.fRight.setPower(fRPower);
                    robot.bLeft.setPower(bLPower);
                    robot.bRight.setPower(bRPower);
                }
            case 1:
                float left;
                float right;
                left = -gamepad1.left_stick_y + 3 * (gamepad1.right_stick_x);
                right = -gamepad1.left_stick_y - 3 * (gamepad1.right_stick_x);

                float max = Math.max(left, right);

                if (max > 1.00) {
                    left /= max;
                    right /= max;
                }
                if(gamepad1.left_bumper){
                    robot.fLeft.setPower(-left/3);
                    robot.fRight.setPower(left/3);
                    robot.bLeft.setPower(right/3);
                    robot.bRight.setPower(-right/3);
                }else {
                    robot.fLeft.setPower(left);
                    robot.fRight.setPower(-left);
                    robot.bLeft.setPower(-right);
                    robot.bRight.setPower(right);
                }

        }
         */
        if(gamepad1.dpad_down){
            float left;
            float right;
            left = -gamepad1.left_stick_y + 3;
            right = -gamepad1.left_stick_y - 3 ;

            float max = Math.max(left, right);

            if(gamepad1.left_bumper) {
                robot.fLeft.setPower(-left/12);
                robot.fRight.setPower(-left/12);
                robot.bLeft.setPower(right/12);
                robot.bRight.setPower(right/12);
            }else{
                robot.fLeft.setPower(-left/5);
                robot.fRight.setPower(-left/5);
                robot.bLeft.setPower(right/5);
                robot.bRight.setPower(right/5);
            }


        }else if(gamepad1.dpad_up){
            float left;
            float right;
            left = -gamepad1.left_stick_y + 3;
            right = -gamepad1.left_stick_y - 3 ;

            float max = Math.max(left, right);

            if(gamepad1.left_bumper) {
                robot.fLeft.setPower(left/12);
                robot.fRight.setPower(left/12);
                robot.bLeft.setPower(-right/12);
                robot.bRight.setPower(-right/12);
            }else {
                robot.fLeft.setPower(left / 5);
                robot.fRight.setPower(left / 5);
                robot.bLeft.setPower(-right / 5);
                robot.bRight.setPower(-right / 5);
            }

        }else if(gamepad1.dpad_right){
            float left;
            float right;
            left = -gamepad1.left_stick_y + 3;
            right = -gamepad1.left_stick_y - 3 ;

            float max = Math.max(left, right);

            if(gamepad1.left_bumper) {
                robot.fLeft.setPower(left/12);
                robot.fRight.setPower(-left/12);
                robot.bLeft.setPower(-right/12);
                robot.bRight.setPower(right/12);
            }else {
                robot.fLeft.setPower(left / 5);
                robot.fRight.setPower(-left / 5);
                robot.bLeft.setPower(-right / 5);
                robot.bRight.setPower(right / 5);
            }
        }else if(gamepad1.dpad_left){
            float left;
            float right;
            left = -gamepad1.left_stick_y + 3;
            right = -gamepad1.left_stick_y - 3 ;

            float max = Math.max(left, right);

            if(gamepad1.left_bumper) {
                robot.fLeft.setPower(-left/12);
                robot.fRight.setPower(left/12);
                robot.bLeft.setPower(right/12);
                robot.bRight.setPower(-right/12);
            }else{
                robot.fLeft.setPower(-left/5);
                robot.fRight.setPower(left/5);
                robot.bLeft.setPower(right/5);
                robot.bRight.setPower(-right/5);
            }
        }
        else {
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

        if(gamepad2.x){
            robot.upRight.setPosition(0.80);
            robot.upLeft.setPosition(0.15);
        }else if(gamepad2.y){
            robot.upRight.setPosition(0.02);
            robot.upLeft.setPosition(0.9);
        }
        if(gamepad2.a) {
            robot.downRight.setPosition(1);
            robot.downLeft.setPosition(0);
        }else if(gamepad2.b){
            robot.downRight.setPosition(0.2);
            robot.downLeft.setPosition(0.8);
        }
        if(gamepad2.left_bumper){
            robot.upRight.setPosition(0.1);
            robot.upLeft.setPosition(0.9);
        }
        if(gamepad2.right_bumper){
            robot.downRight.setPosition(1);
            robot.downLeft.setPosition(0);
        }

        if(gamepad2.left_trigger>0){
            if(gamepad2.right_stick_y==0){
                robot.left.setPower(0.3);
                robot.right.setPower(0.22);
            }
        }

        // Elapsed Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addLine();
        telemetry.addLine("Drive Motor Powers:");
        telemetry.addData("Front Left", fLPower);
        telemetry.addData("Front Right", fRPower);
        telemetry.addData("Back Left", bLPower);
        telemetry.addData("Back Right", bRPower);
    }



    @Override
    public void stop() {
    }
}

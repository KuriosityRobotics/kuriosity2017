package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Maccanum Control", group="Iterative Opmode")
//@Disabled
public class MeccanumControl extends OpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;
    double inputPower;

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

        //Set Power to the drive motors
        robot.fLeft.setPower(fLPower);
        robot.fRight.setPower(fRPower);
        robot.bLeft.setPower(bLPower);
        robot.bRight.setPower(bRPower);

        inputPower = gamepad2.right_stick_y;

        //Set power to intake motors
        robot.intakeLeft.setPower(inputPower);
        robot.intakeRight.setPower(inputPower);

        robot.arm.setPower((gamepad2.right_stick_y/3));

        //Takes input for claw position
        if(gamepad2.a){
            clawActivated = true;
        }else if(gamepad2.b){
            clawActivated = false;
        }

        //takes input for grabber position
        if(gamepad2.x){
            grabberActivated = true;
        }else if(gamepad2.y){
            grabberActivated = false;
        }

        //Moves claw to correct position
        if(clawActivated){
            robot.leftClaw.setPosition(0.7);
            robot.rightClaw.setPosition(0.4);
        }else{
            robot.leftClaw.setPosition(0.4);
            robot.rightClaw.setPosition(0.7);
        }

        //Moves grabber to correct position
        if(grabberActivated){
            robot.leftGrabber.setPosition(1);
            robot.rightGrabber.setPosition(0.02);
        }else{
            robot.leftGrabber.setPosition(0.02);
            robot.rightGrabber.setPosition(1);
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

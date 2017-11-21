package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red: Facing", group="Linear Opmode")
//@Disabled
public class RedFacing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap);

        resetEncoders(robot);

        //causing problem below
        //JewelArm.jewelArm(hardwareMap);
        //closeClaws(robot);
        //moveSlide(robot);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int moveFoward = -1500;
            double power = 0.5;
            robot.fLeft.setPower(power);
            robot.fRight.setPower(power);
            robot.bLeft.setPower(power);
            robot.bRight.setPower(power);


            robot.fLeft.setTargetPosition(moveFoward);
            robot.fRight.setTargetPosition(moveFoward);
            robot.bLeft.setTargetPosition(moveFoward);
            robot.bRight.setTargetPosition(moveFoward);

            power = 0.25;
            robot.fLeft.setPower(power);
            robot.fRight.setPower(power);
            robot.bLeft.setPower(power);
            robot.bRight.setPower(power);;
            moveFoward = -1600;

            robot.fLeft.setTargetPosition(moveFoward);
            robot.fRight.setTargetPosition(moveFoward);
            robot.bLeft.setTargetPosition(moveFoward);
            robot.bRight.setTargetPosition(moveFoward);
            Thread.sleep(3000);
            if(robot.bLeft.getCurrentPosition() <= moveFoward){
                turn(robot);
            }


            robot.fLeft.setPower(0);
            robot.fRight.setPower(0);
            robot.bLeft.setPower(0);
            robot.bRight.setPower(0);

            Thread.sleep(500);

            robot.resetEncoders();

            robot.fLeft.setTargetPosition(800);
            robot.fRight.setTargetPosition(800);
            robot.bLeft.setTargetPosition(800);
            robot.bRight.setTargetPosition(800);

            robot.setDrivePower(0.1);

            Thread.sleep(1000000);
        }
    }

    public void turn(Kuro robot){
        robot.resetEncoders();

        robot.fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle = 0;

        while(robot.angles.firstAngle < 60){
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.fLeft.setPower(-0.5);
            robot.fRight.setPower(0.5);
            robot.bLeft.setPower(-0.5);
            robot.bRight.setPower(0.5);
            telemetry.addLine(robot.angles.firstAngle + "");
            telemetry.addLine("Turning Under 60");
            telemetry.update();
        }

        while (robot.angles.firstAngle < 74){
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.fLeft.setPower(-0.25);
            robot.fRight.setPower(0.25);
            robot.bLeft.setPower(-0.25);
            robot.bRight.setPower(0.25);
            telemetry.addLine("Angle is  " + robot.angles.firstAngle);
            telemetry.update();

        }
    }

    public void resetEncoders(Kuro robot){
        robot.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armServo.setPosition(0.1);
    }

    public void closeClaws(Kuro robot){
        robot.upRight.setPosition(0.02);
        robot.upLeft.setPosition(0.9);
        sleep(500);
    }

    public void moveSlide(Kuro robot){
        robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left.setPower(0.2);
        robot.right.setPower(0.2);
        robot.right.setTargetPosition(1);
        sleep(100000);

    }

    public moveRobot(double speed, int targetPostition,Kuro robot){
        if(targetPostition > 0) {

                robot.fLeft.setPower(speed);
                robot.fRight.setPower(speed);
                robot.bLeft.setPower(speed);
                robot.bRight.setPower(speed);

                robot.fLeft.setTargetPosition(targetPostition);
                robot.fRight.setTargetPosition(targetPostition);
                robot.bLeft.setTargetPosition(targetPostition);
                robot.bRight.setTargetPosition(targetPostition);
                sleep(500);
                if(robot.fLeft.getCurrentPosition() < targetPostition) {
                    return;
                }
        }else{

                robot.fLeft.setPower(speed);
                robot.fRight.setPower(speed);
                robot.bLeft.setPower(speed);
                robot.bRight.setPower(speed);

                robot.fLeft.setTargetPosition(targetPostition);
                robot.fRight.setTargetPosition(targetPostition);
                robot.bLeft.setTargetPosition(targetPostition);
                robot.bRight.setTargetPosition(targetPostition);

                if(robot.fLeft.getCurrentPosition() > targetPostition) {
                    return;
                }
        }
    }
}

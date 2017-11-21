package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
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

        robot.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.armServo.setPosition(0.1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.fLeft.setPower(0.5);
            robot.fRight.setPower(0.5);
            robot.bLeft.setPower(0.5);
            robot.bRight.setPower(0.5);

            robot.fLeft.setTargetPosition(-1000);
            robot.fRight.setTargetPosition(-1000);
            robot.bLeft.setTargetPosition(-1000);
            robot.bRight.setTargetPosition(-1000);

            robot.fLeft.setPower(0.25);
            robot.fRight.setPower(0.25);
            robot.bLeft.setPower(0.25);
            robot.bRight.setPower(0.25);;

            robot.fLeft.setTargetPosition(-1475);
            robot.fRight.setTargetPosition(-1475);
            robot.bLeft.setTargetPosition(-1475);
            robot.bRight.setTargetPosition(-1475);

            Thread.sleep(2500);

            resetEncoders(robot);

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

            while (robot.angles.firstAngle < 90){
                robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robot.fLeft.setPower(-0.25);
                robot.fRight.setPower(0.25);
                robot.bLeft.setPower(-0.25);
                robot.bRight.setPower(0.25);
                telemetry.addLine("Turning Under 90");
                telemetry.update();

            }

            robot.fLeft.setPower(0);
            robot.fRight.setPower(0);
            robot.bLeft.setPower(0);
            robot.bRight.setPower(0);



            Thread.sleep(500);


            Thread.sleep(1000000);
        }
    }

    public void turnOff(Kuro robot){
        robot.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fLeft.setPower(0);
        robot.fRight.setPower(0);
        robot.bLeft.setPower(0);
        robot.bRight.setPower(0);
    }

    public void resetEncoders(Kuro robot){
        robot.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

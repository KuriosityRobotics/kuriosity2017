package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


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
        KuroVuforiaPictograph pictograph = new KuroVuforiaPictograph();

        RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 5000);

        resetEncoders(robot);

        //causing problem below



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            JewelArm.jewelArm(hardwareMap);
            closeClaws(robot);
            moveSlide(0.25,500, robot);
            sleep(1000);

            if(vuMark == RelicRecoveryVuMark.CENTER){
                moveRobot(0.5,-1000,robot);
                moveRobot(0.25,-600,robot);
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){
                moveRobot(0.5,-1000,robot);
                moveRobot(0.25,-300,robot);
            }else if(vuMark == RelicRecoveryVuMark.LEFT){
                moveRobot(0.5,-1000,robot);
                moveRobot(0.25,-900,robot);
            }
            turn(90,robot);
            robot.resetEncoders();
            moveRobot(0.25,800,robot);
            robot.setDrivePower(0.1);
            sleep(2000);
            openClaws(robot);
            Thread.sleep(1000000);

        }
    }

    public void turn(double degrees, Kuro robot){
        robot.resetEncoders();

        changeRunModeToUsingEncoder(robot);

        while (Math.abs(robot.angles.firstAngle) <= Math.abs(degrees)){
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double deltatAngle = Math.abs(degrees) - Math.abs(robot.angles.firstAngle);
            double power = 1 - (Math.abs(robot.angles.firstAngle) / Math.abs(degrees));
            if(Math.abs(robot.angles.firstAngle) >= Math.abs(degrees)){
                break;
            }
            robot.fLeft.setPower(-power);
            robot.fRight.setPower(power);
            robot.bLeft.setPower(-power);
            robot.bRight.setPower(power);
        }

        robot.fLeft.setPower(0);
        robot.fRight.setPower(0);
        robot.bLeft.setPower(0);
        robot.bRight.setPower(0);

        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addLine("Angle is  " + robot.angles.firstAngle);
        telemetry.update();

        resumeEncoders(robot);
    }

    public void resetEncoders(Kuro robot){
        robot.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.armServo.setPosition(0.1);
    }

    public void closeClaws(Kuro robot){
        robot.upRight.setPosition(0.02);
        robot.upLeft.setPosition(0.9);
        sleep(500);

    }

    public void moveSlide(double power, int targetPosition, Kuro robot){
        robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left.setPower(power);
        robot.right.setPower(power);

        robot.right.setTargetPosition(targetPosition);
        robot.left.setTargetPosition(targetPosition);

        while(robot.left.isBusy() && robot.right.isBusy() ){
            sleep(10);
        }
        robot.left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void moveRobot(double speed, int targetPostition, Kuro robot){

        robot.fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.fLeft.setPower(speed);
        robot.fRight.setPower(speed);
        robot.bLeft.setPower(speed);
        robot.bRight.setPower(speed);

        robot.fLeft.setTargetPosition(targetPostition);
        robot.fRight.setTargetPosition(targetPostition);
        robot.bLeft.setTargetPosition(targetPostition);
        robot.bRight.setTargetPosition(targetPostition);


//        if(Math.abs(robot.fLeft.getCurrentPosition()) >= Math.abs(targetPostition)) {
//            return;
//        }
//
        while(robot.fLeft.isBusy() && robot.fRight.isBusy() && robot.bLeft.isBusy() && robot.bRight.isBusy()){
            sleep(10);
        }

        resetEncoders(robot);

}

    public void resumeEncoders(Kuro robot){
        robot.fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void changeRunModeToUsingEncoder(Kuro robot){
        robot.fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openClaws(Kuro robot){
        robot.upRight.setPosition(0.80);
        robot.upLeft.setPosition(0.15);
    }
}

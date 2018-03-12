package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="Red: Back", group="Linear Opmode")
//@Disabled
public class AriesRedBack extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Aries robot = new Aries(hardwareMap,telemetry,this);

        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.intializeIMU();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Pictograph pictograph = new Pictograph();

            RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 2000);
            telemetry.addLine(vuMark.toString());
            telemetry.update();

            robot.jewelArm();

            robot.moveRobotInches(0.4, 24);
            robot.moveRobotInches(0.5, -4);
            sleep(1000);
            robot.goToCryptoBox(0.1, 0.2);

            //Moves to right column
            robot.moveRobotInches(0.25, 3.25);

            //If not right, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobotInches(0.25, 6.75);
            }else if (vuMark == RelicRecoveryVuMark.LEFT) {
                robot.moveRobotInches(0.25, 6.75 * 2);
            }

            robot.finalTurn(90, 7500);
            robot.moveTray(78);
            sleep(1000);
            robot.moveRobot(0.7, -300);
            robot.moveTray(3);

            robot.moveRobotInches(1,36);
            robot.brakeMotors();
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
            sleep(2000);
            robot.finalTurn(45);
            robot.moveRobotInches(1,6);
            robot.brakeMotors();
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
            sleep(2000);
            robot.moveRobotInches(0.75,-6);
            robot.brakeMotors();
            robot.finalTurn(90);
            robot.moveRobotInches(0.75,-25);
            robot.brakeMotors();
            robot.finalTurn(75);
            robot.moveTray(78);
            robot.moveRobotInches(0.5,-10);
            robot.brakeMotors();
            robot.moveRobotInches(0.5,10);
            robot.brakeMotors();
            robot.moveTray(3);
            sleep(1000000);
        }
    }
}
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

@Autonomous(name="Blue: Back", group="Linear Opmode")
//@Disabled
public class AriesBlueBack extends LinearOpMode {

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

            robot.moveRobotInches(0.4, -28);
            robot.moveRobotInches(0.4, 8);
            sleep(1000);
            robot.goToCryptoBox(-0.1, 0.22);

            //Moves to left column
            robot.moveRobotInches(0.25, -2.5);

            //If not left, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobotInches(0.25, -6.75);
            }else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                robot.moveRobotInches(0.25, -6.75 * 2);
            }

            robot.finalTurn(90, 7500);
            robot.moveTray(78);
            robot.moveRobot(0.7,-300);
            robot.moveRobot(0.7,100);
            robot.moveTray(3);
            sleep(1000000);
        }
    }
}
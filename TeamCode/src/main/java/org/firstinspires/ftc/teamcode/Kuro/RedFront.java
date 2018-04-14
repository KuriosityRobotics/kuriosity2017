package org.firstinspires.ftc.teamcode.Kuro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Aries.Pictograph;


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

@Autonomous(name=" KURO    Red: Front", group="Linear Opmode")
//@Disabled
public class RedFront extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap,telemetry,this);


        robot.resetEncoders();
        //causing problem below

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Pictograph pictograph = new Pictograph();

            RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 2000);

            robot.closeClaws();
            robot.moveSlide(0.5, -850);

            robot.jewelArm();

            robot.moveRobot(0.3, -900);
            robot.moveRobot(0.25, -300);
            robot.moveRobotInches(0.4,9);
            robot.moveRobotInches(0.4,-6.75);
            robot.finalTurn(90);
            robot.moveRobotInches(0.4,6);
            sleep(1000);

            robot.goToCryptoBox(-0.25,0.55);

            robot.finalTurn(-90, 7500);

            if(vuMark == RelicRecoveryVuMark.RIGHT){
                robot.moveRobotInches(0.25, -4.5);

            }else if(vuMark == RelicRecoveryVuMark.LEFT){
                robot.moveRobotInches(0.25, 11.5);

            }else /*Center or unknown*/{
                //Defaults here
                robot.moveRobotInches(0.25, 4.5);

            }

            robot.finalTurn(-135);

            sleep(250);

            robot.moveSlide(0.4, 500);

            sleep(250);

            robot.moveRobotInches(0.45, 9);

            robot.openClaws();

            sleep(500);

            robot.moveRobotInches(0.5, -6);

            robot.finalTurn(-90, 2500);

            if(vuMark == RelicRecoveryVuMark.RIGHT){
                robot.moveRobotInches(0.3, 18);

            }else if(vuMark == RelicRecoveryVuMark.LEFT){

            }else /*Center or unknown*/{
                //Defaults here
                robot.moveRobotInches(0.3, 14.5);

            }

            sleep(1000000);
        }
    }
}
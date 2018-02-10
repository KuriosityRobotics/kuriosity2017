package org.firstinspires.ftc.teamcode.Kuro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Pictograph;


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
public class RedBack extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap,telemetry,this);

        robot.resetEncoders();
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

            robot.moveRobotInches(0.3, -28);
            robot.moveRobotInches(0.4, 8);
            sleep(1000);
            robot.goToCryptoBox(-0.2,0.65);




            //If not right, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobotInches(0.25, 2);
                robot.finalTurn(120);
                robot.resetEncoders();
                robot.moveSlide(0.4, 450);
                robot.moveRobot(0.25, 400);
                robot.openClaws();
                robot.moveRobot(0.25, -200);
            }else if (vuMark == RelicRecoveryVuMark.LEFT) {
                robot.moveRobotInches(0.25, -8);
                robot.finalTurn(120);
                robot.resetEncoders();
                robot.moveSlide(0.4, 450);
                robot.moveRobot(0.25, 400);
                robot.openClaws();
                robot.moveRobot(0.25, -600);
                robot.finalTurn(-90);
                robot.resetEncoders();
                robot.moveRobot(0.4, -600);
                robot.moveRobot(0.9, 300);
                robot.opBottomClaws();
                robot.moveRobotInches(0.25, 3);
                sleep(1000000);
            }else{
                robot.moveRobotInches(0.25, -8);
                robot.finalTurn(65);
                robot.resetEncoders();
                robot.moveSlide(0.4, 450);
                robot.moveRobot(0.25, 400);
                robot.openClaws();
                robot.moveRobot(0.25, -600);
                robot.finalTurn(-90);
                robot.resetEncoders();
                robot.moveRobot(0.4, -600);
                robot.moveRobot(0.9, 300);
                robot.opBottomClaws();
                robot.moveRobotInches(0.25, 3);
                sleep(1000000);
            }


        }
    }
}
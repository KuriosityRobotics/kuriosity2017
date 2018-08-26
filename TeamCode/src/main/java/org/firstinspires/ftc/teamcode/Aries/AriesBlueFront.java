package org.firstinspires.ftc.teamcode.Aries;

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

@Autonomous(name="Blue: Front", group="Linear Opmode")
//@Disabled
public class AriesBlueFront extends LinearOpMode {

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
        int currentTurn = 0;
        int distanceToLastCol = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Pictograph pictograph = new Pictograph();

            RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 2000);
            telemetry.addLine(vuMark.toString());
            telemetry.update();

            robot.jewelArm();

            robot.moveRobot(0.3, -900);
            robot.moveRobot(0.25, -300);
            robot.moveRobotInches(0.6, 9);
            robot.moveRobotInches(0.4, -4.75);



            //Moves to left column
            robot.resetEncoders();
            robot.meccanumWithWeirdReset(robot,-307,0.5);

            //If not left, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.meccanumWithWeirdReset(robot,-346,0.5);
                currentTurn = -1;

                distanceToLastCol = -346;
            }else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                robot.meccanumWithWeirdReset(robot,-346*2,0.5);
            }else {

                distanceToLastCol = -346*2;
            }


            robot.trayRight.setPosition(0.725);
            robot.trayLeft.setPosition(0.225);
            sleep(750);
            robot.moveRobot(0.7, -200);
            robot.moveRobot(0.7, 100);
            robot.bringDownTray();
            robot.meccanum(robot,distanceToLastCol,0.5);


            robot.finalTurn(10);
            robot.multiplyGlyphAuto(robot,2500,0.6);
            if(currentTurn!=0) {
                robot.finalTurn(currentTurn, 7500);
            }


            robot.finalTurn(0);

            robot.trayRight.setPosition(0.725);
            robot.trayLeft.setPosition(0.225);
            sleep(750);

            robot.moveRobot(0.7,-400);


            robot.moveRobot(0.7,300);
            robot.bringDownTray();

            robot.finalTurn(0);
            robot.meccanum(robot,500,1);



            //            robot.moveTray(3);
            sleep(1000000);
        }
    }
}
package org.firstinspires.ftc.teamcode.Aries;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@Autonomous(name="Red: Back Multiple", group="Linear Opmode")
//@Disabled
public class AriesRedBackMultiple extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        int moveAmount = 0;
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
            //tray up
            robot.trayRight.setPosition(0.725);
            robot.trayLeft.setPosition(0.275);

            sleep(1000);
            robot.moveRobot(0.7, -300);
            robot.moveRobot(0.7, 100);

            //tray down
            robot.trayRight.setPosition(0.12);
            robot.trayLeft.setPosition(0.88);

            sleep(1000000);
            while (robot.glyphBackSensor.getDistance(DistanceUnit.CM) > 0) {
                robot.leftIntake.setPower(1);
                robot.rightIntake.setPower(1);
                robot.moveRobotInches(1, 25);
                moveAmount = robot.fLeft.getCurrentPosition();
            }
            robot.moveRobotInches(moveAmount, 1);
            robot.trayRight.setPosition(0.725);
            robot.trayLeft.setPosition(0.275);
    }
    }
}

package org.firstinspires.ftc.teamcode;

/**
 * Created by sam on 12/5/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


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

@Autonomous(name="Red: Back", group="Linear Opmode")
//@Disabled
public class TestOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap,telemetry);
        KuroVuforiaPictograph pictograph = new KuroVuforiaPictograph();

        RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 3000);

        robot.resetEncoders();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.jewelArm();
            robot.closeClaws();
            robot.moveSlide(0.25,-600);
            robot.sleep(1000);
            telemetry.addData("Vuforia Mark ", vuMark);
            robot.moveRobotInches(0.4, -28);
            robot.moveRobotInches(0.4, 8);
            sleep(1000);
            robot.goToCryptoBox(-0.25,0.6);

            //Moves to right column
            robot.moveRobot(0.25, -145);

            //If not right, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobotInches(0.25, -robot.DISTANCE_BETWEEN_GLYPH_COLUMNS);
            }else if (vuMark == RelicRecoveryVuMark.LEFT) {
                robot.moveRobotInches(0.25, -robot.DISTANCE_BETWEEN_GLYPH_COLUMNS * 2);
            }

            robot.turn(90);
            robot.resetEncoders();
            robot.moveRobot(0.25,800);
            robot.setDrivePower(0.1);
            robot.sleep(2000);
            robot.openClaws();
            robot.sleep(2000);
            robot.moveRobot(0.25,-300);
            robot.moveRobot(0.4,300);
            robot.opBottomClaws();
            Thread.sleep(1000000);

        }
    }
}


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

@Autonomous(name="Blue: Back", group="Linear Opmode")
//@Disabled
public class BlueBack extends LinearOpMode {

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
            KuroVuforiaPictograph pictograph = new KuroVuforiaPictograph();

            RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 2000);
            robot.closeClaws();
            robot.moveSlide(0.5, -850);

            robot.jewelArm();

            robot.moveRobotInches(0.3, 28);
            robot.moveRobotInches(0.4, -8);
            sleep(1000);
            robot.goToCryptoBox(0.3, 0.7);
            sleep(1000);

            //Moves to right column
            robot.moveRobotInches(0.25, 2.25);

            //If not right, move to respective location
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobotInches(0.25, 6.5);
            }else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                robot.moveRobotInches(0.25, 6.5 * 2);
            }

            robot.finalTurn(90);
            robot.resetEncoders();
            robot.moveSlide(0.4, 450);
            robot.moveRobotInches(0.4,7);
            robot.openClaws();
            robot.moveRobotInches(0.3,-7);
            robot.opBottomClaws();
            robot.finalTurn(-90);
            robot.resetEncoders();
            robot.moveRobotInches(0.6, -6);
            robot.moveRobotInches(0.6, 3);
            sleep(1000000);

        }
    }
}
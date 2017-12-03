package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous(name="Test: Color Distance", group="Linear Opmode")
//@Disabled
public class TestColorDistanceSensor extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap, telemetry);

        robot.resetEncoders();

        //causing problem below


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean toDo = true;

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addLine("Color");
            telemetry.addData("Red", robot.cryptoBox.red());
            telemetry.addData("Green", robot.cryptoBox.green());
            telemetry.addData("Blue", robot.cryptoBox.blue());
            telemetry.addData("Alpha", robot.cryptoBox.alpha());
            telemetry.addLine();
            telemetry.addLine("Distance");
            telemetry.addData("MM", robot.distance.getDistance(DistanceUnit.MM));
            telemetry.addData("CM", robot.distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Inches", robot.distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Out of Range", robot.distance.distanceOutOfRange);
            telemetry.update();
        }
    }
}

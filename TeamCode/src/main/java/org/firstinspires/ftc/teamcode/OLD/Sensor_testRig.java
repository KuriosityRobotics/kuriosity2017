package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Rig: Sensors", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Sensor_testRig extends LinearOpMode{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    ColorSensor colorSensor;

    TouchSensor touchSensor;

    float left;
    float right;
    float max;

    float turnMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Kuro Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        colorSensor = hardwareMap.colorSensor.get("front_colorRange");

        touchSensor = hardwareMap.touchSensor.get("touch");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Turn Sensitivity", turnMultiplier);
            telemetry.addLine();
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.addData("Alpha", colorSensor.alpha());
            telemetry.addLine();
            telemetry.addData("Touch:", touchSensor.isPressed());
            telemetry.update();

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns forkliftLeft and forkliftRight.
            left  = -gamepad1.left_stick_y + turnMultiplier * (-gamepad1.left_stick_y * gamepad1.right_stick_x);
            right = -gamepad1.left_stick_y + turnMultiplier * (gamepad1.left_stick_y * gamepad1.right_stick_x);

            if(gamepad1.left_stick_y == 0){
                left = gamepad1.right_stick_x;
                right = - gamepad1.right_stick_x;
            }

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            if(gamepad1.left_bumper){
                turnMultiplier += 0.1;

            }else if(gamepad1.right_bumper){
                turnMultiplier -= 0.1;
            }

            leftMotor.setPower(left);
            rightMotor.setPower(right);

            Thread.sleep(250);
        }
    }
}

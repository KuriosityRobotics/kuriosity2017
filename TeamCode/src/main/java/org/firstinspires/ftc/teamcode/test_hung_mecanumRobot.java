/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Drives Stuff", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class mecanumRobot extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    Servo armServo = null;
    Servo armServo2 = null;

    BNO055IMU gyro = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor  = hardwareMap.dcMotor.get("left_drive");
        leftBack = hardwareMap.dcMotor.get("left_back_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        rightBack = hardwareMap.dcMotor.get("right_back_drive");
        armServo = hardwareMap.servo.get("arm_servo");
        armServo2 = hardwareMap.servo.get("arm_servo2");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        double maxRange = 0.8;
        double minRange = 0.1;
        double minRange2 = 0.1;
        double maxRange2 = 0.8;
        boolean alignMode = false;


        Float left;
        Float right;

        int mode = 0;




        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD
        leftBack.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE
        rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();



            if (gamepad1.dpad_left) {
                mode = 0;
            }else if (gamepad1.dpad_right) {
                mode = 1;
            }

            if (gamepad1.a) {
                alignMode = true;
            }else if (gamepad1.b) {
                alignMode = false;
            }

            switch (mode) {
                case 0:

                    double factor = 1.0;

                    if(((gamepad1.left_stick_y > 0.8) && (gamepad1.left_stick_y < - 0.8)) || ((gamepad1.right_stick_y > 0.8) && (gamepad1.right_stick_y < - 0.8)) ){
                        factor = 0.1;
                    }else{
                      factor = 1.0;
                    }

                    leftMotor.setPower(-gamepad1.left_stick_y * 2);
                    leftBack.setPower(gamepad1.left_stick_y * 2);
                    rightMotor.setPower(-gamepad1.right_stick_y * 2);
                    rightBack.setPower(-gamepad1.right_stick_y * 2 );

                    break;
                case 1:
                    left = -gamepad1.left_stick_y + 3 * (gamepad1.right_stick_y);
                    right = -gamepad1.left_stick_y - 3 * (gamepad1.right_stick_y);

                    float max = Math.max(left, right);

                    if (max > 1.00) {
                        left /= max;
                        right /= max;
                    }

                    leftMotor.setPower(left);
                    leftBack.setPower(-left);
                    rightMotor.setPower(right);
                    rightBack.setPower(right);

                    break;
            }

            if (!alignMode) {
                //Translate Left
                leftMotor.setPower(-gamepad1.left_trigger);
                leftBack.setPower(-gamepad1.left_trigger);
                rightMotor.setPower(gamepad1.left_trigger);
                rightBack.setPower(-gamepad1.left_trigger);

                //Translate Right
                rightMotor.setPower(-gamepad1.right_trigger);
                rightBack.setPower(gamepad1.right_trigger);
                leftMotor.setPower(gamepad1.right_trigger);
                leftBack.setPower(gamepad1.right_trigger);
            } else {
                //Translate Left
                leftMotor.setPower(-gamepad2.left_trigger);
                leftBack.setPower(-gamepad2.left_trigger);
                rightMotor.setPower(gamepad2.left_trigger);
                rightBack.setPower(-gamepad2.left_trigger);

                //Translate Right
                rightMotor.setPower(-gamepad2.right_trigger);
                rightBack.setPower(gamepad2.right_trigger);
                leftMotor.setPower(gamepad2.right_trigger);
                leftBack.setPower(gamepad2.right_trigger);
            }




            if(gamepad2.a){
                armServo.setPosition(minRange);
                armServo2.setPosition(minRange2);
            }
            if(gamepad2.b){
                armServo.setPosition(maxRange);
                armServo2.setPosition(maxRange2);
            }

        }
    }
}

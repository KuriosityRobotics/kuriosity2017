/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Meccanum Drive", group="Linear Opmode")
//@Disabled
public class MeccanumControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor fLeft;
    private DcMotor fRight;
    private DcMotor bLeft;
    private DcMotor bRight;
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
//    private Servo collectFront;
//    private Servo collectBack;
//    private Servo collect;


//    private Servo collectorHinge;
//    private Servo leftArm;
//    private Servo rihgtArm;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Map the Motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        intakeLeft = hardwareMap.dcMotor.get("lIntake");
        intakeRight = hardwareMap.dcMotor.get("rIntake");
//        collectBack = hardwareMap.servo.get("collectBack");
//        collectFront = hardwareMap.servo.get("collectFront");
//        collect = hardwareMap.servo.get("collect");

        //Set direction of motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        //Enable encoders
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double fLPower = 0.0;
        double fRPower = 0.0;
        double bLPower = 0.0;
        double bRPower = 0.0;
        double inputPower = 0.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {




            if(gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0){
                fLPower = -gamepad1.left_stick_y;
                bLPower = -gamepad1.left_stick_y;

                fRPower = -gamepad1.right_stick_y;
                bRPower = -gamepad1.right_stick_y;
            }else{
                fLPower = 0;
                fRPower = 0;
                bLPower = 0;
                bRPower = 0;
            }

            if(!(gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0)){
                if(gamepad1.left_trigger != 0){
                    fLPower = -gamepad1.left_trigger;
                    bLPower = gamepad1.left_trigger;
                    fRPower = gamepad1.left_trigger;
                    bRPower = -gamepad1.left_trigger;
                }else if(gamepad1.right_trigger != 0){
                    fLPower = gamepad1.right_trigger;
                    bLPower = -gamepad1.right_trigger;
                    fRPower = -gamepad1.right_trigger;
                    bRPower = gamepad1.right_trigger;
                }

            }
            inputPower = gamepad2.right_stick_y;

            fLeft.setPower(fLPower);
            fRight.setPower(fRPower);
            bLeft.setPower(bLPower);
            bRight.setPower(bRPower);
            intakeLeft.setPower(inputPower);
            intakeRight.setPower(inputPower);
        }
    }
}

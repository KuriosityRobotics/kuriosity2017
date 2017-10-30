package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Bid on 10/29/2017.
 */

public class Kuro {
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;
    public DcMotor intakeLeft;
    public DcMotor intakeRight;

    public DcMotor arm;
    public Servo leftClaw;
    public Servo rightClaw;
    public Servo leftGrabber;
    public Servo rightGrabber;
    
    public Kuro(HardwareMap hardwareMap){
        //Map motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        intakeLeft = hardwareMap.dcMotor.get("lIntake");
        intakeRight = hardwareMap.dcMotor.get("rIntake");
        arm = hardwareMap.dcMotor.get("arm");

        //Set direction of motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        //Enable encoders
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Map servos
        leftClaw = hardwareMap.servo.get("left_claw");
        rightClaw = hardwareMap.servo.get("right_claw");
        leftGrabber = hardwareMap.servo.get("left_grabber");
        rightGrabber = hardwareMap.servo.get("right_grabber");
    }
}

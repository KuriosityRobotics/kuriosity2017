package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public DcMotor right;
    public DcMotor left;
    //public DcMotor intakeLeft;
    //public DcMotor intakeRight;

    //public DcMotor arm;
    //public Servo leftClaw;
    //public Servo rightClaw;
    //public Servo leftGrabber;
    //public Servo rightGrabber;
    public Servo upLeft;
    public Servo upRight;
    public Servo downLeft;
    public Servo downRight;

    public Kuro(HardwareMap hardwareMap){
        //Map motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");
        downLeft = hardwareMap.servo.get("downL");
        downRight = hardwareMap.servo.get("downR");
        upLeft = hardwareMap.servo.get("upL");
        upRight = hardwareMap.servo.get("upR");
        //intakeLeft = hardwareMap.dcMotor.get("lIntake");
        //intakeRight = hardwareMap.dcMotor.get("rIntake");
        //arm = hardwareMap.dcMotor.get("arm");

        //Set direction of motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intakeRight.setDirection(DcMotor.Direction.FORWARD);
        //intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        //arm.setDirection(DcMotor.Direction.FORWARD);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Enable encoders
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Map servos
        //leftClaw = hardwareMap.servo.get("left_claw");
        //rightClaw = hardwareMap.servo.get("right_claw");
        //leftGrabber = hardwareMap.servo.get("left_grabber");
        //rightGrabber = hardwareMap.servo.get("right_grabber");
    }
}

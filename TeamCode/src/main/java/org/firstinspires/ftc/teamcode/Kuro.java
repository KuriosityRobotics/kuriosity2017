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
    public DcMotor forkliftRight;
    public DcMotor forkliftLeft;

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
        forkliftRight = hardwareMap.dcMotor.get("forkliftRight");
        forkliftLeft = hardwareMap.dcMotor.get("forkliftLeft");

        //Set direction of motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        forkliftLeft.setDirection(DcMotor.Direction.FORWARD);
        forkliftRight.setDirection(DcMotor.Direction.REVERSE);

        //Sets zero power behavior
        forkliftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forkliftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Enable encoders
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Map servo
        downLeft = hardwareMap.servo.get("downL");
        downRight = hardwareMap.servo.get("downR");
        upLeft = hardwareMap.servo.get("upL");
        upRight = hardwareMap.servo.get("upR");
    }
}

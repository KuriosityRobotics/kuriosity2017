package org.firstinspires.ftc.teamcode;

/**
 * Created by sam on 1/21/18.
 */

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Kuro on 10/29/2017.
 */

public class Aries {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    //Intake Motors
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    //Tray motors
    public DcMotor trayPivot;
    public DcMotor linearSlideMotor;

    //Jewel arm servos
    public Servo armServo;
    public Servo pivotServo;

    //imu
    public BNO055IMU imu;
    public Orientation angles;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    public Aries(HardwareMap hardwareMap, Telemetry telemetry,LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;


        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Set direction of drive motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);


        //Map intake motors
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        //Set direction of intake motors
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);



        //Map tray motors
        trayPivot = hardwareMap.dcMotor.get("trayPivot");
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");

        //Set direction of intake motors
        trayPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Sets mode of tray motors
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Map jewel arm servos
        armServo = hardwareMap.servo.get("armServo");
        pivotServo = hardwareMap.servo.get("pivotServo");
    }



    public void intializeIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }



    public void setMotorMode(DcMotor.RunMode runMode){
        fLeft.setMode(runMode);
        fRight.setMode(runMode);
        bLeft.setMode(runMode);
        bRight.setMode(runMode);
    }



    public void setDrivePower(double power){
        fLeft.setPower(power);
        fRight.setPower(power);
        bLeft.setPower(power);
        bRight.setPower(power);
    }



    public void finalTurn(double targetHeading){
        finalTurn(targetHeading, 10000);
    }

    public void finalTurn(double targetHeading,long timeInMilli){
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startHeading = angles.firstAngle;
        double maxAngle = startHeading - targetHeading;
        maxAngle = Math.abs(maxAngle);

        int sign = 0;
        if(targetHeading > startHeading){
            sign = 1;
        }else{
            sign = -1;
        }
        if(maxAngle == 0){
            return;
        }
        while(linearOpMode.opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double currentDeltatAngle = Math.abs(angles.firstAngle - startHeading);
            double scaleFactor = currentDeltatAngle / maxAngle;
            double absolutePower = 1-scaleFactor;

            if(absolutePower<0.02){
                absolutePower = 0.02;
            }
            double power = absolutePower * sign;
            if(scaleFactor > 1 || ((SystemClock.elapsedRealtime() - startTime) > timeInMilli)){
                break;
            }
            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(-power);
            bRight.setPower(power);
        }

        setDrivePower(0);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void moveRobotInches(double speed, double targetDistance){
        moveRobot(speed, (int)(targetDistance / 22.25 * 1000));
    }
    public void moveRobot(double speed, int targetPostition) {
        moveRobot(speed, targetPostition, 10000);
    }

    public void moveRobot(double speed, int targetPostition,long timeInMilli){

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        double newSpeed = speed;

        if(targetPostition<0){
            newSpeed = newSpeed * -1;
        }

        fLeft.setPower(newSpeed);
        fRight.setPower(newSpeed);
        bLeft.setPower(newSpeed);
        bRight.setPower(newSpeed);


        fLeft.setTargetPosition(targetPostition);
        fRight.setTargetPosition(targetPostition);
        bLeft.setTargetPosition(targetPostition);
        bRight.setTargetPosition(targetPostition);


        while(fLeft.isBusy() && fRight.isBusy() && bLeft.isBusy() && bRight.isBusy()
                && (SystemClock.elapsedRealtime() - startTime < timeInMilli) && linearOpMode.opModeIsActive()){
        }

        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

package org.firstinspires.ftc.teamcode.Aries;

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
    public DcMotor relicSlide;

    //Intake Motors
    public DcMotor leftIntake;
    public DcMotor rightIntake;

    //Tray motors
    public DcMotor linearSlideMotor;

    //Jewel arm servos
    public Servo armServo;
    public Servo pivotServo;
    public Servo trayRight;
    public Servo trayLeft;
    public Servo relicClaw;
    public Servo relicPivot;
    public Servo cryptoServo;
    public Servo leftTopStopper;
    public Servo rightTopStopper;

    //Jewel arm color sensors
    public ColorSensor ballColor;
    public ColorSensor stoneColor;

    //Cryptobox detector sensor
    public DistanceSensor distance;
    public DistanceSensor glyphDistance;

    //imu
    public BNO055IMU imu;
    public Orientation angles;
    public Position position;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    //Tray Sensors
    public DistanceSensor glyphFrontSensor;
    public DistanceSensor glyphBackSensor;

    public Aries(HardwareMap hardwareMap, Telemetry telemetry,LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;


        //Map drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        relicSlide = hardwareMap.dcMotor.get("relicSlide");

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
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");

        //Set direction of intake motors
//        trayPivot.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Sets mode of tray motors
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Map jewel arm servos
        armServo = hardwareMap.servo.get("armServo");
        pivotServo = hardwareMap.servo.get("pivotServo");
        trayLeft = hardwareMap.servo.get("trayLeft");
        trayRight = hardwareMap.servo.get("trayRight");
        relicClaw = hardwareMap.servo.get("relicClaw");
        relicPivot = hardwareMap.servo.get("relicPivot");
        leftTopStopper = hardwareMap.servo.get("leftTopStopper");
        rightTopStopper = hardwareMap.servo.get("rightTopStopper");


        //Map jewel arm sensors
        ballColor = hardwareMap.colorSensor.get("ballColor");
        stoneColor = hardwareMap.colorSensor.get("stoneColor");


        //Map cryptobox detector sensor
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        glyphDistance = hardwareMap.get(DistanceSensor.class, "glyphDistance");

        //Tray Sensors
        glyphBackSensor = hardwareMap.get(DistanceSensor.class, "glyphBackSensor");
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
    public void resetEncoders(){
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void resumeEncoders(){
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void changeRunModeToUsingEncoder(){
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        position = imu.getPosition();
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

            if(absolutePower< 0.01){
                absolutePower = 0.01;
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
        brakeMotors();
    }
    public void moveRobot(double speed, int targetPostition) {
        moveRobot(speed, targetPostition, 10000);
        brakeMotors();
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

        brakeMotors();

        telemetry.addLine("finished sleeping");
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public void meccanum(Aries robot, int targetPosition, double speed){
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fLeft.setPower(speed);
        robot.bLeft.setPower(speed);
        robot.fRight.setPower(speed);
        robot.bRight.setPower(speed);

        robot.fLeft.setTargetPosition(targetPosition);
        robot.bLeft.setTargetPosition(-1*targetPosition);
        robot.fRight.setTargetPosition(-1*targetPosition);
        robot.bRight.setTargetPosition(targetPosition);

        while(robot.fLeft.isBusy() &&robot.linearOpMode.opModeIsActive()){

        }
        robot.setDrivePower(0);

    }

    public void meccanumWithWeirdReset(Aries robot, int targetPosition, double speed){
        robot.fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.fLeft.setPower(speed);
        robot.bLeft.setPower(speed);
        robot.fRight.setPower(speed);
        robot.bRight.setPower(speed);

        robot.fLeft.setTargetPosition(targetPosition);
        robot.bLeft.setTargetPosition(-1*targetPosition);
        robot.fRight.setTargetPosition(-1*targetPosition);
        robot.bRight.setTargetPosition(targetPosition);
        sleep(500);
        while(robot.fLeft.isBusy() &&robot.linearOpMode.opModeIsActive()){

        }
        robot.setDrivePower(0);

    }









    public void goToCryptoBox(double power, double servoPosition){
        armServo.setPosition(servoPosition);

        this.changeRunModeToUsingEncoder();
        setDrivePower(power);

        while (!isCryptoBox() && linearOpMode.opModeIsActive()) {}

        brakeMotors();
        armServo.setPosition(0.65);
    }

    public void brakeMotors(){
        setDrivePower(0);
    }


    public void jewelArm() throws InterruptedException{
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        this.pivotServo.setPosition(0.45);
        sleep(100);
        this.armServo.setPosition(0);
        sleep(1000);
        if(getColor(this.ballColor).equals(getColor(this.stoneColor))){
            this.pivotServo.setPosition(0);
            sleep(1000);

        }else{
            this.pivotServo.setPosition(1);
            sleep(1000);
        }

        this.armServo.setPosition(0.65);
        sleep(2000);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void goToCryptobox(double power, double servoPosition){
        this.armServo.setPosition(servoPosition);
        sleep(500);
        //Start moving forward
        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDrivePower(power);

        //Wait until it sees cryptobox
        while (!isCryptoBox() && linearOpMode.opModeIsActive()){}

        //Stop robot
        this.setDrivePower(0);
        this.armServo.setPosition(0);
    }

    public boolean isCryptoBox(){ return !Double.isNaN(glyphBackSensor.getDistance(DistanceUnit.CM)); }

    public void autoNav(double speed, double x_start, double y_start, double x_end, double y_end){
        double distanceToTravel = 0;
        distanceToTravel = Math.sqrt(((x_start-x_end)*(x_start-x_end))+((y_start-y_end)*(y_start-y_end)));
        double oppositeSide = Math.abs(y_end-y_start);
        double lengthB = distanceToTravel;
        double adjacentSjde = Math.abs(x_end-x_start);
        double angle = 0;
        angle = Math.atan(oppositeSide/adjacentSjde);
        this.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("angle",angle);
        telemetry.addData("distance",distanceToTravel);
        telemetry.update();
        finalTurn(angle);

        fLeft.setPower(speed);
        fRight.setPower(speed);
        bLeft.setPower(speed);
        bRight.setPower(speed);

        moveRobotInches(speed,distanceToTravel*12);

    }

    public void movetoCrypto(){
        sleep(1000);
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setPower(-0.7);
        bLeft.setPower(0.7);
        fRight.setPower(0.7);
        bRight.setPower(-0.7);
        while(distance.getDistance(DistanceUnit.CM)>9) {

        }
        setDrivePower(0);
        cryptoServo.setPosition(0.5);
    }

    public void dumpTray(){
        trayRight.setPosition(0.725);
        trayLeft.setPosition(0.275);
    }

    public void multiplAutofinalTurn(double targetHeading,long timeInMilli){
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        this.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        position = imu.getPosition();
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

            if(absolutePower< 0.01){
                absolutePower = 0.01;
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

    public void bringDownTray(){
        trayRight.setPosition(0.12);
        trayLeft.setPosition(0.7);
    }

    public void multiplyGlyphAuto(Aries robot, int maxDistance,double powerBack){
        robot.rightTopStopper.setPosition(0.2);
        robot.leftTopStopper.setPosition(0.5);
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (Double.isNaN(robot.glyphDistance.getDistance(DistanceUnit.CM)) && robot.linearOpMode.opModeIsActive()) {
            robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if(robot.fLeft.getCurrentPosition()>maxDistance){
                break;
            }
            robot.leftIntake.setPower(-0.8);
            robot.rightIntake.setPower(-1);
                robot.fLeft.setPower(0.2);
                robot.bLeft.setPower(0.2);
                robot.fRight.setPower(0.7);
                robot.bRight.setPower(0.7);
                sleep(200);
                robot.fLeft.setPower(0.7);
                robot.bLeft.setPower(0.7);
                robot.fRight.setPower(0.2);
                robot.bRight.setPower(0.2);
                sleep(200);

        }

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftIntake.setPower(-1);
        robot.rightIntake.setPower(-1);
        sleep(250);
        robot.leftIntake.setPower(1);
        robot.rightIntake.setPower(1);
        sleep(250);
        robot.setDrivePower(powerBack);

        robot.fLeft.setTargetPosition(0);
        robot.bLeft.setTargetPosition(0);
        robot.fRight.setTargetPosition(0);
        robot.bRight.setTargetPosition(0);
        robot.leftIntake.setPower(-1);
        robot.rightIntake.setPower(-1);


        while(robot.fLeft.isBusy() &&robot.linearOpMode.opModeIsActive()){

        }
        robot.setDrivePower(0);
    }
}
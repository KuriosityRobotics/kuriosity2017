package org.firstinspires.ftc.teamcode.Kuro;

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

public class Kuro {
    //Drive Motors
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;

    //Glyph spool motors
    public DcMotor right;
    public DcMotor left;

    //Relic Claw control motors
    public DcMotor relicPivot;
    public DcMotor relicSlide;

    //Relic claw servos
    public Servo relicClawLeft;
    public Servo relicClawRight;

    //Glyph claw servos
    public Servo upLeft;
    public Servo upRight;
    public Servo downLeft;
    public Servo downRight;

    //Jewel Arm servos
    public Servo armServo;
    public Servo pivotServo;

    //Jewel Arm Color sensors
    public ColorSensor ballColor;
    public ColorSensor stoneColor;

    //Cryptobox detector sensors
    public DistanceSensor distance;
    public ColorSensor cryptoBox;

    //imu
    public BNO055IMU imu;
    public Orientation angles;

    //Inherited classes from Op Mode
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public LinearOpMode linearOpMode;

    public Kuro(HardwareMap hardwareMap, Telemetry telemetry,LinearOpMode linearOpMode){

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;

        intializeIMU();

        //Map motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        relicPivot = hardwareMap.dcMotor.get("relicPivot");
        relicSlide = hardwareMap.dcMotor.get("relicSlide");


        //Set direction of motors
        fLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.REVERSE);

        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        //Sets zero power behavior
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Enable encoders
        fLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Map servos
        downLeft = hardwareMap.servo.get("downL");
        downRight = hardwareMap.servo.get("downR");
        upLeft = hardwareMap.servo.get("upL");
        upRight = hardwareMap.servo.get("upR");
        armServo = hardwareMap.servo.get("armServo");
        pivotServo = hardwareMap.servo.get("pivotServo");

        relicClawLeft = hardwareMap.servo.get("relicClawLeft");
        relicClawRight = hardwareMap.servo.get("relicClawRight");

        //Map Sensors
        ballColor = hardwareMap.colorSensor.get("ballColor");
        stoneColor = hardwareMap.colorSensor.get("stoneColor");

        cryptoBox = hardwareMap.colorSensor.get("distance");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
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

    public void setDrivePower(double power){
        fLeft.setPower(power);
        fRight.setPower(power);
        bLeft.setPower(power);
        bRight.setPower(power);
    }

    public void goToCryptoBox(double power, double servoPosition){
        armServo.setPosition(servoPosition);
//        moveRobotInches(0.4,-2);
//        resumeEncoders();
        this.changeRunModeToUsingEncoder();
        setDrivePower(power);

        while (!isCryptoBox() && linearOpMode.opModeIsActive()) {}

        brakeMotors();
        armServo.setPosition(0);
    }

    public boolean isCryptoBox(){
        return !Double.isNaN(distance.getDistance(DistanceUnit.CM));
    }

    public void brakeMotors(){
        setDrivePower(0);
    }

    public void turn(double degrees){
        turn(degrees, 1500);
    }

    public void turn(double degrees, long timeInMilli){
        long startTime = SystemClock.elapsedRealtime();
        resetEncoders();

        changeRunModeToUsingEncoder();

        while (Math.abs(angles.firstAngle) <= Math.abs(degrees) && linearOpMode.opModeIsActive()){
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltatAngle = Math.abs(degrees) - Math.abs(angles.firstAngle);

            double power = 1 - (Math.abs(angles.firstAngle) / Math.abs(degrees));

            if(Math.abs(angles.firstAngle) >= Math.abs(degrees)){
                break;
            }

            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(-power);
            bRight.setPower(power);

            if (SystemClock.elapsedRealtime()> startTime + timeInMilli){
                break;
            }
        }

        brakeMotors();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        resumeEncoders();
    }

    public void finalTurn(double targetHeading){
        finalTurn(targetHeading, 10000);
    }

    public void finalTurn(double targetHeading,long timeInMilli){
        targetHeading = Range.clip(targetHeading, -179, 179);

        long startTime = SystemClock.elapsedRealtime();

        resetEncoders();
        changeRunModeToUsingEncoder();

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
        brakeMotors();
        resumeEncoders();
    }

    public void resetEncoders(){
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void closeClaws(){
        upRight.setPosition(0.02);
        upLeft.setPosition(0.9);
        sleep(500);
    }

    public void moveSlide(double power, int targetPosition){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        right.setPower(0);
        left.setPower(0);

        right.setTargetPosition(targetPosition);
        left.setTargetPosition(targetPosition);

        left.setPower(-power);
        right.setPower(-power);

        while(left.isBusy() && right.isBusy() && linearOpMode.opModeIsActive()){}
    }

    public void moveRobotInches(double speed, double targetDistance){
        moveRobot(speed, (int)(targetDistance / 22.25 * 1000));
    }
    public void moveRobot(double speed, int targetPostition) {
        moveRobot(speed, targetPostition, 10000);
    }

    /**
     *
     * @param speed always positive
     * @param targetPostition backwards = negative foward = positive
     */
    public void moveRobot(double speed, int targetPostition,long timeInMilli){

        long startTime = SystemClock.elapsedRealtime();

        resetEncoders();
        fLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        resetEncoders();
        resumeEncoders();
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

    public void openClaws(){
        upRight.setPosition(0.80);
        upLeft.setPosition(0.15);
    }

    public void opBottomClaws(){
        downRight.setPosition(0);
        downLeft.setPosition(1);
    }

    public void jewelArm() throws InterruptedException{
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        this.pivotServo.setPosition(0.45);
        sleep(100);
        this.armServo.setPosition(0.95);
        sleep(1000);
        if(getColor(this.ballColor).equals(getColor(this.stoneColor))){
            this.pivotServo.setPosition(0);
            sleep(1000);

        }else{
            this.pivotServo.setPosition(1);
            sleep(1000);
        }

        this.armServo.setPosition(0.15);
        sleep(2000);
    }

    public String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void grabRelic(){
        relicPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relicClawRight.setPosition(0);
        relicClawLeft.setPosition(1);
        sleep(1000);

        relicSlide.setPower(1);
        relicSlide.setTargetPosition(-200);
        while(relicSlide.isBusy() && linearOpMode.opModeIsActive()){

        }

        relicPivot.setPower(1);
        relicPivot.setTargetPosition(-80);
        relicPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicSlide.setPower(1);
        relicSlide.setTargetPosition(-10);
        while((relicPivot.isBusy() || relicSlide.isBusy()) && linearOpMode.opModeIsActive()){
            telemetry.addData("Relic Pivot Position", relicPivot.getCurrentPosition());
            telemetry.update();

        }
        relicSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void releaseRelic(){
        relicPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relicSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relicSlide.setPower(1);
        relicSlide.setTargetPosition(-1235);
        while(relicSlide.isBusy() && linearOpMode.opModeIsActive()){

        }

        relicPivot.setPower(1);
        relicPivot.setTargetPosition(60);
        while(relicPivot.isBusy() && linearOpMode.opModeIsActive()){

        }
        sleep(1000);
        relicClawRight.setPosition(1);
        relicClawLeft.setPosition(0);
        sleep(500);

        relicSlide.setTargetPosition(-800);
        while(relicSlide.isBusy() && linearOpMode.opModeIsActive()){

        }

        relicClawRight.setPosition(0);
        relicClawLeft.setPosition(1);
        relicPivot.setTargetPosition(-10);
        relicPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicSlide.setPower(1);
        relicSlide.setTargetPosition(-10);
        while((relicSlide.isBusy() || relicPivot.isBusy()) && linearOpMode.opModeIsActive()){

        }

        relicSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
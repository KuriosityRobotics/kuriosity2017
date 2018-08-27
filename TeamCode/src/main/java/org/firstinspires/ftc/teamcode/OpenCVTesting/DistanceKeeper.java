package org.firstinspires.ftc.teamcode.OpenCVTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.CameraBridgeViewBase.*;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;

import java.util.*;

@TeleOp(name="Distance Keeper", group="Linear Opmode")
public class DistanceKeeper extends LinearOpMode {
    public DcMotor fLeft;
    public DcMotor fRight;
    public DcMotor bLeft;
    public DcMotor bRight;
    public BNO055IMU imu;
    public Orientation angles;

    static int size = 100;
    static int dist = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        intializeIMU();

        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            telemetry.addData("Dist", dist);
        }
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

    static class SquareDetect extends OpenCVPipeline{
        @Override
        public Mat processFrame(Mat rgba, Mat gray) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(rgba,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

            Iterator<MatOfPoint> iterator = contours.iterator();
            while (iterator.hasNext()){
                MatOfPoint contour = iterator.next();

                double epsilon = 0.1*Imgproc.arcLength(new MatOfPoint2f(contour.toArray()),true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()),approx,epsilon,true);
                if(approx.elemSize()==4){
                    Rect bound = Imgproc.boundingRect(new MatOfPoint(approx.toArray()));
                    if(bound.height==bound.width){
                        dist = bound.height-size;
                        break;
                    }
                }
            }

            return rgba;
        }
    }
}

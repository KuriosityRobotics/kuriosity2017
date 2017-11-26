package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "JewelArm")
public class JewelArm extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Kuro robot = new Kuro(hardwareMap,telemetry);


        telemetry.addData("Alpha", robot.ballColor.alpha());
        telemetry.addData("Red  ", robot.ballColor.red());
        telemetry.addData("Green", robot.ballColor.green());
        telemetry.addData("Blue ", robot.ballColor.blue());
        telemetry.update();

        waitForStart();
        // Lower Arm between jewels

        // If our team is BLUE, run this

        // If our team is RED, run this command
        // runArmMissionWithColorParameter(false);

        jewelArm(hardwareMap,telemetry);
    }

    public static String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public static void jewelArm(HardwareMap hardwareMap, Telemetry telemetry) {
        Kuro robot = new Kuro(hardwareMap,telemetry);
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        robot.pivotServo.setPosition(0.45);
        kuroSleep(100);
        robot.armServo.setPosition(1);
        kuroSleep(1000);
        if(getColor(robot.ballColor).equals(getColor(robot.stoneColor))){
            robot.pivotServo.setPosition(0);
            kuroSleep(1000);

        }else{
            robot.pivotServo.setPosition(1);
            kuroSleep(1000);

        }
        robot.armServo.setPosition(0.15);
        kuroSleep(2000);
    }
    public static void kuroSleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
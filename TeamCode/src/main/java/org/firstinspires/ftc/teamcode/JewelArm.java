package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "JewelArm")
public class JewelArm extends LinearOpMode {
    Servo DownMotor;
    Servo PivotMotor;
    ColorSensor ballColor;
    ColorSensor stoneColor;

    public void runOpMode() throws InterruptedException {
        DownMotor = hardwareMap.servo.get("downMotor");
        PivotMotor = hardwareMap.servo.get("pivotMotor");
        ballColor = hardwareMap.colorSensor.get("ballColor");
        stoneColor = hardwareMap.colorSensor.get("stoneColor");
        telemetry.addData("Alpha", ballColor.alpha());
        telemetry.addData("Red  ", ballColor.red());
        telemetry.addData("Green", ballColor.green());
        telemetry.addData("Blue ", ballColor.blue());
        telemetry.update();

        waitForStart();
        // Lower Arm between jewels

        // If our team is BLUE, run this

        // If our team is RED, run this command
        // runArmMissionWithColorParameter(false);

        sleep(1000);
        jewelArm();
    }

    public String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public void jewelArm() {
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        PivotMotor.setPosition(0.5);
        sleep(1000);
        DownMotor.setPosition(0.675);
        sleep(500);
        if(getColor(ballColor).equals(getColor(stoneColor))){
            PivotMotor.setPosition(0);
            sleep(1000);

        }else{
            PivotMotor.setPosition(1);
            sleep(1000);

        }
            DownMotor.setPosition(0.1);
            sleep(5000);

    }
}
package org.firstinspires.ftc.teamcode.OLD;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "KhuesJewelArm2")
public class KhuesJewelArm2 extends LinearOpMode {
    Servo DownMotor;
    Servo PivotMotor;
    ColorSensor sensorColor;

    public void runOpMode() throws InterruptedException {
        DownMotor = hardwareMap.servo.get("DownMotor");
        PivotMotor = hardwareMap.servo.get("PivotMotor");
        sensorColor = hardwareMap.colorSensor.get("sensorColor");

        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.update();

        waitForStart();

        // Lower Arm between jewels
        PivotMotor.setPosition(0.5);
        sleep(1000);
        DownMotor.setPosition(1);
        sleep(500);

        // If our team is BLUE, run this
        runArmMissionWithColorParameter(true);
        // If our team is RED, run this command
        // runArmMissionWithColorParameter(false);

        sleep(1000);
    }


    public void runArmMissionWithColorParameter(boolean goodColorIsBlue) {
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        if ( (goodColorIsBlue && (sensorColor.blue() > sensorColor.red()))
                || (!goodColorIsBlue && (sensorColor.red() > sensorColor.blue())) ) {
            PivotMotor.setPosition(1);
            sleep(1000);
            DownMotor.setPosition(0.5);
            sleep(1000);
            PivotMotor.setPosition(0.5);
            sleep(1000);
        }
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        else if ( (goodColorIsBlue && sensorColor.red() > sensorColor.blue())
                || (!goodColorIsBlue && sensorColor.blue() > sensorColor.red()) ) {
            PivotMotor.setPosition(0);
            sleep(1000);
            DownMotor.setPosition(0.5);
            sleep(1000);
            PivotMotor.setPosition(0.5);
            sleep(1000);
        } else if (sensorColor.red() == sensorColor.blue()){
            DownMotor.setPosition(0.5);
            sleep(1000);
            PivotMotor.setPosition(0.5);
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "JewelArm")
public class JewelArm extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Kuro robot = new Kuro(hardwareMap);


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

        sleep(1000);
        jewelArm(hardwareMap);
    }

    public String getColor(ColorSensor colorSensor){
        if(colorSensor.blue() > colorSensor.red()){
            return "red";
        }else{
            return "blue";
        }
    }

    public void jewelArm(HardwareMap hardwareMap) {
        Kuro robot = new Kuro(hardwareMap);
        //Checks which color ball is then moves the arm to knock of jewel that is matching opposing team color
        robot.pivotServo.setPosition(0.45);
        sleep(100);
        robot.armServo.setPosition(0.675);
        sleep(500);
        if(getColor(robot.ballColor).equals(getColor(robot.stoneColor))){
            robot.pivotServo.setPosition(0);
            sleep(1000);

        }else{
            robot.pivotServo.setPosition(1);
            sleep(1000);

        }
        robot.armServo.setPosition(0.1);
        sleep(5000);

    }
}
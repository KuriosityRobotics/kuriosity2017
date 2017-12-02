package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * Created by Khue on 11/26/17.
 */
@Autonomous(name="Blue Black", group="Linear Opmode")
public class BlueBack extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Kuro robot = new Kuro(hardwareMap, telemetry);
        KuroVuforiaPictograph pictograph = new KuroVuforiaPictograph();

        RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 3000);

        robot.resetEncoders();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            JewelArm.jewelArm(hardwareMap, telemetry);
            robot.closeClaws();
            robot.sleep(1000);
            robot.moveSlide(0.25,-600);
            telemetry.addData("Vuforia Mark ", vuMark);
            if(vuMark == RelicRecoveryVuMark.CENTER){
                robot.moveRobot(0.5,1350);
                robot.moveRobot(0.25,200);
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){
                robot.moveRobot(0.5,1600);
                robot.moveRobot(0.25,200);
            }else{
                robot.moveRobot(0.5,1000);
                robot.moveRobot(0.25,200);
            }
            robot.turn(90);
            robot.resetEncoders();
            robot.moveRobot(0.25,800);
            robot.setDrivePower(0.1);
            robot.sleep(2000);
            robot.openClaws();
            robot.sleep(2000);
            robot.moveRobot(0.25,-300);
            robot.moveRobot(0.4,300);
            robot.opBottomClaws();
            Thread.sleep(1000000);
        }
    }

}

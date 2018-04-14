package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.GlyphOpMode;

import org.firstinspires.ftc.teamcode.Aries.Aries2;

@TeleOp(name="DogeCV Glyph Detector", group="DogeCV")

public class GlyphOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Aries2 robot;


    public GlyphDetector glyphDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot = new Aries2(hardwareMap,telemetry);


        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();

        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();


    }

    @Override
    public void loop() {
        if(glyphDetector.getChosenGlyphPosition().x<50 || glyphDetector.getChosenGlyphPosition().x>150){
            robot.fLeft.setPower(0.4);
            robot.bLeft.setPower(0.4);
            robot.fRight.setPower(-0.4);
            robot.bRight.setPower(-0.4);
        }else {
            robot.setDrivePower((1 / (glyphDetector.getChosenGlyphPosition().x / 17)));
            robot.leftIntake.setPower(-1);
            robot.rightIntake.setPower(-1);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Glyph Pos X", glyphDetector.getChosenGlyphOffset());
        telemetry.addData("Glyph Pos Offest", glyphDetector.getChosenGlyphPosition().toString());
        telemetry.addData("FLEFT POWER", robot.fLeft.getPower());
        telemetry.update();


    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        glyphDetector.disable();
    }

}

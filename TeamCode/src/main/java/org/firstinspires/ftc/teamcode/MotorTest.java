package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Aries: MotorTest", group="Linear Opmode")
//@Disabled
public class MotorTest extends LinearOpMode
{
    double fLPower;
    double fRPower;
    double bLPower;
    double bRPower;

    long startTime = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        //Init's robot
        Aries robot = new Aries(hardwareMap, telemetry, this);   //DO NOT DELETE

        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor[] arrayOfMotors = {robot.fRight, robot.bRight, robot.fLeft, robot.bLeft};

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            for(DcMotor motor : arrayOfMotors){
                for (DcMotor toBrake : arrayOfMotors){
                    toBrake.setPower(0);
                }

                motor.setPower(0.1);
                telemetry.addLine(motor.getConnectionInfo());
                telemetry.addLine(motor.getPortNumber() + "");
                telemetry.update();

                sleep(2500);
            }
        }
    }
}

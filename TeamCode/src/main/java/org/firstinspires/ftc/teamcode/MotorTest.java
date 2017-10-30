package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="Motor Test IT.", group="Iterative Opmode")
//@Disabled
public class MotorTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    Kuro robot = new Kuro(hardwareMap);



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }



    @Override
    public void init_loop() {
        runtime.reset();
    }



    @Override
    public void start() {
        runtime.reset();
    }

    DcMotor[] arrayOfMotor = {robot.fLeft, robot.fRight, robot.bLeft, robot.bRight};
    int currentMotorIndex = 0;
    long startedTime = 0;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        startedTime++;
        if (runtime.time() > 100.0) {
            startedTime = 0;
            for(DcMotor toBrake : arrayOfMotor){
                toBrake.setPower(0);
            }
        }
        currentMotorIndex = (currentMotorIndex + 1) % arrayOfMotor.length;
        DcMotor currentMotor = arrayOfMotor[currentMotorIndex];
        currentMotor.setPower(1);




//        DcMotor[] arrayOfMotor = {frontLeftDrive,fRight,bLeft,bRight};
//
//        for(DcMotor motor : arrayOfMotor){
//            //Brake all
//            for(DcMotor toBrake : arrayOfMotor){
//                toBrake.setPower(0);
//            }
//            if (stopRequested) {
//                break;
//            }
//
//            //Tell what motor should e running
//            telemetry.addData("Motor", motor.getPortNumber());
//            motor.setPower(1);
//
//
//            telemetry.update();
//            delay(1000);
//        }
    }



    @Override
    public void stop() {
        for(DcMotor toBrake : arrayOfMotor){
            toBrake.setPower(0);
        }
    }

}

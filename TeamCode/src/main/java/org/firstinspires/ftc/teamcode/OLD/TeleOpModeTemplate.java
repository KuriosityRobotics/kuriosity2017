package org.firstinspires.ftc.teamcode.OLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Kuro;


@TeleOp(name="NAME HERE", group="Iterative Opmode")
@Disabled
public class TeleOpModeTemplate extends OpMode
{
    ElapsedTime runtime = new ElapsedTime();

    //This runs the init of KuriosityOPMode
    Kuro robot = new Kuro(hardwareMap);



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
    }



    @Override
    public void init_loop() {
    }



    @Override
    public void start() {
        runtime.reset();
    }



    @Override
    public void loop() {
        //Code here

        // Elapsed Time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }



    @Override
    public void stop() {
    }
}

    package org.firstinspires.ftc.teamcode.Aries;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


    /**
     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
     * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
     * class is instantiated on the Robot Controller and executed.
     *
     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
     * It includes all the skeletal structure that all linear OpModes contain.
     *
     * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     */

    @Autonomous(name="Blue: Back", group="Linear Opmode")
    //@Disabled
    public class AriesBlueBack extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() throws InterruptedException{
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            Aries robot = new Aries(hardwareMap,telemetry,this);

            robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();
            robot.intializeIMU();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {



                Pictograph pictograph = new Pictograph();

                RelicRecoveryVuMark vuMark = pictograph.startInit(hardwareMap, 2000);
                telemetry.addLine(vuMark.toString());
                telemetry.update();

                robot.jewelArm();
                if(vuMark == RelicRecoveryVuMark.CENTER) {
                    robot.moveRobotInches(0.4, -31);
                }else if(vuMark == RelicRecoveryVuMark.LEFT){
                    robot.moveRobot(0.4,-24);
                }else{
                    robot.moveRobot(0.4,-38);
                }



                robot.finalTurn(90, 7500);
                robot.moveRobot(0.5,-200);
                robot.cryptoServo.setPosition(1);
                sleep(500);
                robot.movetoCrypto();
                robot.trayRight.setPosition(0.725);
                robot.trayLeft.setPosition(0.275);
                sleep(1500);
                robot.moveRobot(0.5,200);
                robot.bringDownTray();

                robot.multiplyGlyphAuto(robot,2500);
                robot.finalTurn(110,7500);

                sleep(1000);

                robot.trayRight.setPosition(0.725);
                robot.trayLeft.setPosition(0.275);
                sleep(1500);

                robot.moveRobot(0.7,-300);


                robot.moveRobot(0.7,-300);
                robot.bringDownTray();

                robot.finalTurn(90);
                robot.meccanum(robot,300,0.7);



    //            robot.moveTray(3);
                sleep(1000000);
            }
        }
    }
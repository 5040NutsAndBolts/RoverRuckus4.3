package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="PiD Test", group="Auto")
@Disabled
public class PIDTest extends AutoMethods {

    private PID pid;
    private Hardware robot;
    private MecanumDrive driveTrain;

    @Override
    public void runOpMode() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        robot.init(hardwareMap,true);

        final double kp = 0.00099; //These are the gain values, set them to final when they are good
        long kd = Long.MAX_VALUE;

        double mod = 0.1;

        while(!isStarted()) {
            pid = new PID(kp, 0 , kd);
            telemetry.addLine("PID Class Created");
            telemetry.update();
        }

        driveTrain.forwardInch(5);

        boolean toggle = true;
        while(opModeIsActive()) {
            telemetry.addLine("----------------------------");
            telemetry.addData("Proportional:", kp);
            telemetry.addData("Integral:    ", 0);
            telemetry.addData("Derivative:  ", kd);
            telemetry.addLine("-----------------------------");
            telemetry.addData("Modifier:    ", mod);


            // Move the robot forward or backwards
            if(gamepad1.a) {
                driveTrain.forwardInch(48);
            }

            if(gamepad1.b) {
                driveTrain.forwardInch(-48);
            }

            if(toggle) {

                // Increase the derivative gain by the modifier
                if (gamepad1.right_trigger > 0.1) {
                    kd += mod;
                    pid.setKd(kd);
                    toggle = false;
                }

                // Decrease the derivative gain by the modifier
                if(gamepad1.left_trigger > 0.1) {
                    kd -= mod;
                    pid.setKd(kd);
                    toggle = false;
                }

                // Increase the modifier
                if(gamepad1.left_bumper) {
                    mod = mod / 10;
                    toggle = false;
                }

                // Decrease the modifier
                if(gamepad1.right_bumper) {
                    mod = mod * 10;
                    toggle = false;
                }

            } else if(!gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.left_trigger < 0.1 && gamepad1.right_trigger < 0.1){
                toggle = true;
            }


            telemetry.addData("Error:       ", driveTrain.forwardPID());
            telemetry.update();
        }
    }

}

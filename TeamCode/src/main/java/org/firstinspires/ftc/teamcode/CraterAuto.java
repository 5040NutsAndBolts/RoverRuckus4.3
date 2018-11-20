package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoCrater", group="Auto")
public class CraterAuto extends AutoMethods {

    private Hardware robot;
    private MecanumDrive driveTrain;
    private LiftMechanism lifter;
    private ElapsedTime wait;
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        lifter = new LiftMechanism(robot);
        wait = new ElapsedTime();

        double power;
        int goldPos = 3;

        robot.init(hardwareMap);

        robot.teamMarker.setPosition(0);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        dogeCVSetup(detector);

        telemetry.addLine("ready");
        telemetry.update();

        while(!isStarted()) {
            telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
            telemetry.update();
        }
        detector.enable();
        wait.reset();
        robot.wrist.setPower(0.5);
        robot.wrist.setTargetPosition(0);
        robot.collectionSlide.setPower(0.3);
        robot.collectionSlide.setTargetPosition(0);

        //lowers hang mechanism
        lifter.lift(true,false);
        while(opModeIsActive() && robot.hangingMotor.getCurrentPosition() < 6800) {
            //scans for gold mineral spot
            if(robot.hangingMotor.getCurrentPosition() < 200) {
                if (detector.getXPosition() > 200) {
                    goldPos = 2;
                } else if (detector.getXPosition() > 10) {
                    goldPos = 1;
                }
            }

            telemetry.addData("detector aligned", detector.getAligned());
            telemetry.addData("wait seconds",wait.seconds());
            telemetry.addData("goldPos",goldPos);
            telemetry.addData("x pos of gold", detector.getXPosition());
            telemetry.addData("hanging slide power", robot.hangingMotor.getPower());
            telemetry.addData("hanging motor pos",robot.hangingMotor.getCurrentPosition());
            telemetry.update();
        }
        detector.disable();

        //moves hook off the lander
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-5);
        while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }
        robot.hangingMotor.setTargetPosition(0);
        robot.hangingMotor.setPower(1);

        //moves away from hook
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.sidewaysInch(5);
        while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }


        //GOLD ON THE LEFT
        if(goldPos == 1) {
            driveTrain.rotate(25);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(20);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            //drive into wall
            driveTrain.rotate(25);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-5);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(28);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

        }
        //GOLD IN THE MIDDLE
        else if(goldPos == 2) {
            driveTrain.rotate(-26);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(17);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //drive into wall
            driveTrain.rotate(70);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-23);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(35);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
        //GOLD ON THE RIGHT
        else {
            driveTrain.rotate(-55);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(22);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //drive into wall
            driveTrain.rotate(90);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-29);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(45);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

        }
        robot.hangingMotor.setPower(0);

        power = 0;
        driveTrain.powerSet(power);
        driveTrain.sidewaysInch(-4);
        while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
            telemetry.addData("goldPos", goldPos);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }

        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-56);
        while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
            telemetry.update();
            if(robot.rightDriveFront.getCurrentPosition() > robot.rightDriveFront.getTargetPosition()+1000) {
                if(power <= 1)
                    power += 0.05;
            }
            else {
                if(power >= 0.4)
                    power -= 0.05;

            }
            driveTrain.powerSet(power);
        }

        robot.teamMarker.setPosition(.27);

        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(75);
        while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }

        while(opModeIsActive()){

        }

    }
}

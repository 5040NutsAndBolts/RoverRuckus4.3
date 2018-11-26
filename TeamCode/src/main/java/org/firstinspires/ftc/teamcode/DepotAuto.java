package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDepot", group="Auto")
public class DepotAuto extends AutoMethods {

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

       int goldPos = landing(robot,detector, driveTrain);


        //GOLD ON THE LEFT
        if(goldPos == 1) {

            runToRotateWait(25, robot, driveTrain);

            //knocks off sample
            runToSidewaysWait(25, robot, driveTrain);

            runToRotateWait(-165, robot, driveTrain);

            runToSidewaysWait(-15, robot, driveTrain);

            runToForwardWait(-30, robot, driveTrain);

            robot.teamMarker.setPosition(.27);
        }
        //GOLD IN THE MIDDLE
        else if(goldPos == 2) {
            driveTrain.rotate(-25);
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
            driveTrain.sidewaysInch(20);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            driveTrain.rotate(-80);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-28);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            robot.teamMarker.setPosition(.27);

            driveTrain.rotate(-63);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+5 && opModeIsActive()) {
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
            driveTrain.sidewaysInch(25);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            driveTrain.rotate(-22);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-32);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            robot.teamMarker.setPosition(.27);

            driveTrain.rotate(-90);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+5 && opModeIsActive()) {
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
        driveTrain.forwardInch(70);
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

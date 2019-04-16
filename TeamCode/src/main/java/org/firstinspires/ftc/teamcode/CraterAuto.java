package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Auto for the depot landing spot
 */
@Autonomous(name="Crater Auto", group="Auto")
public class CraterAuto extends AutoMethods {

    //class objects
    private Hardware robot;
    private MecanumDrive driveTrain;
    private Collection collection;
    private GoldAlignDetector detector;
    private String driverSpot = "Crater";

    private Thread t1 = new Thread(){
        public void run() {
            while(!t1.isInterrupted()) {
                robot.wristRight.setPosition(collection.wristPos);
            }
        }
    };

    private Thread t2 = new Thread(){
        public void run() {
            while(!t2.isInterrupted()) {
                robot.wristLeft.setPosition(collection.wristPos);
            }
        }
    };


    /**
     * The method that gets run when you hit init
     */
    @Override
    public void runOpMode() {
        ElapsedTime time = new ElapsedTime();
        //sets up the objects for the other classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        collection = new Collection(robot);

        //initializes the robot hardware and sets powers so things don't move
        robot.init(hardwareMap,true);

        robot.collectionSlide.setPower(0.3);
        robot.collectionSlide.setTargetPosition(0);

        robot.hang.setTargetPosition(0);
        robot.hang.setPower(1);

        robot.scoringSlide.setTargetPosition(0);
        robot.scoringSlide.setPower(0);

        robot.scoringStop.setPosition(0);
        robot.intakeStop.setPosition(0.4);

        t1.start();
        t2.start();


        //sets up the detector for mineral detection
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        dogeCVSetup(detector);


        boolean spotToggle = false;
        //waits here til you hit start or stop
        while(!isStarted()&&!isStopRequested()) {
            if(gamepad1.x && !spotToggle){
                spotToggle = true;
                if(driverSpot.equals("Crater"))
                    driverSpot = "Depot";
                else
                    driverSpot = "Crater";
            }
            else if(!gamepad1.x && spotToggle)
                spotToggle = false;
            telemetry.addData("Driver spot",driverSpot);
            telemetry.update();
        }
        //enables the detector until it scans the minerals
        detector.enable();

        //lands the robot and returns what position the gold mineral is in
        int goldPos = landing(-90,robot,detector, driveTrain);

        double power;

        robot.hang.setTargetPosition(0);
        time.reset();
        while(time.seconds() < 0.2 && opModeIsActive()){}

        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(13);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<700) {
                if (power > 0.4)
                    power -= 0.05;
                else
                    power += 0.05;
            }
            else {
                if (power < 0.7)
                    power += 0.05;
            }
            driveTrain.powerSet(power);

            //runs slide out for placing the TM
            if(goldPos == 2) {
                collection.wristPos = collection.wristDownPos;
                robot.collectionSlide.setPower(0.5);
                robot.collectionSlide.setTargetPosition(170);
            }

        }
        collection.wristPos = collection.wristDownPos;

        if(goldPos == 1) {
            runToRotateWait(-45,robot,driveTrain);
        }
        else if(goldPos == 3){
            runToRotateWait(45,robot,driveTrain);
        }
        if(goldPos != 2) {
            robot.collectionSlide.setTargetPosition(300);
            robot.collectionSlide.setPower(0.6);
        }

        while (Math.abs(robot.collectionSlide.getTargetPosition() - robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos", robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }

        robot.collectionSlide.setTargetPosition(0);
        collection.wristPos = collection.wristUpPos;

        if(goldPos == 2) {
            driveTrain.resetMotors();
        }
        power = 0;
        driveTrain.powerSet(power);
        int rDegrees = -88*9;
        robot.leftDriveFront.setTargetPosition(rDegrees);
        robot.leftDriveRear.setTargetPosition(rDegrees);
        robot.rightDriveFront.setTargetPosition(-rDegrees);
        robot.rightDriveRear.setTargetPosition(-rDegrees);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power+=0.1;
            driveTrain.powerSet(power);
        }

        runToForwardWait(50,robot,driveTrain);

        collection.wristPos = collection.wristDownPos;
        runToRotateWait(-47,robot,driveTrain);

        //collection.wristPos = collection.wristDownPos;
        robot.collectionSlide.setTargetPosition(750);
        while (Math.abs(robot.collectionSlide.getTargetPosition() - robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos", robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }
        robot.intakeStop.setPosition(0.02);

        robot.collectionSlide.setTargetPosition(0);
        while (Math.abs(robot.collectionSlide.getTargetPosition() - robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos", robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }
        collection.wristPos = 0.4;

        runToRotateWait(-190,robot,driveTrain);
        runToForwardWait(25,robot,driveTrain);
        collection.wristPos = 0.4;

        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}
        t1.interrupt();
        t2.interrupt();
    }
}

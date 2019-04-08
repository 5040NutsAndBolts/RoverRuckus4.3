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
@Autonomous(name="Depot Auto Cycles", group="Auto")
public class DepotAutoCycles extends AutoMethods {

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

        //gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters);
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.collectionSlide.setPower(0.6);
        robot.collectionSlide.setTargetPosition(0);

        robot.hang.setTargetPosition(0);
        robot.hang.setPower(1);

        robot.scoringSlide.setTargetPosition(0);
        robot.scoringSlide.setPower(0);

        collection.wristSetPosition(0.75);
        robot.scoringStop.setPosition(0);
        robot.intakeStop.setPosition(0.37);
        robot.sampleArm.setPosition(1);

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

            telemetry.addData("imu calabration", robot.imu.isGyroCalibrated());
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
            if(goldPos != 2) {
                if (robot.collectionSlide.getCurrentPosition() > 120) {
                    collection.wristPos = collection.wristDownPos;
                }
            }
            else {
                collection.wristPos = collection.wristDownPos;
            }
            robot.collectionSlide.setPower(0.45);
            robot.collectionSlide.setTargetPosition(850);

        }

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }

        robot.intake.setPower(0.65);
        time.reset();
        while(time.seconds() > 0.15 && opModeIsActive()){}
        robot.collectionSlide.setTargetPosition(0);

        if(goldPos ==1) {
            runToRotateWait(-44,robot,driveTrain);
        }
        else if(goldPos == 3) {
            runToRotateWait(45, robot, driveTrain);
        }
        else {
            time.reset();
            while(time.seconds() < 0.5 && opModeIsActive()){}
        }

        time.reset();
        while(time.seconds() < 0.2 && opModeIsActive()){}
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            robot.intake.setPower(0);
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();

            if(robot.collectionSlide.getCurrentPosition() < 250) {
                collection.wristPos = collection.wristUpPos;
            }
        }
        collection.wristPos = collection.wristUpPos;
        robot.intake.setPower(0);

        robot.collectionSlide.setTargetPosition(0);
        robot.collectionSlide.setPower(0.6);

        if(goldPos == 1) {
            runToRotateWait(-43, robot, driveTrain);
        }
        else if(goldPos == 2){
            runToRotateWait(-90,robot,driveTrain);
        }
        else {
            runToRotateWait(-137, robot, driveTrain);
        }
        time.reset();
        while(time.seconds() < 0.2 && opModeIsActive()){}


        //drive to crater
        runToForwardWait(45,robot,driveTrain);
        runToRotateWait(-45,robot,driveTrain);

        power = 1;
        driveTrain.resetMotors();
        robot.leftDriveFront.setTargetPosition(1200);
        robot.rightDriveRear.setTargetPosition(1200);
        robot.leftDriveRear.setTargetPosition(0);
        robot.rightDriveFront.setTargetPosition(0);
        driveTrain.powerSet(power);
        while (Math.abs(robot.rightDriveRear.getTargetPosition()-robot.rightDriveRear.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            driveTrain.powerSet(power);
        }

        robot.intake.setPower(-1);
        collection.wristPos = collection.wristDownPos;

        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}

        robot.collectionSlide.setPower(0.45);
        robot.collectionSlide.setTargetPosition(650);

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 20 && opModeIsActive()) {}

        robot.collectionSlide.setTargetPosition(0);

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 20 && opModeIsActive()) {
            if(robot.collectionSlide.getCurrentPosition() < 250) {
                collection.wristPos = collection.wristUpPos;
            }
            if(robot.intakeDetector.alpha() > 120) {
                robot.intake.setPower(-0.8);
                robot.intakeStop.setPosition(0.37);
            }
        }

        //runs back to go to lander
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-8);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<400) {
                if (power > 0.25)
                    power -= 0.05;
                else
                    power += 0.05;
            }
            else {
                if (power < 0.7)
                    power += 0.05;
            }
            if(robot.intakeDetector.alpha() > 120) {
                robot.intake.setPower(-0.8);
                robot.intakeStop.setPosition(0.37);
            }
            driveTrain.powerSet(power);
        }

        runToSidewaysWait(5,robot,driveTrain);
        robot.intake.setPower(0);
        runToRotateWait(47,robot,driveTrain);
        robot.scoringSlide.setPower(0.8);
        robot.scoringSlide.setTargetPosition(1000);
        telemetry.addData("scoring Slide mode", robot.scoringSlide.getMode());
        telemetry.addData("scoring slide power", robot.scoringSlide.getPower());
        telemetry.addData("scoring slide target pos", robot.scoringSlide.getTargetPosition());
        telemetry.addData("scoring slide cur pos", robot.scoringSlide.getCurrentPosition());
        telemetry.update();
        time.reset();
        //while(time.seconds() < 5 && opModeIsActive()){}


        runToForwardWait(-42,robot,driveTrain);
        runToRotateWait(85,robot,driveTrain);
        runToForwardWait(-20,robot,driveTrain);

        //deposits mineral
        robot.scoringStop.setPosition(0.3);
        time.reset();
        while(time.seconds() < 0.3 && opModeIsActive()){}

        runToForwardWait(17,robot,driveTrain);
        robot.scoringSlide.setTargetPosition(0);
        robot.scoringSlide.setPower(0.5);

        runToRotateWait(-86,robot,driveTrain);
        runToForwardWait(47,robot,driveTrain); System.out.println("oof ouch owie my bones");

        runToRotateWait(-40,robot,driveTrain);

        runToForwardWait(15,robot,driveTrain);

        collection.wristPos = 0.4;
        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}
        t1.interrupt();
        t2.interrupt();
    }
}

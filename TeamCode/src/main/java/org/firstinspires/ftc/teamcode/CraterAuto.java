package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

        robot.collectionSlide.setPower(0.3);
        robot.collectionSlide.setTargetPosition(0);

        robot.hang.setTargetPosition(0);
        robot.hang.setPower(1);

        collection.wristSetPosition(1);
        robot.scoringStop.setPosition(0);
        robot.intakeStop.setPosition(0);

        //moves the teamMarker servo to starting position
        //robot.teamMarker.setPosition(0);

        //sets up the detector for mineral detection
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        dogeCVSetup(detector);


        boolean spotToggle = false;
        //waits here til you hit start or stop
        while(!isStarted()) {
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
        int goldPos = landing(40,robot,detector, driveTrain);

        driveTrain.powerSet(1);
        time.reset();
        while(time.seconds() < 0.2 && opModeIsActive()){}

        //run to forward away from lander to the swing turn
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-48);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 200 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<1800) {
                power -= 0.075;
                if(power < 0.2)
                    power = 0.2;
            }
            else {
                if (power < 0.6)
                    power += 0.05;
            }
            power+=0.05;
            driveTrain.powerSet(power);
        }
        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());

        time.reset();

        driveTrain.powerSet(0.5);

        while(time.seconds()<1 && opModeIsActive()){}

        //turn for getting to face the depot
        runToRotateWait(100,robot,driveTrain);

        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());

        time.reset();

        driveTrain.powerSet(1);

        while(time.seconds()<1 && opModeIsActive()){}

        //moves forward to be able to place TM in zone
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(25);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 200 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<1100) {
                power -= 0.07;
                if(power < 0.2)
                    power = 0.2;
            }
            else {
                if (power < 0.55)
                    power += 0.05;
            }
            power+=0.05;
            driveTrain.powerSet(power);
        }
        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());

        time.reset();

        driveTrain.powerSet(0.5);

        while(time.seconds()<1 && opModeIsActive()){}

        collection.wristSetPosition(0.35);

        //runs slide out for placing the TM
        robot.collectionSlide.setPower(1);
        robot.collectionSlide.setTargetPosition(1000);

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.update();
        }

        //starts dropping TM
        time.reset();
        robot.intake.setPower(0.7);
        while(time.seconds()<0.15 && opModeIsActive()){}
        robot.intake.setPower(0);

        //runs slide out for placing the TM
        robot.collectionSlide.setPower(1);
        robot.collectionSlide.setTargetPosition(0);
        //sets wrist up
        collection.wristSetPosition(0.9);

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.update();
        }

        runToForwardWait(-4,robot,driveTrain);
        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());
        time.reset();

        driveTrain.powerSet(1);

        while(time.seconds()<0.5 && opModeIsActive()){}

        //rotates to drive back to samples
        runToRotateWait(45,robot,driveTrain);

        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());
        time.reset();

        driveTrain.powerSet(0.1);

        while(time.seconds()<0.5 && opModeIsActive()){}

        power = 0;//drives back to samples
        driveTrain.forwardInch(-42);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 200 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<1500) {
                power -= 0.075;
                if(power < 0.2)
                    power = 0.2;
            }
            else {
                if (power < 0.6)
                    power += 0.05;
            }
            power+=0.05;
            driveTrain.powerSet(power);
        }
        robot.rightDriveFront.setTargetPosition(robot.rightDriveFront.getCurrentPosition());
        robot.leftDriveFront.setTargetPosition(robot.leftDriveFront.getCurrentPosition());
        robot.rightDriveRear.setTargetPosition(robot.rightDriveRear.getCurrentPosition());
        robot.leftDriveRear.setTargetPosition(robot.leftDriveRear.getCurrentPosition());

        time.reset();

        driveTrain.powerSet(0.5);

        while(time.seconds()<1 && opModeIsActive()){}

        if(goldPos == 1) {
            collection.wristSetPosition(collection.wristDownPos);
            runToRotateWait(68,robot,driveTrain);
            robot.intake.setPower(-1);

            robot.collectionSlide.setTargetPosition(550);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            collection.wristSetPosition(0.9);
            robot.collectionSlide.setTargetPosition(0);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            time.reset();
            while (time.seconds()<0.2 && opModeIsActive()){}

        }
        else if(goldPos == 2) {
            runToRotateWait(120,robot,driveTrain);

            collection.wristSetPosition(collection.wristDownPos);
            time.reset();
            while(time.seconds()<1 && opModeIsActive()){}

            robot.intake.setPower(-1);

            robot.collectionSlide.setTargetPosition(450);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            collection.wristSetPosition(0.9);
            robot.collectionSlide.setTargetPosition(0);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            time.reset();
            while (time.seconds()<0.2 && opModeIsActive()){}
        }
        else {
            runToRotateWait(150,robot,driveTrain);

            collection.wristSetPosition(collection.wristDownPos+0.02);
            time.reset();
            while(time.seconds()<1 && opModeIsActive()){}
            robot.intake.setPower(-1);

            robot.collectionSlide.setTargetPosition(900);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            collection.wristSetPosition(0.9);
            robot.collectionSlide.setTargetPosition(0);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.update();
            }

            time.reset();
            while (time.seconds()<0.2 && opModeIsActive()){}
        }

        robot.intakeStop.setPosition(0.35);
        robot.intake.setPower(-1);
        time.reset();
        while(time.seconds()<1 && opModeIsActive()){}
        robot.intakeStop.setPosition(0);
        robot.intake.setPower(0);

        robot.scoringSlide.setPower(1);
        robot.scoringSlide.setTargetPosition(950);

        power = 0;
        int rDegrees = 815;

        robot.leftDriveFront.setTargetPosition(rDegrees);
        robot.leftDriveRear.setTargetPosition(rDegrees);
        robot.rightDriveFront.setTargetPosition(-rDegrees);
        robot.rightDriveRear.setTargetPosition(-rDegrees);
        driveTrain.powerSet(power);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }

        runToForwardWait(-19,robot,driveTrain);
        robot.scoringStop.setPosition(0.5);
        time.reset();
        while(time.seconds()<0.5 && opModeIsActive()){}

        runToForwardWait(22,robot,driveTrain);
        robot.scoringSlide.setTargetPosition(0);

        robot.collectionSlide.setTargetPosition(700);
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.update();
        }
        collection.wristSetPosition(0.4);
    }
}

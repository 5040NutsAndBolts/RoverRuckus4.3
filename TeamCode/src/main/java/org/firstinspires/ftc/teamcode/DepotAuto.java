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
@Autonomous(name="Depot Auto Other", group="Auto")
public class DepotAuto extends AutoMethods {

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
        int goldPos = landing(285,robot,detector, driveTrain);

        //run to forward for dropping team marker
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(16);


        //sets wrist down for placing the TM
        collection.wristSetPosition(0.4);
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

            if(robot.rightDriveFront.getCurrentPosition() > 200){
                //runs slide out for placing the TM
                robot.collectionSlide.setPower(1);
                robot.collectionSlide.setTargetPosition(1000);
            }
        }
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }

        //starts dropping TM
        time.reset();
        robot.intake.setPower(0.4);
        while(time.seconds()<0.15 && opModeIsActive()){}
        robot.collectionSlide.setTargetPosition(0);
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                robot.intake.setPower(0);
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }

        robot.collectionSlide.setPower(1);
        robot.collectionSlide.setTargetPosition(-10);

        if(goldPos == 1) {
            //turns to face the gold mineral
            runToRotateWait(-47,robot,driveTrain);

            //sets wrist down to pick up gold mineral
            collection.wristSetPosition(0.255);
            time.reset();
            while(time.seconds() < 0.2 && opModeIsActive()){}
            robot.intake.setPower(-1);
            robot.collectionSlide.setTargetPosition(400);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }

            collection.wristSetPosition(1);
            time.reset();
            //pulls slide back in
            robot.collectionSlide.setTargetPosition(0);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }

            runToRotateWait(47,robot,driveTrain);
            robot.intake.setPower(-0.6);
        }
        else if(goldPos == 2) {
            runToForwardWait(-5,robot,driveTrain);

            collection.wristSetPosition(0.255);
            time.reset();
            while(time.seconds() < 0.2 && opModeIsActive()){}
            robot.intake.setPower(-1);
            runToForwardWait(7,robot,driveTrain);

            collection.wristSetPosition(1);
            time.reset();

            robot.intake.setPower(-0.6);
            time.reset();
            while(time.seconds()<1 && opModeIsActive()){}
        }
        else {
            //turns to face the gold mineral
            runToRotateWait(43,robot,driveTrain);

            //sets wrist down to pick up gold mineral
            collection.wristSetPosition(0.255);
            time.reset();
            while(time.seconds() < 0.2 && opModeIsActive()){}
            robot.intake.setPower(-1);
            robot.collectionSlide.setTargetPosition(400);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }

            collection.wristSetPosition(1);
            time.reset();
            //pulls slide back in
            robot.collectionSlide.setTargetPosition(0);
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }

            runToRotateWait(-43,robot,driveTrain);
            robot.intake.setPower(-0.6);
        }

        robot.intakeStop.setPosition(0.35);
        time.reset();
        while(time.seconds()<0.5 && opModeIsActive()){}
        robot.intakeStop.setPosition(0);
        robot.intake.setPower(0);

        robot.scoringSlide.setPower(1);
        robot.scoringSlide.setTargetPosition(1000);

        while(Math.abs(robot.scoringSlide.getTargetPosition()-robot.scoringSlide.getCurrentPosition()) > 10 && opModeIsActive()) { }

        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-18);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos", robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }


        robot.scoringStop.setPosition(0.5);
        time.reset();
        while(time.seconds()<0.5 && opModeIsActive()){}
        runToForwardWait(17,robot,driveTrain);

        robot.scoringStop.setPosition(0);
        robot.scoringSlide.setTargetPosition(0);

        runToRotateWait(-95,robot,driveTrain);

        runToForwardWait(35,robot,driveTrain);
        runToRotateWait(-45,robot,driveTrain);
        runToSidewaysWait(-10,robot,driveTrain);
        runToForwardWait(15,robot,driveTrain);
        collection.wristSetPosition(0.4);
    }
}

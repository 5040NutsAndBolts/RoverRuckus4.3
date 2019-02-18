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
        int goldPos = landing(222,robot,detector, driveTrain);

        //run to forward away from lander to the swing turn
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(30);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 300 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }

        if(power > 1){
            power=1;
        }

        //turn for getting to face the crater. SHOULD BE A SMOOTH TRANSITION FROM THE MOVE FORWARD
        driveTrain.rotate(-160);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<600) {
                if (power > 0.2)
                    power -= 0.05;
                else
                    power += 0.05;
            }
            else {
                if (power < 1)
                    power += 0.05;
            }
            driveTrain.powerSet(power);
        }

        //runs slide out for placing the TM
        robot.collectionSlide.setPower(1);
        robot.collectionSlide.setTargetPosition(1000);
        //sets wrist down for placing the TM
        collection.wristSetPosition(0.4);

        //moves forward to be able to place TM in zone
        driveTrain.forwardInch(7);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
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
        robot.intake.setPower(0.35);
        while(time.seconds()<0.15 && opModeIsActive()){}
        robot.intake.setPower(0);

        //runs slide out for placing the TM
        robot.collectionSlide.setPower(1);
        robot.collectionSlide.setTargetPosition(-10);
        //sets wrist up
        collection.wristSetPosition(0.9);

        power = 0;
        //moves forward to be able to place TM in zone
        driveTrain.forwardInch(-10);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            power += 0.05;
            driveTrain.powerSet(power);
        }

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 30 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
        }

        if(power > 1){
            power=1;
        }


        if(goldPos == 1) {

        }
        else if(goldPos == 2) {

        }
        else {

        }

        while (opModeIsActive()){}
    }
}

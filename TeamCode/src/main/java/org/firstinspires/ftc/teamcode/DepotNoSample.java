package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Auto for the depot landing spot
 */
@Autonomous(name="Depot No Sample Opp Crater", group="Auto")
@Disabled
public class DepotNoSample extends AutoMethods {

    //class objects
    private Hardware robot;
    private MecanumDrive driveTrain;
    private GoldAlignDetector detector;
    private String driverSpot = "Depot";

    /**
     * The method that gets run when you hit init
     */
    @Override
    public void runOpMode() {
        //sets up the objects for the other classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

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

        //robot.wrist.setPower(0.5);
        //robot.wrist.setTargetPosition(0);
        robot.collectionSlide.setPower(0.3);
        robot.collectionSlide.setTargetPosition(0);

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
        int goldPos = landing(0,robot,detector, driveTrain);

        runToSidewaysWait(10,robot,driveTrain);

        runToForwardWait(-30,robot,driveTrain);

        runToRotateWait(-140,robot,driveTrain);

        runToSidewaysWait(-20,robot,driveTrain);

        runToForwardWait(-60,robot,driveTrain);

        //places the team marker in the depot
        //robot.teamMarker.setPosition(.27);

        runToSidewaysWait(-5,robot,driveTrain);

        //powers of the hanging motor to conserve power and not break the motor
        robot.hang.setPower(0);

        //parks in crater
        runToForwardWait(85,robot,driveTrain);
        /*runToForwardWait(55,robot,driveTrain);

        runToSidewaysWait(10,robot,driveTrain);
        runToRotateWait(180,robot,driveTrain);
        runToSidewaysWait(15,robot,driveTrain);
        runToForwardWait(-35,robot,driveTrain);*/

        // Writes gyro angle to the data file
        writeToFile(robot, driverSpot);
    }
}

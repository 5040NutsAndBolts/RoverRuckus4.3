package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Auto for the depot landing spot
 */
@Autonomous(name="Crater", group="Auto")
public class CraterAuto extends AutoMethods {

    //class objects
    private Hardware robot;
    private MecanumDrive driveTrain;
    private GoldAlignDetector detector;
    private String driverSpot = "Crater";

    /**
     * The method that gets run when you hit init
     */
    @Override
    public void runOpMode() {
        //sets up the objects for the other classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        //initializes the robot hardware and sets powers so things don't move
        robot.init(hardwareMap);

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

        robot.wrist.setPower(0.5);
        robot.wrist.setTargetPosition(0);
        robot.collectionSlide.setPower(0.3);
        robot.collectionSlide.setTargetPosition(0);

        //moves the teamMarker servo to starting position
        robot.teamMarker.setPosition(0);

        //sets up the detector for mineral detection
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        dogeCVSetup(detector);

        //waits here til you hit start or stop
        while(!isStarted()) {
            telemetry.addLine("ready");
            if(gamepad1.x){
                if(driverSpot.equals("Crater"))
                    driverSpot = "Depot";
                else
                    driverSpot = "Crater";
            }
            telemetry.addData("Driver spot",driverSpot);
            telemetry.update();
        }
        //enables the detector until it scans the minerals
        detector.enable();

        //lands the robot and returns what position the gold mineral is in
        int goldPos = landing(robot,detector, driveTrain);


        //GOLD ON THE LEFT
        if(goldPos == 1) {
            //rotates to face gold
            runToRotateWait(120, robot, driveTrain);
            //knocks off gold
            runToForwardWait(20, robot, driveTrain);
            //rotates to be level with wall
            runToRotateWait(-80, robot, driveTrain);
            //runs into wall
            runToForwardWait(-15,robot,driveTrain);
            runToSidewaysWait(20, robot, driveTrain);
        }
        //GOLD IN THE MIDDLE
        else if(goldPos == 2) {
            //rotates to face gold
            runToRotateWait(75,robot,driveTrain);
            //knocks off gold
            runToForwardWait(20,robot,driveTrain);
            runToForwardWait(-10,robot,driveTrain);
            //rotates to be level with wall
            runToRotateWait(-80,robot,driveTrain);
            //runs into wall
            runToForwardWait(-35,robot,driveTrain);
            runToRotateWait(45,robot,driveTrain);
            runToSidewaysWait(10,robot,driveTrain);
        }
        //GOLD ON THE RIGHT
        else {
            //rotates to face gold
            runToRotateWait(35,robot,driveTrain);
            //knocks off gold
            runToForwardWait(30,robot,driveTrain);
            //rotates to be level with wall
            runToRotateWait(-40,robot,driveTrain);
            //runs into wall
            runToSidewaysWait(-10,robot,driveTrain);
            runToForwardWait(-55,robot,driveTrain);
            runToRotateWait(45,robot,driveTrain);
            runToSidewaysWait(15,robot,driveTrain);
        }

        //moves into the depot
        runToForwardWait(-50, robot, driveTrain);

        //places the team marker in the depot
        robot.teamMarker.setPosition(.27);

        //powers of the hanging motor to conserve power and not break the motor
        robot.hangingMotor.setPower(0);

        runToSidewaysWait(3,robot,driveTrain);
        //parks in crater
        runToForwardWait(80,robot,driveTrain);

        // Writes gyro angle to the data file
        writeToFile(robot, driverSpot);
    }
}

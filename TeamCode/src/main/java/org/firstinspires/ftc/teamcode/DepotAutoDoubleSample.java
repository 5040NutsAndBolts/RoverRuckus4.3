package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Auto for the depot landing spot
 */
@Autonomous(name="Depot Double Sample", group="Auto")
//@Disabled
public class DepotAutoDoubleSample extends AutoMethods {

    //class objects
    private Hardware robot;
    private MecanumDrive driveTrain;
    private GoldAlignDetector detector;

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
            telemetry.update();
        }
        //enables the detector until it scans the minerals
        detector.enable();

        //lands the robot and returns what position the gold mineral is in
        int goldPos = landing(robot,detector, driveTrain);


        //GOLD ON THE LEFT
        if(goldPos == 1) {
            //rotates to face gold
            runToRotateWait(22, robot, driveTrain);
            //knocks off gold
            runToSidewaysWait(25, robot, driveTrain);
            //rotates to be level with wall
            runToRotateWait(300, robot, driveTrain);
            //runs into wall
            runToSidewaysWait(40, robot, driveTrain);
        }
        //GOLD IN THE MIDDLE
        else if(goldPos == 2) {
            //rotates to face gold
            runToRotateWait(-15,robot,driveTrain);
            //knocks off gold
            runToSidewaysWait(40,robot,driveTrain);
            //rotates to be level with wall
            runToRotateWait(335,robot,driveTrain);
            //runs into wall
            runToSidewaysWait(15,robot,driveTrain);
        }
        //GOLD ON THE RIGHT
        else {
            //rotates to face gold
            runToRotateWait(-50,robot,driveTrain);
            //knocks off gold
            runToSidewaysWait(30,robot,driveTrain);
            //rotates to be level with wall
            runToForwardWait(10,robot,driveTrain);
            runToSidewaysWait(15,robot,driveTrain);
            //runs into wall
            runToForwardWait(-40,robot,driveTrain);
        }

        //places the team marker in the depot
        robot.teamMarker.setPosition(.27);

        //powers of the hanging motor to conserve power and not break the motor
        robot.hangingMotor.setPower(0);

        //drives into the other teams crater
        runToForwardWait(50,robot,driveTrain);
        runToSidewaysWait(-5,robot,driveTrain);
        runToRotateWait(-50,robot,driveTrain);
        if(goldPos == 1) {
            runToForwardWait(20,robot,driveTrain);
            runToSidewaysWait(10,robot,driveTrain);
            runToRotateWait(90,robot,driveTrain);
            runToForwardWait(10,robot,driveTrain);
        }else if(goldPos == 2) {
            runToForwardWait(40,robot,driveTrain);
            runToSidewaysWait(10,robot,driveTrain);
            runToSidewaysWait(-10,robot,driveTrain);
            runToForwardWait(-40,robot,driveTrain);
            runToRotateWait(50,robot,driveTrain);
            runToSidewaysWait(5,robot,driveTrain);
            runToForwardWait(20,robot,driveTrain);
        }else {
            runToForwardWait(60,robot,driveTrain);
            runToSidewaysWait(10,robot,driveTrain);
            runToRotateWait(90,robot,driveTrain);
            runToForwardWait(10,robot,driveTrain);
        }
    }
}

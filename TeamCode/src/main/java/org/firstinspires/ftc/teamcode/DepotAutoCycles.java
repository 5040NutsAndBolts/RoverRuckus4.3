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
    private String driverSpot = "Depot";

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

        robot.collectionSlide.setPower(0.6);
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

        robot.hang.setPower(0.1);
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
                robot.collectionSlide.setTargetPosition(900);
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
            robot.intake.setPower(-0.25);
            robot.collectionSlide.setTargetPosition(300);
            robot.collectionSlide.setPower(0.6);
        }


            while (Math.abs(robot.collectionSlide.getTargetPosition() - robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos", robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();

                if(goldPos == 2) {
                    if (Math.abs(robot.collectionSlide.getTargetPosition() - robot.collectionSlide.getCurrentPosition()) > 100 && opModeIsActive()) {
                        //robot.intakeStop.setPosition(0.02);
                    }
                }
            }

        //turns back to face the Depot as well as starts to extend the slide out for placing the TM
        if(goldPos != 2) {
            robot.collectionSlide.setTargetPosition(900);
            robot.collectionSlide.setPower(0.5);

            power = 0;
            driveTrain.powerSet(power);
            int rDegrees = 0;
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
                if(robot.intakeDetector.alpha() > 200) {
                    robot.intake.setPower(0);
                }
            }

            //makes sure slide is all the way out and drops the TM.
            while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }
        }
        robot.intakeStop.setPosition(0.02);
        robot.intake.setPower(1);
        robot.collectionSlide.setTargetPosition(0);

        //pulls slide back in after placing TM
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power)   ;
            telemetry.update();

            //sets wrist up when slid comes in enough but before it hits off the center mineral
            if(robot.collectionSlide.getCurrentPosition() < 500) {
                collection.wristPos = collection.wristUpPos;
            }
        }
        collection.wristPos = collection.wristUpPos;

        robot.intake.setPower(0);

        //rotates to face Crater to go pick up minerals
        runToRotateWait(-85,robot,driveTrain);


        //drive to crater
        runToForwardWait(42,robot,driveTrain);
        runToRotateWait(-50,robot,driveTrain);

        //moves diagnolly so we know where is is located up against the Crater.
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

        driveTrain.powerSet(0.5);
        driveTrain.sidewaysInch(-3);

        //sets wrist down and starts intaking
        robot.intake.setPower(-1);
        collection.wristPos = collection.wristDownPos;

        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}

        //runs slide out and pulls it back in to pick up minerals
        robot.collectionSlide.setPower(0.45);
        robot.collectionSlide.setTargetPosition(650);
        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 20 && opModeIsActive()) {}

        robot.collectionSlide.setTargetPosition(0);

        while(Math.abs(robot.collectionSlide.getTargetPosition()-robot.collectionSlide.getCurrentPosition()) > 20 && opModeIsActive()) {
            if(robot.collectionSlide.getCurrentPosition() < 300) {
                collection.wristPos = collection.wristUpPos;
            }
            //transfers minerals when wrist is all the way back up.
            if(robot.intakeDetector.alpha() > 200) {
                robot.intake.setPower(-0.8);
                robot.intakeStop.setPosition(0.37);
            }
        }

        //moves back to go to lander
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-9.5);
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
            if(robot.intakeDetector.alpha() > 200) {
                robot.intake.setPower(-0.8);
                robot.intakeStop.setPosition(0.37);
            }
            driveTrain.powerSet(power);
        }

        //moves away from the wall to rotate to move level with lander
        runToSidewaysWait(5,robot,driveTrain);
        robot.intake.setPower(0);
        runToRotateWait(50,robot,driveTrain);
        robot.scoringSlide.setPower(0.8);
        robot.scoringSlide.setTargetPosition(1100);
        telemetry.addData("scoring Slide mode", robot.scoringSlide.getMode());
        telemetry.addData("scoring slide power", robot.scoringSlide.getPower());
        telemetry.addData("scoring slide target pos", robot.scoringSlide.getTargetPosition());
        telemetry.addData("scoring slide cur pos", robot.scoringSlide.getCurrentPosition());
        telemetry.update();


        runToForwardWait(-43,robot,driveTrain);
        runToRotateWait(90,robot,driveTrain);
        power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(-25);
        time.reset();
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive() && time.seconds() < 2.1) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<400) {
                if (power > 0.2)
                    power -= 0.05;
                else
                    power += 0.05;
            }
            else {
                if (power < 0.4)
                    power += 0.05;
            }
            driveTrain.powerSet(power);
        }
        driveTrain.powerSet(0);

        //deposits mineral
        robot.scoringStop.setPosition(0.35);
        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}

        runToForwardWait(16,robot,driveTrain);

        runToRotateWait(-85,robot,driveTrain);
        runToForwardWait(49,robot,driveTrain);
        robot.scoringSlide.setTargetPosition(0);
        robot.scoringSlide.setPower(0.5);

        runToRotateWait(-40,robot,driveTrain);

        runToForwardWait(15,robot,driveTrain);

        collection.wristPos = 0.4;
        time.reset();
        while(time.seconds() < 0.5 && opModeIsActive()){}
        t1.interrupt();
        t2.interrupt();

        if(driverSpot.equals("Depot")) {
            writeToFile(robot, "-2.115");
        }
        else {
            writeToFile(robot, "2.9005");
        }
    }
}

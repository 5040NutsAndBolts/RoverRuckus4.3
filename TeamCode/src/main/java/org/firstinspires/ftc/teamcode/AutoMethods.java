package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.IOException;

/**
 * Class for auto methods. The methods in here have loops that are used in auto and need the parameter of opModeIsActive().
 * The methods may also not fit into any other class such as the dogeCVSetup.
 */
@Disabled
class AutoMethods extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {}

    /**
     * sets up the detector for sampling
     * @param detector - the DogeCV object for configuring
     */
    public void dogeCVSetup(GoldAlignDetector detector) {
        // Optional Tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned.
                                  // (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
    }

    /**
     * moves the robot forward so many inches while increasing speed.
     * stays in loop until it reaches distance.
     * @param forwardInches - how for forward it needs to go
     * @param robot - the hardware object to move the motors
     * @param driveTrain - the MecanumDrive object so it can use the forwardInch method
     */
    public void runToForwardWait(int forwardInches,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(forwardInches);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            driveTrain.forwardPID();
        }
    }

    /**
     * moves the robot sideways so many inches while increasing speed
     * stays in loop until it reaches distance
     * @param sidewaysInches - how far sideways it needs to go
     * @param robot - the hardware object to move the motors
     * @param driveTrain - the MecanumDrive object so it can use the sidewaysInch method
     */
    public void runToSidewaysWait(int sidewaysInches,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.sidewaysInch(sidewaysInches);
            while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
                if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<400) {
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
            }
    }

    /**
     * moves the robot sideways so many inches while increasing speed
     * stays in loop until it reaches distance
     * @param forewardInches - how far sideways it needs to go
     * @param robot - the hardware object to move the motors
     * @param driveTrain - the MecanumDrive object so it can use the sidewaysInch method
     */
    public void runToForewardWaitPID(int forewardInches,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(forewardInches);
        while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
            telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
            telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.update();
            driveTrain.powerSet(power);
        }            if(Math.abs(robot.rightDriveFront.getCurrentPosition()-robot.rightDriveFront.getTargetPosition())<400) {
                if (power > 0.4)
                    power -= 0.05;
                else
                    power += 0.05;
            }
            else {
                if (power < 0.7)
                    power += 0.05;
            }
    }

    /**
     * rotates the robot so many degrees while increasing speed
     * stays in loop until it reaches rotation
     * @param degrees - how many degrees it needs to rotate
     * @param robot - the hardware object to move the motors
     * @param driveTrain - the MecanumDrive object so it can use the rotate method
     */
    public void runToRotateWait(int degrees,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.rotate(degrees);
            while (Math.abs(robot.rightDriveFront.getTargetPosition()-robot.rightDriveFront.getCurrentPosition()) > 20 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("collection slide current Pos",robot.collectionSlide.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
                power+=0.1;
                driveTrain.powerSet(power);
            }
    }

    /**
     *This method lands the robot and scans the minerals for where the gold one is located
     * @param robot - the hardware object to move the motors
     * @param detector - the DogeCV object for scannign the minerals
     * @param driveTrain - the MecanumDrive object so it can use movements within those classes
     * @return - returns what position the gold mineral is in, 1-is left,2-is middle,3-is right from the phone.
     */
    public int landing(int rotate,Hardware robot, GoldAlignDetector detector, MecanumDrive driveTrain) {
        int goldPos = 3;
        //lowers hang mechanism
        robot.hang.setTargetPosition(6089);
        robot.hang.setPower(1);
        while(opModeIsActive() && robot.hang.getCurrentPosition() < robot.hang.getTargetPosition()-20) {
            //scans for gold mineral spot
            if(robot.hang.getCurrentPosition() < 3000) {
                if (detector.getYPosition() > 200) {
                    goldPos = 2;
                    detector.disable();
                } else if (detector.getYPosition() > 0) {
                    goldPos = 1;
                    detector.disable();
                }
            }
            else {
                detector.disable();
            }
            telemetry.addData("detector aligned", detector.getAligned());
            telemetry.addData("goldPos",goldPos);
            telemetry.addData("x pos of gold", detector.getXPosition());
            telemetry.addData("hanging slide power", robot.hang.getPower());
            telemetry.addData("hanging motor pos",robot.hang.getCurrentPosition());
            telemetry.addData("hanging motor set pos",robot.hang.getTargetPosition());
            telemetry.update();
        }

        //moves hook off the lander
        //runToForwardWait(-4,robot,driveTrain);
        runToRotateWait(rotate,robot,driveTrain);
        if(rotate != 0) {
            robot.hang.setTargetPosition(0);
        }

        //moves away from hook
        //runToSidewaysWait(4,robot,driveTrain);
        return goldPos;
    }

    /**
     *  This method creates the file that the gyro angle is written to and writes that
     *  @param robot - object used to get the gyro readings and where the file is stored
     *  @param angle - determines the position of the driver and how much to add to the gyro adjust
     */
    public void writeToFile(Hardware robot, String angle){
        //makes a new (or clears the) text file
        try {
            robot.exportData = new FileHelper();
            robot.exportData.clearFile();
        } catch (IOException e) {
            telemetry.addLine("IOException: \n"+e);
            telemetry.update();
        }
        //write to the text file
        try {

            robot.exportData.writeToFile(""+angle);
        } catch (IOException e) {
            telemetry.addLine("IOException in file writing: \n"+e);
            telemetry.update();
        }
    }
}

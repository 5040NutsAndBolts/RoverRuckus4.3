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

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Disabled
class AutoMethods extends LinearOpMode {


    @Override public void runOpMode() throws InterruptedException {}

    public void Sample() {

    }
    public void dogeCVSetup(GoldAlignDetector detector) {
       /*detector = new GoldAlignDetector();
       detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
       detector.useDefaults();*/

        // Optional Tuning
        detector.alignSize = 500; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void runToWaitForward(int forwardAmount,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.forwardInch(forwardAmount);
        if(forwardAmount < 0) {
            while (robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
        else {
            while (robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
    }

    public void runToWaitSideways(int sidewaysAmount,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.sidewaysInch(sidewaysAmount);
        if(sidewaysAmount < 0) {
            while (robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
        else {
            while (robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
    }

    public void runToWaitRotate(int rotateAmount,Hardware robot, MecanumDrive driveTrain) {
        double power = 0;
        driveTrain.powerSet(power);
        driveTrain.rotate(rotateAmount);
        if(rotateAmount > 0) {
            while (robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
        else {
            while (robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos", robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos", robot.rightDriveFront.getTargetPosition());
                telemetry.addData("rightDriveFront power", robot.rightDriveFront.getPower());
                telemetry.addData("power", power);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
    }

    public void sampling(int goldPos, Hardware robot, MecanumDrive driveTrain) {
        int power;
        //GOLD ON THE LEFT
        if(goldPos == 1) {
            driveTrain.rotate(25);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(20);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            //drive into wall
            driveTrain.rotate(25);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-5);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(28);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

        }
        //GOLD IN THE MIDDLE
        else if(goldPos == 2) {
            driveTrain.rotate(-30);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+10 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(17);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //drive into wall
            driveTrain.rotate(70);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-20);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(35);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
        }
        //GOLD ON THE RIGHT
        else {
            driveTrain.rotate(-55);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            //knocks off sample
            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(22);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }
            //drive into wall
            driveTrain.rotate(90);
            power = 0;
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-5 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.forwardInch(-29);
            while(robot.rightDriveFront.getTargetPosition() < robot.rightDriveFront.getCurrentPosition()-50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

            power = 0;
            driveTrain.powerSet(power);
            driveTrain.sidewaysInch(45);
            while(robot.rightDriveFront.getTargetPosition() > robot.rightDriveFront.getCurrentPosition()+50 && opModeIsActive()) {
                telemetry.addData("rightDriveFront pos",robot.rightDriveFront.getCurrentPosition());
                telemetry.addData("rightDriveFront target pos",robot.rightDriveFront.getTargetPosition());
                telemetry.addData("goldPos", goldPos);
                telemetry.update();
                power += 0.05;
                driveTrain.powerSet(power);
            }

        }
        robot.hangingMotor.setPower(0);
    }

    public int landing(Hardware robot, GoldAlignDetector detector, MecanumDrive driveTrain) {
        int goldPos = 3;
        //lowers hang mechanism
        robot.hangingMotor.setTargetPosition(6900);
        robot.hangingMotor.setPower(1);
        while(opModeIsActive() && robot.hangingMotor.getCurrentPosition() < 6800) {
            //scans for gold mineral spot
            if(robot.hangingMotor.getCurrentPosition() < 2000) {
                if (detector.getXPosition() > 200) {
                    goldPos = 2;
                    detector.disable();
                } else if (detector.getXPosition() > 10) {
                    goldPos = 1;
                    detector.disable();
                }
            }
            telemetry.addData("detector aligned", detector.getAligned());
            telemetry.addData("goldPos",goldPos);
            telemetry.addData("x pos of gold", detector.getXPosition());
            telemetry.addData("hanging slide power", robot.hangingMotor.getPower());
            telemetry.addData("hanging motor pos",robot.hangingMotor.getCurrentPosition());
            telemetry.update();
        }

        //moves hook off the lander
        runToWaitForward(-5,robot,driveTrain);
        robot.hangingMotor.setTargetPosition(0);

        //moves away from hook
        runToWaitSideways(5,robot,driveTrain);
        return goldPos;
    }
}

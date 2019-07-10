package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.IOException;


@TeleOp(name="HangStressTesting", group="Teleop")
@Disabled
public class HangStressTesting extends OpMode {

    //sets up the objects for other classes
    private Hardware robot;
    private MineralScorer mineralScorer;
    private MecanumDrive driveTrain;
    private LiftMechanism lifter;
    private Collection collection;
    private boolean toggle = false;
    private boolean toggle2 = false;
    private double pos = 0.5;

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
     * sets up the objects for the other classes
     */
    public HangStressTesting() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        lifter = new LiftMechanism(robot);
        mineralScorer = new MineralScorer(robot);
    }

    /**
     * method runs once on init
     * init's the hardware of the robot
     */
    public void init() {
        robot.init(hardwareMap,true);
        robot.hang.setPower(1);
        robot.hang.setTargetPosition(0);
        robot.collectionSlide.setPower(0.8);
        robot.collectionSlide.setTargetPosition(0);

        //t1.start();
        //t2.start();

        //collection.wristPos = collection.wristUpPos;
    }

    /**
     * method runs until it is not in the init phase
     * tells you if the gyro is calibrated
     */
    @Override
    public void init_loop() {
        telemetry.update();
    }

    /**
     * runs once you hit start
     * drops the team marker if it didn't during auto
     */
    @Override
    public void start() {
        driveTrain.resetMotors();
    }

    /**
     * loops while in the play phase
     */
    public void loop() {

        if(gamepad1.x && !toggle) {
            toggle = true;
            pos = .84;
            robot.wristRight.setPosition(pos);
        }
        else if(!gamepad1.x && toggle) {
            toggle = false;
        }
        if(gamepad1.y && !toggle2) {
            toggle2 = true;
            pos = .16;
            robot.wristRight.setPosition(pos);
        }
        else if(!gamepad1.x && toggle2) {
            toggle2 = false;
        }

        telemetry.addData("servoPosition", robot.wristRight.getPosition());
        telemetry.update();
        /*if(gamepad1.x) {
            robot.leftDriveFront.setPower(1);
            robot.rightDriveRear.setPower(1);
            robot.leftDriveRear.setPower(1);
            robot.rightDriveFront.setPower(1);
            robot.leftDriveRear.setTargetPosition(9999999);
            robot.rightDriveFront.setTargetPosition(9999999);
            robot.leftDriveFront.setTargetPosition(0);
            robot.rightDriveRear.setTargetPosition(0);
        }
        else {
            robot.leftDriveFront.setPower(0);
            robot.rightDriveRear.setPower(0);
            robot.leftDriveRear.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.leftDriveRear.setTargetPosition(0);
            robot.rightDriveFront.setTargetPosition(0);
            robot.leftDriveFront.setTargetPosition(0);
            robot.rightDriveRear.setTargetPosition(0);
            driveTrain.resetMotors();
        }*/
        /*if(robot.hang.getTargetPosition() == 6089) {
            if(robot.hang.getCurrentPosition() > 5989){
                robot.hang.setTargetPosition(0);
            }
        }
        else {
            if(robot.hang.getCurrentPosition() < 100){
                robot.hang.setTargetPosition(6089);
            }
        }*/
    }
}

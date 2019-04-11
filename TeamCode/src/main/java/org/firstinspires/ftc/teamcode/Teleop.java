package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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


@TeleOp(name="Teleop", group="Teleop")

public class Teleop extends OpMode {

    //sets up the objects for other classes
    private Hardware robot;
    private MineralScorer mineralScorer;
    private MecanumDrive driveTrain;
    private LiftMechanism lifter;
    private Collection collection;
    private Controllers controllers;

    private boolean brake = false;

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
    public Teleop() {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        lifter = new LiftMechanism(robot);
        mineralScorer = new MineralScorer(robot);
        controllers = new Controllers();
    }

    /**
     * method runs once on init
     * init's the hardware of the robot
     */
    public void init() {
        robot.init(hardwareMap,false);
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
    }

    /**
     * method runs until it is not in the init phase
     * tells you if the gyro is calibrated
     */
    @Override
    public void init_loop() {
        telemetry.addData("imu calabration", robot.imu.isGyroCalibrated());
        telemetry.update();
    }

    /**
     * runs once you hit start
     * drops the team marker if it didn't during auto
     */
    @Override
    public void start() {
        collection = new Collection(robot);
        // creates a new reference for the file and parses the line to a double
        //      Will fix later if the exportData will always be a double
        try {
            robot.exportData = new FileHelper();
        } catch (IOException e) { }
        try {
            driveTrain.adjust = Double.parseDouble(robot.exportData.readFromFile());
            robot.exportData.clearFile();
            //writeToFile(robot);
        } catch (Exception e) {
            driveTrain.adjust = 0;
            //robot.resetMotors();
        }robot.scoringStop.setPosition(0);

        //starts the threads runs run()
        t1.start();
        t2.start();
    }

    /**
     * loops while in the play phase
     */
    public void loop() {
        //t1.run();
        //t2.run();
        controllers.update(gamepad1, gamepad2);

        //the scoring method calls
        mineralScorer.slide(controllers.x_button2);
        mineralScorer.mineralStop(controllers.right_bumper2);

        //the collection method calls
        collection.wrist(controllers.left_bumper2);
        collection.inTake(controllers.left_bumper2, controllers.y_button2);
        collection.slide(controllers.right_bumper1, controllers.left_bumper1);

        //the lift call
        lifter.lift(controllers.left_trigger1,  controllers.right_trigger1);

            if(controllers.dleft1) {
                controllers.left_stick_x1 = -0.4;
            }
            else if(controllers.dright1) {
                controllers.left_stick_x1 = 0.4;
            }
            else if(controllers.dup1) {
                controllers.left_stick_y1 = -0.4;
            }
            else if(controllers.ddown1) {
                controllers.left_stick_y1 = 0.4;
            }


            if(controllers.left_stick_x1 == 0 && controllers.left_stick_y1 == 0 && controllers.right_stick_x1 == 0 && !controllers.x_button1) {
                driveTrain.brakeMotors();
                brake = true;
            }
            else {
                if(brake) {
                    robot.leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                driveTrain.orientedDrive(controllers.left_stick_y1, controllers.left_stick_x1, -controllers.right_stick_x1, controllers.x_button1);
            }

        //resets
        mineralScorer.reset(controllers.y_button3);
        collection.reset(controllers.right_bumper3);
        lifter.reset(controllers.left_bumper3);

        //telemetry lines
        telemetry.addLine("-------Controllers-------");
        telemetry.addData("start-a is in start-x", controllers.gamepadMode1==1);
        telemetry.addData("start-b is in start-x", controllers.gamepadMode2==1);
        telemetry.addLine("--------IMU--------");
        telemetry.addData("imu heading radians", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("imu heading degrees", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("adjust", driveTrain.adjust);
        telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
        telemetry.addLine("-------DRIVE MOTORS-------");
        telemetry.addData("front left drive", robot.leftDriveFront.getPower());
        telemetry.addData("rear left drive", robot.leftDriveRear.getPower());
        telemetry.addData("front right drive", robot.rightDriveFront.getPower());
        telemetry.addData("rear right drive", robot.rightDriveRear.getPower());
        telemetry.addLine("--------HANGING MOTOR--------");
        telemetry.addData("hanging motor position", robot.hang.getCurrentPosition());
        telemetry.addData("hanging motor target position", robot.hang.getTargetPosition());
        telemetry.addData("hanging motor power", robot.hang.getPower());
        telemetry.addLine("--------COLLECTION WRIST--------");
        telemetry.addData("Left Wrist Position", robot.wristLeft.getPosition());
        telemetry.addData("Right Wrist Position", robot.wristRight.getPosition());
        telemetry.addData("color Sensor argb", robot.intakeDetector.argb());
        telemetry.addData("color Sensor alpha", robot.intakeDetector.alpha());
        telemetry.addLine("--------COLLECTION INTAKE--------");
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addLine("--------COLLECTION SLIDE--------");
        telemetry.addData("collection slide Position", robot.collectionSlide.getCurrentPosition());
        telemetry.addData("collection slide power", robot.collectionSlide.getPower());
        telemetry.addLine("-----SCORING SLIDE-----");
        telemetry.addData("scoring slide position", robot.scoringSlide.getCurrentPosition());
        telemetry.addData("scoring slide power", robot.scoringSlide.getPower());
        telemetry.addLine("-----MINERAL Stop BAR-----");
        telemetry.addData("right Trigger 2", controllers.right_trigger2);
        telemetry.addData("scoring bar position", robot.scoringStop.getPosition());
        telemetry.update();
    }

    public void stop() {
        t1.interrupt();
        t2.interrupt();
    }
}

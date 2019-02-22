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
        collection = new Collection(robot);
        mineralScorer = new MineralScorer(robot);
    }

    /**
     * method runs once on init
     * init's the hardware of the robot
     */
    public void init() {
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
            robot.resetMotors();
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
        //inputs for controller 1
        double leftStickY1 = gamepad1.left_stick_y;
        double leftStickX1 = gamepad1.left_stick_x;
        double rightStickX1 = gamepad1.right_stick_x;
        boolean leftBumper1 = gamepad1.left_bumper;
        boolean rightBumper1 = gamepad1.right_bumper;
        boolean dPadDown1 = gamepad1.dpad_down;
        boolean dPadRight1 = gamepad1.dpad_right;
        boolean dPadLeft1 = gamepad1.dpad_left;
        boolean dPadUp1 = gamepad1.dpad_up;
        boolean x1 = gamepad1.x;
        boolean leftTrigger1 = gamepad1.left_trigger > 0.3;

        //controller 2 input
        boolean leftBumper2 = gamepad2.left_bumper;
        boolean rightBumper2 = gamepad2.right_bumper;
        boolean x2 = gamepad2.x;
        boolean y2 = gamepad2.y;
        boolean rightTrigger2 = gamepad2.right_trigger > 0.3;
        boolean leftTrigger2 = gamepad2.left_trigger > 0.3;
        double leftStickY2 = gamepad2.left_stick_y;

        //the scoring method calls
        mineralScorer.slide(x2);
        mineralScorer.mineralStop(rightTrigger2);

        //the collection method calls
        collection.wrist(leftTrigger2);
        collection.inTake(leftBumper2, y2

        );
        collection.slide(rightBumper1, leftBumper1);
        collection.inTakeStop(rightBumper2);

        //the lift call
        lifter.lift(leftTrigger1, gamepad1.right_trigger > 0.3);

            if(dPadLeft1) {
                leftStickX1 = -0.4;
            }
            else if(dPadRight1) {
                leftStickX1 = 0.4;
            }
            else if(dPadUp1) {
                leftStickY1 = -0.4;
            }
            else if(dPadDown1) {
                leftStickY1 = 0.4;
            }
        driveTrain.orientedDrive(leftStickY1, leftStickX1, -rightStickX1,x1);


        //telemetry lines
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
        telemetry.addLine("--------COLLECTION INTAKE--------");
        telemetry.addData("Intake Power", robot.intake.getPower());
        telemetry.addLine("--------COLLECTION SLIDE--------");
        telemetry.addData("collection slide Position", robot.collectionSlide.getCurrentPosition());
        telemetry.addData("collection slide power", robot.collectionSlide.getPower());
        telemetry.addLine("-----SCORING SLIDE-----");
        telemetry.addData("scoring slide position", robot.scoringSlide.getCurrentPosition());
        telemetry.addData("scoring slide power", robot.scoringSlide.getPower());
        telemetry.addLine("-----MINERAL Stop BAR-----");
        telemetry.addData("right Trigger 2",rightTrigger2);
        telemetry.addData("scoring bar position", robot.scoringStop.getPosition());
        telemetry.update();
    }
}

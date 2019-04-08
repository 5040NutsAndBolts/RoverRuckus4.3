package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="Teleop", group="Teleop")

public class Teleop extends OpMode {

    //sets up the objects for other classes
    private Hardware robot;
    private MineralScorer mineralScorer;
    private MecanumDrive driveTrain;
    private LiftMechanism lifter;
    private Collection collection;

    private Controllers controllers = new Controllers();

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
        robot.init(hardwareMap);
        //gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
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
        robot.teamMarker.setPosition(0.27);
    }

    /**
     * loops while in the play phase
     */
    public void loop() {

        double rightStickX1 = controllers.right_stick_x1;
        double leftStickX1  = controllers.left_stick_x1;
        double leftStickY1  = controllers.left_stick_y1;

        controllers.update(gamepad1, gamepad2);

        //the scoring method calls
        mineralScorer.slide(controllers.x_button2);
        mineralScorer.mineralBar(controllers.right_trigger2);
        mineralScorer.bop(controllers.y_button2);
        mineralScorer.reset(controllers.y_button3);

        //the collection method calls
        collection.wrist(controllers.left_trigger2);
        collection.inTake(controllers.right_bumper2, controllers.left_bumper2);
        collection.slide(controllers.right_bumper1, controllers.left_bumper1);
        collection.reset(controllers.left_bumper3);

        //the lift call
        lifter.lift(controllers.left_trigger1, controllers.ddown1);
        lifter.reset(controllers.right_bumper3);

        //slows down the driving when the scoring slide is up
        if(robot.scoringSlide.getCurrentPosition() > 200) {
            rightStickX1 /= 2;
            leftStickX1  /= 2;
            leftStickY1  /= 2;
        }

        if(rightStickX1 == 0) {
            if(controllers.dleft1) {
                rightStickX1 = -0.3;
            } else if(controllers.dright1) {
                rightStickX1 = 0.3;
            }
        }
        driveTrain.orientedDrive(leftStickY1, leftStickX1, rightStickX1, controllers.x_button1);


        //telemetry lines
        telemetry.addLine("--------IMU--------");
        telemetry.addData("imu first angle", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("imu heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("imu system calibrated", robot.imu.isSystemCalibrated());
        telemetry.addData("imu calibrated?", robot.imu.isGyroCalibrated());
        telemetry.addLine("-------DRIVE MOTORS-------");
        telemetry.addData("front left drive", robot.leftDriveFront.getPower());
        telemetry.addData("rear left drive", robot.leftDriveRear.getPower());
        telemetry.addData("front right drive", robot.rightDriveFront.getPower());
        telemetry.addData("rear right drive", robot.rightDriveRear.getPower());
        telemetry.addLine("--------HANGING MOTOR--------");
        telemetry.addData("hanging motor position", robot.hangingMotor.getCurrentPosition());
        telemetry.addData("hanging motor set position", robot.hangingMotor.getTargetPosition());
        telemetry.addData("hanging motor power", robot.hangingMotor.getPower());
        telemetry.addLine("--------COLLECTION WRIST--------");
        telemetry.addData("wrist Position", robot.wrist.getCurrentPosition());
        telemetry.addData("wrist Power", robot.wrist.getPower());
        telemetry.addLine("--------COLLECTION SLIDE--------");
        telemetry.addData("collection slide Position", robot.collectionSlide.getCurrentPosition());
        telemetry.addData("collection slide power", robot.collectionSlide.getPower());
        telemetry.addLine("-----SCORING SLIDE-----");
        telemetry.addData("scoring slide position", robot.scoringSlide.getCurrentPosition());
        telemetry.addData("scoring slide power", robot.scoringSlide.getPower());
        telemetry.addLine("-----MINERAL DROP BAR-----");
        telemetry.addData(" scoring bar position", robot.blockingBar.getPosition());
        telemetry.update();
    }
}

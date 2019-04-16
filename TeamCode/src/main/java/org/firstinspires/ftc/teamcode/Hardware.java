package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 */
public class Hardware {

    public static final String MESSAGETAG = "5040MSG";

    HardwareMap hwMap;

    //scoring mechanism
    public DcMotor scoringSlide = null;
    public Servo scoringStop = null;

    //drive train motors
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;
    public DcMotor leftDriveRear = null;
    public DcMotor rightDriveRear = null;


    public DcMotor hang = null;

    //intake mechanism
    public  DcMotor intake = null;
    public DcMotor collectionSlide = null;
    public Servo wristLeft = null;
    public Servo wristRight = null;
    public Servo intakeStop = null;
    public ColorSensor intakeDetector = null;

    //gyro
    public BNO055IMU imu;

    //export data
    public FileHelper exportData = null;

    /**
     * Constructor to set up the Hardwaremap
     */
    public Hardware() {
        hwMap = null;
    }

    /**
     * Method for initializing all the hardware components.
     * Use at the beginning of code initialization
     * @param ahwMap the hardware declaration being passed into this class
     */
    public void init(HardwareMap ahwMap, boolean resetMotors) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //scoring mechanism
        scoringSlide = hwMap.dcMotor.get("scoringSlide");
        scoringSlide.setDirection(DcMotor.Direction.REVERSE);
        scoringSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scoringStop = hwMap.servo.get("scoringStop");


        //hanging mechanism
        hang = hwMap.dcMotor.get("hang");
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(20);


        //drive train motor setup
        leftDriveFront = hwMap.dcMotor.get("leftDriveFront");
        leftDriveRear = hwMap.dcMotor.get("leftDriveRear");
        rightDriveFront = hwMap.dcMotor.get("rightDriveFront");
        rightDriveRear = hwMap.dcMotor.get("rightDriveRear");
        //reversing the right side motors
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);


        //collection mechanism
        collectionSlide = hwMap.dcMotor.get("collectionSlide");
        collectionSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //wrist
        wristLeft = hwMap.servo.get("wristLeft");
        wristRight = hwMap.servo.get("wristRight");
        wristLeft.setDirection(Servo.Direction.REVERSE);

        intakeDetector = hwMap.get(ColorSensor.class, "intakeDetector");

        //intake
        intake = hwMap.dcMotor.get("intake");
        intakeStop = hwMap.servo.get("intakeStop");

        if(resetMotors) {
            resetMotors();
        }
    }

    public void resetMotors(){
        scoringSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoringSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        collectionSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

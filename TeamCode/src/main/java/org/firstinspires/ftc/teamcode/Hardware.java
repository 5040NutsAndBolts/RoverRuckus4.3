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
 *This class is for setting up all the hardware components of the robot.
 *This will have all the sensors, motors and servos declarations.
 *It will also be used to initialize everything for autonomous
 */
public class Hardware {

    public static final String MESSAGETAG = "5040MSG";

    HardwareMap hwMap;

    //scoring mechanism
    public DcMotor scoringSlide = null;
    public Servo blockingBar = null;
    public Servo bopper = null;

    //drive train motors
    public DcMotor leftDriveFront = null;
    public DcMotor rightDriveFront = null;
    public DcMotor leftDriveRear = null;
    public DcMotor rightDriveRear = null;


    public DcMotor hangingMotor = null;

    //intake mechanism
    public CRServo intake = null;
    public DcMotor collectionSlide = null;
    public DcMotor wrist = null;

    public Servo teamMarker = null;

    //gyro
    public BNO055IMU imu;

    /**
     * Constructor to set up the Hardwaremap
     */
    public Hardware() {
        hwMap = null;
    }

    /**
     *Method for initializing all the hardware components.
     *Use at the beginning of code initialization
     * @param ahwMap the hardware declaration being passed into this class
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        //scoring mechanism
        scoringSlide = hwMap.dcMotor.get("scoringSlide");
        scoringSlide.setDirection(DcMotor.Direction.REVERSE);
        scoringSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoringSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        blockingBar = hwMap.servo.get("blockingBar");
        bopper = hwMap.servo.get("bopper");


        //hanging mechanism
        hangingMotor = hwMap.dcMotor.get("hangingMotor");
        hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        collectionSlide.setDirection(DcMotor.Direction.REVERSE);
        collectionSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wrist = hwMap.dcMotor.get("wrist");
        wrist.setDirection(DcMotor.Direction.REVERSE);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake = hwMap.crservo.get("intake");


        teamMarker = hwMap.servo.get("teamMarker");
    }
}

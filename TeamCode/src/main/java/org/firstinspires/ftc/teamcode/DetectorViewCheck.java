package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DetectorViewCheck", group="Auto")
public class DetectorViewCheck extends AutoMethods {

    private Hardware robot;
    private MecanumDrive driveTrain;
    private LiftMechanism lifter;
    private ElapsedTime wait;
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);
        lifter = new LiftMechanism(robot);
        wait = new ElapsedTime();

        double power;
        int goldPos = 3;

        robot.init(hardwareMap);

        robot.teamMarker.setPosition(0);

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        dogeCVSetup(detector);

        telemetry.addLine("ready");
        telemetry.update();
        detector.enable();

        while (!isStarted()) {
            telemetry.addData("imu calibration", robot.imu.isGyroCalibrated());
            telemetry.update();
        }
        while(opModeIsActive()) {
            telemetry.addData("detector aligned", detector.getAligned());
            telemetry.addData("goldPos",goldPos);
            telemetry.addData("x pos of gold", detector.getXPosition());
            telemetry.addData("hanging slide power", robot.hangingMotor.getPower());
            telemetry.addData("hanging motor pos",robot.hangingMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

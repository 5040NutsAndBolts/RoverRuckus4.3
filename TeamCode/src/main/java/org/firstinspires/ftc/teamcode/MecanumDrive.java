package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.IOException;

import static java.lang.Math.abs;

/**
 * Class is for the mecanum drive code
 */
public class MecanumDrive {

    Hardware robot;
    public double adjust = 0;
    private boolean stop = false;

    /**
     * sets up the hardware refernce so you don't have to pass it as a parameter and sets the adjust
     * @param r - r is the hardware reference from the code
     */
    MecanumDrive(Hardware r) { robot = r; }

    /**
     * this method is for driving the mecanum with the three inputs
     * @param forward - the forward value input
     * @param sideways - the sideways value input
     * @param rotation - the rotation value input
     */
    public void drive(double forward, double sideways, double rotation) {

        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if(scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        robot.leftDriveFront.setPower(forward-rotation-sideways);
        robot.leftDriveRear.setPower(forward-rotation+sideways);
        robot.rightDriveFront.setPower(forward+rotation+sideways);
        robot.rightDriveRear.setPower(forward+rotation-sideways);
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }


    public void orientedDrive(double forward, double sideways, double rotation, boolean reset) {

        double P = Math.hypot(sideways, forward);
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double robotAngle = Math.atan2(forward, -sideways);

        if(reset) {
            adjust = angles.firstAngle;
        }
        if(robot.collectionSlide.getCurrentPosition() > 60) {
            rotation /= 2.5;
        } else {
            rotation /= 2;
        }

        /*if(!stop && forward==0 && sideways==0 && rotation==0){
            forwardInch(0);
            powerSet(0.2);
            stop = true;
        }
        else if(stop &&(forward!=0 || sideways!=0 || rotation!=0)) {
            robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stop = false;
        }
        else if(!stop) {*/
            double v5 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
            double v6 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;
            double v7 = P * Math.sin(robotAngle - angles.firstAngle + adjust) - P * Math.cos(robotAngle - angles.firstAngle + adjust) - rotation;
            double v8 = P * Math.sin(robotAngle - angles.firstAngle + adjust) + P * Math.cos(robotAngle - angles.firstAngle + adjust) + rotation;

            robot.leftDriveFront.setPower(v5);
            robot.rightDriveFront.setPower(v6);
            robot.leftDriveRear.setPower(v7);
            robot.rightDriveRear.setPower(v8);
        //}
    }

    /**
     *sets power of motors
     * @param power the power the robot is set to 0-1
     */
    public void powerSet(double power) {
        robot.leftDriveFront.setPower(power);
        robot.leftDriveRear.setPower(power);
        robot.rightDriveFront.setPower(power);
        robot.rightDriveRear.setPower(power);
    }

    /**
     * MT is Motor Ticks
     * 1 inch forward = 87 MT
     * @param fInches inches forward
     */
    public void forwardInch(int fInches){
        int fPos = fInches*43;

        resetMotors();

        robot.leftDriveFront.setTargetPosition(fPos);
        robot.leftDriveRear.setTargetPosition(fPos);
        robot.rightDriveFront.setTargetPosition(fPos);
        robot.rightDriveRear.setTargetPosition(fPos);
    }

    /**
     * MT is Motor Ticks
     * 1 inch sideways left = 129 MT
     * @param sInches inches sideways positive is to the left
     */
    public void sidewaysInch(int sInches) {
        int sPos = sInches*64;

        resetMotors();

        robot.leftDriveFront.setTargetPosition(-sPos);
        robot.leftDriveRear.setTargetPosition(sPos);
        robot.rightDriveFront.setTargetPosition(sPos);
        robot.rightDriveRear.setTargetPosition(-sPos);
    }

    /**
     * rotates robot certain amount of degrees
     * @param degrees degrees to turn
     */
    public void rotate(int degrees) {
        int rDegrees = degrees*9;

        resetMotors();

        robot.leftDriveFront.setTargetPosition(rDegrees);
        robot.leftDriveRear.setTargetPosition(rDegrees);
        robot.rightDriveFront.setTargetPosition(-rDegrees);
        robot.rightDriveRear.setTargetPosition(-rDegrees);
    }

    /**
     * Resets all motor positions back to 0
     */
    public void resetMotors() {
        robot.leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDriveRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

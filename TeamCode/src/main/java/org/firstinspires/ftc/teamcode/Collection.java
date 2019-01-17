package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class for the collection Mechanism code
 */
public class Collection {

    private Hardware robot;

    private boolean wristReset = false;
    private boolean wristToggle = false;   //toggle for the wrist
    private boolean wristDown = true;      //when true and slide is out the wrist will be down

    /**
     * sets up the hardware so you don't have to pass it as a parameter
     * @param r - Hardware reference that needs to get passed through
     */
    public Collection(Hardware r) {
        robot = r;
    }

    /**
     * control for the wrist motor
     * @param toggle - when true it will move it up if down or vice-versa
     */
    public void wrist(boolean toggle) {

        //set the wrist up when slide is in
        if(robot.collectionSlide.getCurrentPosition()<50) {
            robot.wrist.setTargetPosition(10);

            //throws wrist up so it puts minerals into scoring bucket
            if(robot.wrist.getCurrentPosition() >30) {
                robot.wrist.setPower(1);
            }else {
                if (wristReset == true) {
                    robot.wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    wristReset = false;
                }
                robot.wrist.setPower(0.3);
            }
            //will set the wrist all the way down when the slide is extended out
            wristDown = true;
        }
        //sets wrist part way up
        else if(!wristDown) {
            robot.wrist.setPower(0.5);
            robot.wrist.setTargetPosition(450);
            wristReset = true;
        }
        //sets wrist all the way down
        else {
            robot.wrist.setTargetPosition(680);
            robot.wrist.setPower(0.3);
            wristReset = true;
        }
        //toggle for the wrist when toggle is true
        if(toggle && !wristToggle && robot.collectionSlide.getCurrentPosition()>50) {
            wristToggle = true;

            if(robot.wrist.getCurrentPosition() > 500) {
                wristDown = false;
            }
            else {
                wristDown = true;
            }
        }
        else if(!toggle) {
            wristToggle = false;
        }
    }

    /**
     * spins the intake vex motor either in or out
     * @param in - when true will pull in minerals
     * @param out - when true will spit out minerals
     */
    public void inTake(boolean in, boolean out) {
        if(in) {
            robot.intake.setPower(-0.5);
        }
        else if(out) {
            robot.intake.setPower(0.5);
        }
        else{
            robot.intake.setPower(0);
        }
    }

    /**
     * controls the collectionSlide motor
     * @param in - when true will pull in slide
     * @param out - when true will push out slide
     */
    public  void slide(boolean in, boolean out) {
        if(out) {
            robot.collectionSlide.setPower(1);
            robot.collectionSlide.setTargetPosition(1220);
        }
        else if(in) {
            robot.collectionSlide.setPower(1);
            if(robot.collectionSlide.getCurrentPosition() > 10) {
                robot.collectionSlide.setTargetPosition(-200);
            }
            else {
                robot.collectionSlide.setTargetPosition(-50);
            }
        }
        else {
            robot.collectionSlide.setPower(0);
            robot.collectionSlide.setTargetPosition(robot.collectionSlide.getCurrentPosition());
        }
    }
}
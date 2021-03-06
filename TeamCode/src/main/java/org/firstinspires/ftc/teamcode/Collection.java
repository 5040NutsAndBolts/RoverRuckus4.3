package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for the collection Mechanism code
 */
public class Collection {

    private Hardware robot;

    private boolean wristReset = false;
    private boolean wristToggle = false;   //toggle for the wrist
    private boolean resetToggle = false;

    public boolean collect = false;
    public double wristDownPos = 0.16;
    public double wristUpPos = 0.84;
    public double wristPos = wristUpPos;
    public boolean canTransfer = false;


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
        /*if(robot.collectionSlide.getCurrentPosition() > 500) {
            robot.scoringSlide.setTargetPosition(0);
            wristPos=wristDownPos;
            collect = true;
        }*/
        if(toggle && !wristToggle) {
            wristToggle = true;

            if(robot.wristRight.getPosition() == wristDownPos)
            {
                wristPos=wristUpPos;
            }
            else {
                robot.scoringSlide.setTargetPosition(0);
                wristPos=wristDownPos;
                collect = true;
            }
        }
        else if(!toggle && wristToggle) {
            wristToggle = false;
        }

        /*if(wristPos==wristUpPos && robot.collectionSlide.getCurrentPosition() > 100) {
            wristPos = 0.5;
        }
        else if(wristPos==0.5 && robot.collectionSlide.getCurrentPosition() < 100) {
            wristPos=wristUpPos;
        }*/
    }

    public void wristSetPosition(double pos) {
        wristPos = pos;
    }

    /**
     * spins the intake vex motor either in or out
     * @param in - when true will pull in minerals
     * @param out - when true will spit out minerals
     */
    public void inTake(boolean in, boolean out) {

        if(robot.scoringSlide.getPower() == 1) {
            canTransfer = false;
            robot.intakeStop.setPosition(0.02);
        }
        if(out) {
            robot.intake.setPower(0.7);
        }
        else if(robot.intakeDetector.alpha() > 200 && robot.scoringSlide.getPower() != 1 &&
                robot.wristRight.getPosition() == wristUpPos && canTransfer){
            robot.intake.setPower(-0.8);
            robot.intakeStop.setPosition(0.37);
            //slide(true,false);
            collect = false;
        }
        else if(collect) {
            robot.intake.setPower(-1);
            robot.intakeStop.setPosition(0.02);
            canTransfer = true;
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
            robot.collectionSlide.setPower(0.5);
            robot.collectionSlide.setTargetPosition(930);
            //if(wristDown)
               // wristSetPosition(0.3);
        }
        else if(in) {
            robot.collectionSlide.setPower(0.5);
            robot.collectionSlide.setTargetPosition(-10);
        }
        else {
            robot.collectionSlide.setPower(0.8);
            robot.collectionSlide.setTargetPosition(robot.collectionSlide.getCurrentPosition());
        }
    }

    public void inTakeStop(boolean stop){
        /*if(robot.scoringSlide.getCurrentPosition() > 100 || robot.hang.getPower() == 1) {
            canTransfer = false;
        }
        if(robot.wristLeft.getPosition() == wristDownPos) {
            canTransfer = true;
        }
        if(robot.intakeDetector.alpha() > 120 && canTransfer) {
            robot.intakeStop.setPosition(0.37);
            slide(true,false);
        }
        else if(robot.intakeDetector.alpha() < 120) {
            collect = true;
            robot.intakeStop.setPosition(0.02);
        }*/

        /*if(robot.scoringSlide.getCurrentPosition()<10 && robot.collectionSlide.getCurrentPosition()<20 &&
                (robot.wristLeft.getPosition() == wristUpPos) && stop) {
            robot.intakeStop.setPosition(0.37);
            collect = false;
            slide(true,false);
        }
        else
            robot.intakeStop.setPosition(0.02);*/
    }

    /**
     * method for resetting the slide back in
     * @param resetting - when true it will start moving slide in
     */
    public void reset(boolean resetting) {
        if(resetting){
            resetToggle = true;
            robot.collectionSlide.setPower(.5);
            robot.collectionSlide.setTargetPosition(-1000);
        }
        else if (resetToggle) {
            resetToggle = false;
            robot.collectionSlide.setPower(.8);
            robot.collectionSlide.setTargetPosition(robot.scoringSlide.getCurrentPosition());
            robot.collectionSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.collectionSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}
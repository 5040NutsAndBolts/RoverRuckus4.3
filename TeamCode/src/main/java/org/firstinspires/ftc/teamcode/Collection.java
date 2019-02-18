package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class for the collection Mechanism code
 */
public class Collection {

    private Hardware robot;

    private boolean wristReset = false;
    private boolean wristToggle = false;   //toggle for the wrist
    private boolean wristDown = false;      //when true and slide is out the wrist will be down

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

    public double wristPosition=0;

    public void wrist(boolean toggle)
    {

        robot.wristRight.setPosition(wristPosition);
        robot.wristLeft.setPosition(wristPosition);
        if(!wristDown){

            wristPosition=0.9;

        }else {

            wristPosition=0.25+(.00004*robot.collectionSlide.getCurrentPosition());

        }
        if(toggle && !wristToggle)
        {
            wristToggle = true;

            if(wristDown)
            {
                wristDown = false;
            }
            else {
                wristDown = true;
            }
        }
        else if(!toggle && wristToggle)
        {
            wristToggle = false;
        }
    }

    /**
     * spins the intake vex motor either in or out
     * @param in - when true will pull in minerals
     * @param out - when true will spit out minerals
     */
    public void inTake(boolean in, boolean out) {
        if(in || robot.intakeStop.getPosition()==0.37) {
            robot.intake.setPower(-1);
        }
        else if(out) {
            robot.intake.setPower(0.7);
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
            robot.collectionSlide.setPower(0.7);
            robot.collectionSlide.setTargetPosition(1000);
            if(wristDown)
                wristSetPosition(0.3);
        }
        else if(in) {
            robot.collectionSlide.setPower(0.7);
            robot.collectionSlide.setTargetPosition(-10);
        }
        else {
            robot.collectionSlide.setPower(0.3);
            robot.collectionSlide.setTargetPosition(robot.collectionSlide.getCurrentPosition());
        }
    }

    public void wristSetPosition(double pos) {
        robot.wristRight.setPosition(pos);
        robot.wristLeft.setPosition(pos);
    }

    public boolean stopToggle = false;
    public void inTakeStop(boolean stop){
        if(robot.scoringSlide.getCurrentPosition()<10 && robot.collectionSlide.getCurrentPosition()<20 && robot.wristLeft.getPosition()==0.9 && stop)
            robot.intakeStop.setPosition(0.37);
        else
            robot.intakeStop.setPosition(0);
        /*if(stop && !stopToggle){
            if(robot.intakeStop.getPosition() == 0)
                robot.intakeStop.setPosition(0.25);
            else
                robot.intakeStop.setPosition(0);
            stopToggle = true;
        }
        else if(!stop && stopToggle) {
            stopToggle = false;
        }*/
    }

}
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

    public double wristPosition=0;

    public void wrist(boolean toggle)
    {

        robot.wristRight.setPosition(wristPosition);
        robot.wristLeft.setPosition(wristPosition);
        if(robot.collectionSlide.getCurrentPosition()<50)
        {

            wristPosition=0;
            wristDown = true;

        } else if(!wristDown){

            wristPosition=.4;

        }else {

            wristPosition=1;

        }
        if(toggle && !wristToggle && robot.collectionSlide.getCurrentPosition()>50)
        {
            wristToggle = true;

            wristDown = !(robot.wristLeft.getPosition() > .3);
        }
        else if(!toggle)
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
        if(in) {
            robot.intake.setPower(-1);
        }
        else if(out) {
            robot.intake.setPower(1);
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
                robot.collectionSlide.setTargetPosition(-1000);
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
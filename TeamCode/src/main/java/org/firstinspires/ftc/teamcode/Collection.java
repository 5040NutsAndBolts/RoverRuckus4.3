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
    public boolean wristDown = false;      //when true and slide is out the wrist will be down
    public double wristDownPos = 0.2;
    public ElapsedTime time = new ElapsedTime();
    public double wristPos = 0.75;


    /**
     * sets up the hardware so you don't have to pass it as a parameter
     * @param r - Hardware reference that needs to get passed through
     */
    public Collection(Hardware r) {
        //t1.start();
        //t2.start();
        robot = r;
    }

    /**
     * control for the wrist motor
     * @param toggle - when true it will move it up if down or vice-versa
     */


    public void wrist(boolean toggle)
    {

        wristSetPosition(wristPos);
        if(!wristDown){

            wristPos=0.75;

        }else {
            robot.scoringSlide.setTargetPosition(0);
            wristPos=wristDownPos+(.00004*robot.collectionSlide.getCurrentPosition());

        }
        if(toggle && !wristToggle)
        {
            time.reset();
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

    public void wristSetPosition(double pos) {
        wristPos = pos;
    }

    /**
     * spins the intake vex motor either in or out
     * @param in - when true will pull in minerals
     * @param out - when true will spit out minerals
     */
    public void inTake(boolean in, boolean out) {
        if(robot.intakeStop.getPosition()==0.37) {
            robot.intake.setPower(-0.8);
        }
        else if(out) {
            robot.intake.setPower(0.7);
        }
        else if(in || wristDown) {
            robot.intake.setPower(-1);
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

    public boolean stopToggle = false;
    public void inTakeStop(boolean stop){
        if(robot.scoringSlide.getCurrentPosition()<10 && robot.collectionSlide.getCurrentPosition()<20 && !wristDown && stop)
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
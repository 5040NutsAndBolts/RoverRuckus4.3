package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

/**
 * Class for the scoring mechanism
 */
public class MineralScorer {

    Hardware robot;

    private boolean scoringToggle = false;     //toggle for the scoring slide
    private  boolean barToggle = false;        //toggle for the scoring bar
    private double slidePower = 0.3;
    private Collection collection;

    MineralScorer(Hardware r) {
        robot = r;
        collection = new Collection(robot);
    }

    /**
     * this method extends the arm on the back of the robot up to the lander
     * @param toggle - when true it will move it up if down or vice-versa
     */
    public void slide(boolean toggle){
        //toggle for the scoring slide
        if(toggle && !scoringToggle) {
            scoringToggle = true;

            if(robot.scoringSlide.getTargetPosition()==0) {
                robot.scoringSlide.setTargetPosition(950);
            }
            else {
                robot.scoringSlide.setTargetPosition(0);
            }
        }
        else if(!toggle) {
            scoringToggle = false;
        }
        if(collection.wristDown);
            //robot.scoringSlide.setTargetPosition(0);
        //if the scoring slide motor is down and it is supposed to be down it sets the power to 0
        //if(robot.scoringSlide.getCurrentPosition() <= 20 && robot.scoringSlide.getTargetPosition() == 0) {
         //   robot.scoringSlide.setPower(0.3);
        if(abs(robot.scoringSlide.getCurrentPosition()-robot.scoringSlide.getTargetPosition()) < 50) {
            slidePower=0.5;
        }
        else
            slidePower = 1;
        robot.scoringSlide.setPower(slidePower);
    }

    /**
     *controls the scoring bucket bar to drop the minerals
     * the bar goes all the way down when the slide is down
     * @param open - toggle for moving the bar
     */
    public void mineralStop(boolean open) {
        if(robot.scoringSlide.getCurrentPosition()<100)
            robot.scoringStop.setPosition(0);
        else if(open && !barToggle) {
            if(robot.scoringStop.getPosition() == 0.5){
                robot.scoringStop.setPosition(0);
            }
            else {
                robot.scoringStop.setPosition(0.5);
            }
            barToggle = true;
        }
        else if(!open && barToggle) {
            barToggle = false;
        }
    }
}
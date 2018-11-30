package org.firstinspires.ftc.teamcode;

/**
 * Class for the scoring mechanism
 */
public class MineralScorer {

    Hardware robot;

    private boolean scoringToggle = false;     //toggle for the scoring slide
    private  boolean barToggle = false;        //toggle for the scoring bar
    private int barPlace = 0;                  //keeps track of what place the scoring bar is in

    MineralScorer(Hardware r) {
        robot = r;
    }

    /**
     * this method extends the arm on the back of the robot up to the lander
     * @param toggle - when true it will move it up if down or vice-versa
     */
    public void slide(boolean toggle){
        //toggle for the scoring slide
        if(toggle && !scoringToggle) {
            scoringToggle = true;

            if(robot.scoringSlide.getCurrentPosition() < 80) {
                robot.scoringSlide.setTargetPosition(1700);
                robot.scoringSlide.setPower(1);
            }
            else {
                robot.scoringSlide.setTargetPosition(0);
                robot.scoringSlide.setPower(0.8);
            }
        }
        else if(!toggle) {
            scoringToggle = false;
        }
        //if the scoring slide motor is down and it is supposed to be down it sets the power to 0
        if(robot.scoringSlide.getCurrentPosition() <= 20 && robot.scoringSlide.getTargetPosition() == 0) {
            robot.scoringSlide.setPower(0);
        }
    }

    /**
     * control for the servo that is right before the scoring bucket
     * @param hit - When true it goes out and hits the mineral over.
     */
    public void bop(boolean hit) {
        if(hit) {
            robot.bopper.setPosition(0.16);
        }
        else {
            robot.bopper.setPosition(0.04);
        }
    }

    /**
     *controls the scoring bucket bar to drop the minerals
     * the bar goes all the way down when the slide is down
     * @param open - toggle for moving the bar
     */
    public void mineralBar(boolean open) {
        //puts the bar to drop gold
        if(open && barPlace==0 && !barToggle) {
            barToggle = true;
            barPlace = 1;
            robot.blockingBar.setPosition(0.4);
        }
        //puts the bar to drop silver and gold
        else if(open && barPlace==1 && !barToggle) {
            barToggle = true;
            barPlace = 2;
            robot.blockingBar.setPosition(1);
        }
        //if the bar is in the second position or the slide is down it puts the bar all the way down
        else if((open && barPlace==2 && !barToggle)
                || robot.scoringSlide.getCurrentPosition() <= 200) {
            barToggle = true;
            barPlace = 0;
            robot.blockingBar.setPosition(0);
        }
        else if(!open && barToggle) {
            barToggle = false;
        }
    }
}
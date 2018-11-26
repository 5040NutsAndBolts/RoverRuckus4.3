package org.firstinspires.ftc.teamcode;

/**
 * Class for scoring minerals in the lander
 */
public class MineralScorer {

    Hardware robot;

    private boolean scoringToggle = false;
    private  boolean barToggle = false;
    private int barPlace = 0;

    MineralScorer(Hardware r) {
        robot = r;
    }

    /**
     * this method extends the arm on the back of the robot out to the lander
     * @param //stick
     */
    public void slide(boolean toggle){

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
        if(robot.scoringSlide.getCurrentPosition() <= 20 && robot.scoringSlide.getTargetPosition() == 0) {
            robot.scoringSlide.setPower(0);
        }
    }

    public void bop(boolean hit) {
        if(hit) {
            robot.bopper.setPosition(0.16);
        }
        else {
            robot.bopper.setPosition(0.04);
        }
    }

    /**
     * TEMPORARY UNTIL WE CAN GET COLOR SENSORS WORKING
     * @param open
     */
    public void mineralBar(boolean open) {

        if(open && barPlace==0 && !barToggle) {
            barToggle = true;
            barPlace = 1;
            robot.blockingBar.setPosition(0.4);
        }
        else if(open && barPlace==1 && !barToggle) {
            barToggle = true;
            barPlace = 2;
            robot.blockingBar.setPosition(1);
        }
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
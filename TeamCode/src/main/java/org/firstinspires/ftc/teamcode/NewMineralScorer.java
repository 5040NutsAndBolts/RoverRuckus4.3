package org.firstinspires.ftc.teamcode;

/**
 * New class for the scoring mechanism
 */
public class NewMineralScorer {
    private Hardware robot;

    private boolean scoringToggle = false;
    private boolean blockToggle = false;
    private int slideTargetPos = 1700;
    private int blockBarsOpen = 0;

    public NewMineralScorer(Hardware r){ robot = r; }

    /**
     * Method to move the mineral scoring slide up or down
     * @param toggle - When true moves slide up if down, when false moves slide down if up
     */
    public void slide(boolean toggle){
        if (toggle && !scoringToggle){
            scoringToggle = true;

            if(robot.scoringSlide.getCurrentPosition() < 80) {
                robot.scoringSlide.setTargetPosition(slideTargetPos);
                robot.scoringSlide.setPower(1);
            }
            else {
                robot.scoringSlide.setTargetPosition(0);
                robot.scoringSlide.setPower(0.8);
            }
        } else if (!toggle)
            scoringToggle = false;

        //if the scoring slide motor is down and it is supposed to be down it sets the power to 0
        if(robot.scoringSlide.getCurrentPosition() <= 20 && robot.scoringSlide.getTargetPosition() == 0) {
            robot.scoringSlide.setPower(0);
        }
    }

    /**
     * Method to block minerals from dropping too early
     * @param open - Toggle for moving the bars
     */
    public void mineralBars(boolean open) {
        /*
         There is also a pair of servos that block minerals from going out too early.
         Servo to block minerals from dropping moves when sensor sees lander while slide is up
         */

        //boolean slideUp = robot.scoringSlide.getCurrentPosition() >= slideTargetPos-100;

        // drops gold
        if (open && blockBarsOpen == 0 && !blockToggle) {
            blockToggle = true;
            blockBarsOpen = 1;
            robot.goldBlockBar.setPosition(1);
        // drops silver and gold
        } else if (open && blockBarsOpen == 1 && !blockToggle) {
            blockBarsToggle = true;
            blockBarsOpen = 2;
            robot.goldBlockBar.setPosition(1);
            robot.silverBlockBar.setPosition(1);
        // closes bars
        } else if ((open && blockBarsOpen == 2 && !blockToggle)
                || robot.scoringSlide.getCurrentPosition()<=200){
            blockToggle = true;
            blockBarsOpen = 0;
            robot.goldBlockBar.setPosition(0);
            robot.silverBlockBar.setPosition(0);
        } else if(!open && blockToggle) {
            blockToggle = false;
        }
    }
}

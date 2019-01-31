package org.firstinspires.ftc.teamcode;

/**
 * New class for the scoring mechanism
 */
public class MineralScorer {
    private Hardware robot;

    private boolean scoringToggle = false;
    private boolean blockToggle = false;
    private int slideTargetPos = 1700;
    private int blockBarsOpen = 0;

    public MineralScorer(Hardware r){ robot = r; }

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
     * Method to block minerals from dropping too early.
     * Uses pair of servos to block minerals from going out too early.
     * Servos block minerals from being able to fall until the slide is up and sensor sees lander
     * @param open - Toggle for moving the bars
     */
    public void mineralBars(boolean open) {
        /* Not to be used until the proper parts get put onto the robot
         boolean slideUp = robot.scoringSlide.getCurrentPosition() <= slideTargetPos-100;
         boolean landerInSight = TBD;
         boolean droppingSilver = TBD;

         if (slideUp && landerInSight){
            if (droppingSilver)
                robot.silverBlockBar.setPosition(1);
            robot.goldBlockBar.setPosition(1);
         } else {
            robot.goldBlockBar.setPosition(0);
            robot.silverBlockBar.setPosition(0);
         }
         */

        // drops gold
        if (open && blockBarsOpen == 0 && !blockToggle) {
            blockToggle = true;
            blockBarsOpen = 1;
            robot.goldBlockBar.setPosition(.5);
            // drops silver and gold
        } else if (open && blockBarsOpen == 1 && !blockToggle) {
            blockToggle = true;
            blockBarsOpen = 2;
            robot.goldBlockBar.setPosition(.5);
            robot.silverBlockBar.setPosition(.5);
            // closes bars
        } else if ((open && blockBarsOpen == 2 && !blockToggle)
                || robot.scoringSlide.getCurrentPosition() <= 200){
            blockToggle = true;
            blockBarsOpen = 0;
            robot.goldBlockBar.setPosition(0);
            robot.silverBlockBar.setPosition(0);
        } else if(!open && blockToggle) {
            blockToggle = false;
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
}

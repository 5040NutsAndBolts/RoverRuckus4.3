package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LiftMechanism {

    Hardware robot;

    private boolean liftToggle = false;   //toggle for the lift
    private boolean downReset = false;    //keeps track of if it should reset the hanging motor position

    LiftMechanism(Hardware r) {
        robot = r;
    }

    /**
     * method for controlling the motor for the lift mechanism
     * @param toggle - when true it will move it up if down or vice-versa
     * @param reset - moves slide down and resets position while true
     */
    public void lift(boolean toggle, boolean reset) {
        //toggle for the hang motor
        if(toggle && !liftToggle) {
            liftToggle = true;

            if(robot.hangingMotor.getCurrentPosition() < 40) {
                robot.hangingMotor.setTargetPosition(7200);
            }
            else {
                robot.hangingMotor.setTargetPosition(20);
            }
        }
        else if(!toggle) {
            liftToggle = false;
        }

        //if the slide is down and it is supposed to be down or vice versa for up then it sets the power to 0
        if((robot.hangingMotor.getCurrentPosition() <= 30 && robot.hangingMotor.getTargetPosition() == 20)  ||
                (robot.hangingMotor.getCurrentPosition() >= 6890 && robot.hangingMotor.getTargetPosition() == 6900)) {
            robot.hangingMotor.setPower(0);
        }
        else {
            robot.hangingMotor.setPower(1);
        }



        //reset for the hanging motor
        if(reset) {
            robot.hangingMotor.setPower(1);
            robot.hangingMotor.setTargetPosition(-7000);
            downReset = true;
        }
        else if(downReset) {
            robot.hangingMotor.setPower(0);
            robot.hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            downReset = false;
        }
    }

    /**
     * method for resetting the lift back to the bottom
     * @param resetting - when true it will start moving lift down
     */
    public void reset(boolean resetting){
        if(resetting){
            robot.hangingMotor.setPower(1);
            robot.hangingMotor.setTargetPosition(-7000);
        }
        else {
            robot.hangingMotor.setPower(0);
            robot.hangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}

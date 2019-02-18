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

            if(robot.hang.getTargetPosition() == 20)
                robot.hang.setTargetPosition(3100);
            else {
                robot.hang.setTargetPosition(20);
            }
        }
        else if(!toggle) {
            liftToggle = false;
        }

        //if the slide is down and it is supposed to be down or vice versa for up then it sets the power to 0
        if(robot.hang.getCurrentPosition() >= robot.hang.getTargetPosition()-50 &&
                        robot.hang.getCurrentPosition() <= robot.hang.getTargetPosition()+50) {
            robot.hang.setPower(0);
        }
        else {
            robot.hang.setPower(1);
        }



        //reset for the hanging motor
        if(reset) {
            robot.hang.setPower(1);
            robot.hang.setTargetPosition(-7000);
            downReset = true;
        }
        else if(downReset) {
            robot.hang.setPower(0);
            robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hang.setTargetPosition(20);
            downReset = false;
        }
    }
}

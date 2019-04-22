package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="StartX", group="Teleop")
@Disabled
public class StartXTest extends OpMode {
    /**
     * The instance fields: <code>gamepadMode1</code> and <code>gamepadMode2</code> determine which controller is using the alternative control scheme.
     */
    private int gamepadMode1 = 0, gamepadMode2 = 0;

    /**
     *  <code>isStartXPressable</code> is used for the toggle of start-x, it is used to make switching between the two
     *  modes easier to accomplish.
     */
    private boolean isStartXPressable = true;

    /**
     * sets up the objects for the other classes
     */


    /**
     * method runs once on init
     * init's the hardware of the robot
     */
    public void init() {

    }

    /**
     * runs once you hit start
     * drops the team marker if it didn't during auto
     */
    @Override
    public void start() {

    }

    /**
     * loops while in the play phase
 */
    public void loop() {
        /*
         *   Gamepad 1
         */
        //Joystick Inputs
        double left_stick_y1   = gamepadMode1 == 0 ? gamepad1.left_stick_y : 0; //Basicly an if statement that
        double right_stick_y1  = gamepadMode1 == 0 ? gamepad1.right_stick_y : 0;
        double left_stick_x1   = gamepad1.left_stick_x;
        double right_stick_x1  = gamepad1.right_stick_x;
        boolean stick_press1   = gamepad1.left_stick_button || gamepad1.right_stick_button;

        //Bumpers and Triggers
        boolean left_bumper1   = gamepad1.left_bumper  && gamepadMode1 == 0;
        boolean right_bumper1  = gamepad1.right_bumper && gamepadMode1 == 0;
        boolean left_trigger1  = gamepad1.left_trigger  > 0.3 && gamepadMode1 == 0;//unused
        boolean right_trigger1 = gamepad1.right_trigger > 0.3 && gamepadMode1 == 0;

        //Button inputs
        boolean b_button1 = gamepad1.b && gamepadMode1 == 0;
        boolean a_button1 = gamepad1.a && gamepadMode1 == 0;
        boolean y_button1 = gamepad1.y && gamepadMode1 == 0;
        boolean x_button1 = gamepad1.x && gamepadMode1 == 0;

        //Directional Pad Inputs
        boolean dup1    = gamepad1.dpad_up    && gamepadMode1 == 0;
        boolean ddown1  = gamepad1.dpad_down  && gamepadMode1 == 0;
        boolean dleft1  = gamepad1.dpad_left  && gamepadMode1 == 0;
        boolean dright1 = gamepad1.dpad_right && gamepadMode1 == 0;

        //Auxillary Inputs
        boolean left_stick_press1  = gamepad1.right_stick_button && gamepadMode1 == 0;
        boolean right_stick_press1 = gamepad1.left_stick_button  && gamepadMode1 == 0;
        boolean home   = gamepad1.guide && gamepadMode1 == 0;
        boolean back1  = gamepad1.back  && gamepadMode1 == 0;
        boolean start1 = gamepad1.start && gamepadMode1 == 0;

        /*
         *   Gamepad 2
         */
        //Joystick Inputs
        double left_stick_y2   = gamepadMode2 == 0 ? gamepad2.left_stick_y : 0;
        double right_stick_y_2 = gamepadMode2 == 0 ? gamepad2.right_stick_y : 0;
        double left_stick_x2   = gamepad2.left_stick_x;
        double right_stick_x2  = gamepad2.right_stick_x;

        //Bumpers and Triggers
        boolean left_bumper2   = gamepad2.left_bumper  && gamepadMode2 == 0;
        boolean right_bumper2  = gamepad2.right_bumper && gamepadMode2 == 0;
        boolean left_trigger2  = gamepad2.left_trigger > 0.3  && gamepadMode2 == 0;
        boolean right_trigger2 = gamepad2.right_trigger > 0.3 && gamepadMode2 == 0;

        //Button inputs
        boolean back2     = gamepad2.back && gamepadMode2 == 0;
        boolean b_button2 = gamepad2.b && gamepadMode2 == 0;
        boolean a_button2 = gamepad2.a && gamepadMode2 == 0;
        boolean y_button2 = gamepad2.y && gamepadMode2 == 0;
        boolean x_button2 = gamepad2.x && gamepadMode2 == 0;

        //Directional Pad Inputs
        boolean dup2    = gamepad2.dpad_up    && gamepadMode2 == 0;
        boolean ddown2  = gamepad2.dpad_down  && gamepadMode2 == 0;
        boolean dleft2  = gamepad2.dpad_left  && gamepadMode2 == 0;
        boolean dright2 = gamepad2.dpad_right && gamepadMode2 == 0;

        //Auxillary Inputs
        boolean left_stick_press2  = gamepad2.right_stick_button && gamepadMode2 == 0;
        boolean right_stick_press2 = gamepad2.left_stick_button  && gamepadMode2 == 0;
        boolean home2  = gamepad2.guide  && gamepadMode2 == 0;
        boolean start2 = gamepad2.start && gamepadMode2 == 0;

        /*
         *   Gamepad 3
         */
        //Joystick Inputs
        double left_stick_y3   = (gamepad2.left_stick_y * gamepadMode2) + (gamepad1.left_stick_y * gamepadMode1);
        double right_stick_y3  = (gamepad2.right_stick_y * gamepadMode2) + (gamepad1.right_stick_y * gamepadMode1);
        double left_stick_x3   = (gamepad2.left_stick_x * gamepadMode2) + (gamepad1.left_stick_x * gamepadMode1);
        double right_stick_x3  = (gamepad2.right_stick_x * gamepadMode2) + (gamepad1.right_stick_x * gamepadMode1);
        boolean right_trigger3 = (gamepad2.right_trigger > 0.3 && gamepadMode2 == 1) || (gamepad1.right_trigger > 0.3 && gamepadMode1 == 1);
        boolean left_trigger3  = (gamepad2.left_trigger  > 0.3 && gamepadMode2 == 1) || (gamepad1.left_trigger > 0.3 && gamepadMode1 == 1);
        boolean right_bumper3  = (gamepad2.right_bumper && gamepadMode2 == 1) || (gamepad1.right_bumper && gamepadMode1 == 1);
        boolean left_bumper3   = (gamepad2.left_bumper  && gamepadMode2 == 1) || (gamepad1.left_bumper  && gamepadMode1 == 1);

        //Button inputs
        boolean b_button3 = (gamepad2.b && gamepadMode2 == 1) || (gamepad1.b && gamepadMode1 == 1);
        boolean a_button3 = (gamepad2.a && gamepadMode2 == 1) || (gamepad1.a && gamepadMode1 == 1);
        boolean y_button3 = (gamepad2.y && gamepadMode2 == 1) || (gamepad1.y && gamepadMode1 == 1);
        boolean x_button3 = (gamepad2.x && gamepadMode2 == 1) || (gamepad1.x && gamepadMode1 == 1);

        //Directional Pad Inputs
        boolean dup3    = (gamepad2.dpad_up    && gamepadMode2 == 1) || (gamepad1.dpad_up    && gamepadMode1 == 1);
        boolean ddown3  = (gamepad2.dpad_down  && gamepadMode2 == 1) || (gamepad1.dpad_down  && gamepadMode1 == 1);
        boolean dleft3  = (gamepad2.dpad_left  && gamepadMode2 == 1) || (gamepad1.dpad_left  && gamepadMode1 == 1);
        boolean dright3 = (gamepad2.dpad_right && gamepadMode2 == 1) || (gamepad1.dpad_right && gamepadMode1 == 1);

        //Auxillary Inputs
        boolean right_stick_press3  = (gamepad2.right_stick_button && gamepadMode2 == 1) || (gamepad1.right_stick_button && gamepadMode1 == 1);
        boolean left_stick_press3   = (gamepad2.left_stick_button  && gamepadMode2 == 1)  || (gamepad1.left_stick_button  && gamepadMode1 == 1);
        boolean home3  = (gamepad2.guide && gamepadMode2 == 1);
        boolean start3 = (gamepad1.start && gamepadMode1 == 1) || (gamepad2.start && gamepadMode2 == 1);

                /*
            Switch the controller mode
            mapped to Start-x
         */
        if (x_button1 && start1 && isStartXPressable) {
            gamepadMode1 = 1;
            gamepadMode2 = 0;
            isStartXPressable = false;
        }else if (x_button2 && start2 && isStartXPressable) {
            gamepadMode1 = 0;
            gamepadMode2 = 1;
            isStartXPressable = false;
        }else if (x_button3 && start3 && isStartXPressable) {
            gamepadMode1 = 0;
            gamepadMode2 = 0;
            isStartXPressable = false;
        }
        isStartXPressable = !((x_button1 && start1) || (x_button2 && start2) || (x_button3 && start3));

        telemetry.addLine("-----START X TESTING -----");
//Controller 1
        if (gamepadMode1 == 0) {
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine("CONTROLLER 1");
            telemetry.addData("Left Joystick (x, y)", left_stick_x1 + ", " + left_stick_y1);
            telemetry.addData("Right Joystick (x, y)", right_stick_x1 + ", " + right_stick_y1);
            telemetry.addData("Left Trigger", left_trigger1);
            telemetry.addData("Right Trigger", right_trigger1);
            telemetry.addData("Left Bumper", left_bumper1);
            telemetry.addData("Right Bumper", right_bumper1);
            telemetry.addData("A Button", a_button1);
            telemetry.addData("B Button", b_button1);
            telemetry.addData("X Button", x_button1);
            telemetry.addData("Y Button", y_button1);
            telemetry.addData("Dpad Up", dup1);
            telemetry.addData("Dpad Down", ddown1);
            telemetry.addData("Dpad Left" , dleft1);
            telemetry.addData("Dpad Right", dright1);
            telemetry.addData("Back", back1);
        }

        if (gamepadMode2 == 0) {
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine("CONTROLLER 2");

            telemetry.addData("Left Joystick y",  left_stick_y2);
            telemetry.addData("Right Joystick y", right_stick_y_2);
            telemetry.addData("Left Trigger", left_trigger2);
            telemetry.addData("Right Trigger", right_trigger2);
            telemetry.addData("Left Bumper", left_bumper2);
            telemetry.addData("Right Bumper", right_bumper2);
            telemetry.addData("A Button", a_button2);
            telemetry.addData("B Button", b_button2);
            telemetry.addData("X Button", x_button2);
            telemetry.addData("Y Button", y_button2);
            telemetry.addData("Dpad Up", dup2);
            telemetry.addData("Dpad Down", ddown2);
            telemetry.addData("Home", home2);
        }

        if (gamepadMode1 == 1 || gamepadMode2 == 1) {
            telemetry.addLine("--------------------------------------------------");
            telemetry.addLine("CONTROLLER 3");
            if (gamepadMode1 == 1) {
                telemetry.addLine("Being Controlled by Controller 1");
            } else {
                telemetry.addLine("Being Controlled by Controller 2");
            }
            telemetry.addData("Left Joystick (x, y)", left_stick_x3 + ", " + left_stick_y3);
            telemetry.addData("Right Joystick (x, y)", right_stick_x3 + ", " + right_stick_y3);
            telemetry.addData("Left Trigger", left_trigger3);
            telemetry.addData("Right Trigger", right_trigger3);
            telemetry.addData("Left Bumper", left_bumper3);
            telemetry.addData("Right Bumper", right_bumper3);
            telemetry.addData("A Button", a_button3);
            telemetry.addData("B Button", b_button3);
            telemetry.addData("X Button", x_button3);
            telemetry.addData("Y Button", y_button3);
            telemetry.addData("Dpad Up", dup3);
            telemetry.addData("Dpad Down", ddown3);
            telemetry.addData("Dpad Left" , dleft3);
            telemetry.addData("Dpad Right", dright3);
            telemetry.addData("Start", start3);
            telemetry.addData("Home", home3);
        }
        telemetry.update();
    }
}

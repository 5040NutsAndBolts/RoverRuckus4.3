package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

@TeleOp(name="File Outputs", group="Teleop")
@Disabled
public class FileOutputTesting extends OpMode{
    FileHelper file = null;
    int i = 0;

    @Override
    public void init(){
        try {
            file = new FileHelper();
        } catch (IOException e) {
            telemetry.addLine("IOException in file creation: \n"+e);
        }
    }

    @Override
    public void loop(){
        if(gamepad1.a) {
            try {
                file.clearFile();
            } catch (IOException e) {
                telemetry.addLine("IOException in file clearing");
            }
            i=0;
        }

        else{
            i++;
            try {
                file.writeToFile("" + i);
            } catch (IOException e) {
                telemetry.addLine("IOException in file writing");
            }
        }

        try {
            telemetry.addLine(file.readFromFile());
        } catch (IOException e) {
            telemetry.addLine("IOException in file reading");
        }

        telemetry.update();
    }
}

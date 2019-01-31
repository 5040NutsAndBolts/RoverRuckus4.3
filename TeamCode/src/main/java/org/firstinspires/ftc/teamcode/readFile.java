package org.firstinspires.ftc.teamcode;


import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


@TeleOp(name="readFile", group="Teleop")
@Disabled
public class readFile extends OpMode{
    FileHelper file = null;

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
        try {
            telemetry.addLine(file.readFromFile());
        } catch (IOException e) {
            telemetry.addLine("IOException in file reading: \n"+e);
        }

        telemetry.update();
    }
}

/*
 */

package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

/**
 * This class is used as an assistant for reading, writing, and clearing a text file
 * - Every method needs to throw IOExceptions
 *   - The methods run with IO methods that throw IOExceptions no matter what as a checking system
 * - ANY TIME THESE METHODS ARE CALLED FROM ANOTHER CLASS THEY NEED TO BE IN A TRY CATCH
 *
 * Example for using FileHelper:
 * ===== Creating the file =====
 * try {
 *    file = new FileHelper();
 * } catch (IOException e) {
 *    telemetry.addLine("IOException in file creation: \n"+e);
 * }
 * ===== Clearing the file =====
 * try {
 *    file.clearFile();
 * } catch (IOException e) {
 *    telemetry.addLine("IOException in file clearing: \n"+e);
 * }
 * ===== Writing to the file =====
 * try {
 *    file.writeToFile("TEXT TO WRITE");
 * } catch (IOException e) {
 *    telemetry.addLine("IOException in file writing: \n"+e);
 * }
 * ===== Reading from the file =====
 * try {
 *    telemetry.addLine(file.readFromFile());
 * } catch (IOException e) {
 *    telemetry.addLine("IOException in file reading: \n"+e);
 * }
 */
public class FileHelper{
    private File exportData = null;

    /**
     * Constructor for reading, writing, and clearing the file
     * @throws IOException
     */
    public FileHelper() throws IOException {
        // Finds the absolute path of the internal storage directory which can be written to
        String path = Environment.getExternalStorageDirectory().getAbsolutePath();
        exportData = new File(path + "/exportedData.txt");
        // If the file does not exist, create one
        if (!exportData.exists())
            exportData.createNewFile();
    }

    /**
     * Method for writing text into the file
     * @param text the text to be written into the file
     * @throws IOException to get rid of try catches
     */
    public void writeToFile(String text)throws IOException {
        // Creates a new buffer writer in the file
        //    Buffer writer used file writer to write onto file
        //       Buffer writer overrides current file data
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(exportData.getAbsoluteFile()));
        // Writes text onto the first line of the file
        bufferedWriter.write(text);
        // Closes writer to ensure resources are not lost
        bufferedWriter.close();
    }

    /**
     * Method for getting the line of the file
     * @return string of the first line of the file
     * @throws IOException to get rid of try catches
     */
    public String readFromFile() throws IOException {
        // Creates a new buffer reader using file reader to read data from the file
        BufferedReader bufferedReader = new BufferedReader(new FileReader(exportData.getAbsoluteFile()));
        // Creates a string based on the first line from the file
        String readData = bufferedReader.readLine();
        // Closes the reader since only the first line needs to be read
        bufferedReader.close();
        // Returns the string of first line
        return readData;
    }

    /**
     * Method for clearing the file
     * @throws IOException to get rid of try catches
     */
    public void clearFile() throws IOException {
        // Creates a new buffer writer in the file
        //    Buffer writer used file writer to write onto file
        //       Buffer writer overrides current file data
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(exportData.getAbsoluteFile()));
        // Immediately closes buffer writer after overriding file data which clears the file
        bufferedWriter.close();
    }
}
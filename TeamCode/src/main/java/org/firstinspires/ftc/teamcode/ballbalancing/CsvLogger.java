package org.firstinspires.ftc.teamcode.ballbalancing;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class CsvLogger {
    private File file;
    private FileWriter writer;
    StringBuffer m_csvLogString = new StringBuffer();

    public CsvLogger(String filename) {
        m_csvLogString.setLength(0);
        try {
            String logFilePath = String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
            File dir = AppUtil.getInstance().getSettingsFile(""); // /sdcard/FIRST/settings
            file = new File(dir, filename);
            writer = new FileWriter(file);
            //writer.write("Time,Angle1,Angle2,Angle3\n");  // header
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void log(String data) {
        m_csvLogString.append(data + "\n");
        try {
            writer.write(data);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String close() {
        try {
            writer.flush();
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        String rv = "Stop complete";
        if (m_csvLogString.length() > 0) {
            String csvPath = String.format("%s/FIRST/settings/robot-data.csv",
                    Environment.getExternalStorageDirectory().getAbsolutePath());
            try (FileWriter csvWriter = new FileWriter(csvPath)) {
                csvWriter.write(m_csvLogString.toString());
            }
            catch (IOException e) {
                rv = e.getMessage();
            }
        }
        return rv;
    }

    public File getFile() {
        return file;
    }
}

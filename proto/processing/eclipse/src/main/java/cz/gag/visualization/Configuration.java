/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import cz.gag.common.ProcessingApplet;

public class Configuration {

    static public boolean logToFile = System.getProperty("logToFile") == null ? true
            : Boolean.valueOf(System.getProperty("logToFile"));

    static public String gestureName = System.getProperty("gestureName", "g");

    static public boolean useSerial = System.getProperty("replayPath") == null ? true : false;

    static public boolean paused = false;

    static public boolean replayData = !useSerial && true;

    static public boolean receiveAcc = System.getProperty("receiveAcc") == null ? false
            : Boolean.valueOf(System.getProperty("receiveAcc"));
    static public int packetSize = receiveAcc ? 21 : 15;

    static public String prefixPath = "/dev/";
    static public String deviceRight = "ttyUSB1";
    static public String portNameRight = System.getProperty("portNameRight", prefixPath + deviceRight);
    static public int baudRateRight = System.getProperty("baudRateRight") == null ? 57600 // 115200 38400 57600
            : Integer.valueOf(System.getProperty("baudRateRight"));
    // 38400 9600 115200 57600

    static public String deviceLeft = "ttyUSB0";
    // static String portNameLeft = prefixPath + deviceLeft;
    // static int baudRateLeft = 115200; // 38400 9600 115200 57600

    static public String portNameLeft = System.getProperty("portNameLeft",prefixPath + deviceLeft);
    static public int baudRateLeft = System.getProperty("baudRateLeft") == null ? 57600 // 115200 38400
            : Integer.valueOf(System.getProperty("baudRateLeft"));

    // output-ttyUSB1-2018-12-01_23:26:10.882.log

    // String replayDataFile = "./output-ttyUSB1-2018-12-01_23:26:10.882.log";
    // relative path is g.e.
    // ./gag/proto/processing/eclipse/output-ttyUSB0-2018-12-01_23:46:44.593.log
    // String replayDataFile = "./output-ttyUSB0-2018-12-01_23:46:44.593.log";
    static public String replayDataFile = System.getProperty("replayPath", "./output-test3.log");

    static public String replayRefPathFile = System.getProperty("replayRefPath", "./output-test3-ref.log");

    // String replayDataFile =
    // "/home/vprusa/output-ttyUSB0-2018-12-09_054212.063.log";
    //
    // String replayDataFile =
    // "/home/vprusa/workspace/p/notes/work/projects/arduino/gag/proto/processing/eclipse/output-ttyUSB1-2018-12-01_23:26:10.882.log";
    // Get the date today using Calendar object.
    static public Date today = Calendar.getInstance().getTime();
    // Using DateFormat format method we can create a string
    // representation of a date with the defined format.
    static public DateFormat df = new SimpleDateFormat("HH:mm:ss.SSS");
    static public DateFormat dfDate = new SimpleDateFormat("yyyy-MM-dd_HH:mm:ss.SSS");

    // Print what date is today!
    // System.out.println("Report Date: " + reportDate);

    static public String outputFile = "output-" + gestureName + "-" + deviceRight + "-{date}.log";

    static public boolean showGlobesDots = System.getProperty("showGlobesDots") == null ? false
            : Boolean.valueOf(System.getProperty("showGlobesDots"));
    static public boolean showGlobesBody = System.getProperty("showGlobesBody") == null ? false
            : Boolean.valueOf(System.getProperty("showGlobesBody"));
    static public boolean showRefHandsSensorValues = System.getProperty("showRefBodies") == null ? false
            : Boolean.valueOf(System.getProperty("showRefBodies"));
    
    static public boolean showGlobesLines = System.getProperty("showGlobesLines") == null ? false
            : Boolean.valueOf(System.getProperty("showGlobesLines"));
    static public float globesRotationCoeficientX = System.getProperty("globesRotationCoeficientX") == null ? 1.0f
            : Float.valueOf(System.getProperty("globesRotationCoeficientX"));
    static public float globesRotationCoeficientY = System.getProperty("globesRotationCoeficientY") == null ? 1.0f
            : Float.valueOf(System.getProperty("showGlobesBody"));

    static public boolean useFilter = System.getProperty("useFilter") == null ? true
            : Boolean.valueOf(System.getProperty("useFilter"));

    static public int dotsKeepN = System.getProperty("dotsKeepN") == null ? 30
            : Integer.valueOf(System.getProperty("dotsKeepN"));

    static public int refSensorValueDistanceLimit = 50;
    
    static public ProcessingApplet app;
}

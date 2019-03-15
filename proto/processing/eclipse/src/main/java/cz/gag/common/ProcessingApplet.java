/*
Copyright (c) 2018 Vojtěch Průša, Lukrécia Mertová
*/
package cz.gag.common;

//import peasy.*;
import processing.serial.*;
import processing.*;
import processing.core.PApplet;
//import processing.opengl.*;
//import toxi.color.RGBAccessor;

import toxi.geom.*;
//import toxi.processing.*;

import java.awt.Color;
import java.io.*;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.NavigableSet;
import java.util.OptionalDouble;
import java.util.OptionalLong;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import cz.gag.recognition.BothHandsGesture;
import cz.gag.recognition.Sensor;
import cz.gag.visualization.Button;
import cz.gag.visualization.Configuration;
import cz.gag.visualization.GestLineData;
import cz.gag.visualization.HandData;
import cz.gag.visualization.KeyboardRecorder;
import cz.gag.visualization.LineData;
import cz.gag.visualization.Point3D;
import cz.gag.visualization.RePlayer;

import java.nio.file.Paths;

import jssc.SerialPortEvent;

/**
 * @author Vojtech Prusa
 *
 *         It is nasty to have everything in 1 class but it escalates
 *         prototyping having shared variables and no need to change whole
 *         classes structures every brainstorm happens
 */
public class ProcessingApplet extends PApplet {

    public static void main(String[] args) {
        PApplet.main("cz.gag.common.ProcessingApplet");

        for (int i = 0; i < args.length; i++) {
            String arg = args[i];
            if (arg.startsWith("-D")) {
                if (arg.contains("=")) {
                    String[] mayBeProperty = arg.split("=");
                    if (mayBeProperty != null && mayBeProperty.length == 2) {
                        System.getProperties().setProperty(mayBeProperty[0].replaceAll("-D", ""), mayBeProperty[1]);
                    }
                } else {
                    if (i < args.length - 1) {
                        System.getProperties().setProperty(arg.replaceAll("-D", ""), args[i + 1]);
                    }
                    i++;
                }
            }
            // System.getProperties().setProperty(, value)
        }
        /*
         * System.getProperties().forEach((k, v) -> { System.out.println(k + ":" + v);
         * });
         */
    }
// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP
// output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
// https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2012-06-20 - initial release

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//     (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP
// output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at
// https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2012-06-20 - initial release

    /*
     * ============================================ I2Cdev device library code is
     * placed under the MIT license Copyright (c) 2012 Jeff Rowberg
     * 
     */

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
// (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

    // TODO move sensor in packet to 1. position, change code here and for arduino
    // send packet


    public static HandData leftHandData = new HandData(Hand.LEFT);
    public static HandData rightHandData = new HandData(Hand.RIGHT);
    // TODO
    // HandData rightHandData = new HandData();

    //ToxiclibsSupport gfx;

    Serial portRight; // The serial port
    Serial portLeft; // The serial port
    char[] teapotPacket = new char[21]; // InvenSense Teapot packet
    int serialCount = 0; // current packet byte position
    int aligned = 0;
    int interval = 0;
    int sensorsCount = 6;
    PrintWriter out = null;

    public void settings() {
        // size(900, 400, OPENGL);
        size(900, 500, P3D);
        // size(1200, 900, OPENGL);
        Configuration.app = this;
    }

    // lukrecia -start

    //PeasyCam cam;
    float dim = 0;
    // ArrayList<Point3D>[] points = new ArrayList[5];
    // float[][] axis = new float[5][4];
    // Point3D newP;
    int scale = 300;
    float max;
    float x_axis, y_axis, z_axis;

    int clk = 1; // number of times the button is cli

    Button on_button;
    BothHandsGesture refHandsGesture;
    public void setup() {
        // 300px square viewport using OpenGL rendering
        //gfx = new ToxiclibsSupport(this);

        // Create an instance of SimpleDateFormat used for formatting
        // the string representation of date (month/day/year)
        String reportDate = Configuration.dfDate.format(Configuration.today);

        Configuration.outputFile = Configuration.outputFile.replace("{date}", reportDate);
        if (Configuration.logToFile) {
            try {
                out = new PrintWriter(new FileWriter(Configuration.outputFile, true), true);
                // out.write("Test run " + reportDate);
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        // out.close();

        keyboardRecorder = new KeyboardRecorder(Configuration.outputFile.replace(".log", "-keyboard.log"));

        // setup lights and antialiasing
        lights();
        smooth();

        // create the button object
        translate(0, 0);
        on_button = new Button("Sphere off", 0, 0, 100, 50);

        // display serial port list for debugging/clarity
        println(Serial.list());

        // get the first available port (use EITHER this OR the specific port code
        // below)

        // get a specific serial port (use EITHER this OR the first-available code
        // above)
        // String portName = "COM4";

        /*
         * cam = new PeasyCam(this, 1000); cam.setMinimumDistance(20);
         * cam.setMaximumDistance(900);
         */
        x_axis = width * 0.3f;
        y_axis = -height * 0.3f;
        z_axis = height * 0.3f;

        // open the serial port
        // port = new Serial(this, portName, 115200); // 9600 115200 57600
        if (Configuration.useSerial) {

            try {
                portLeft = new Serial(this, Configuration.portNameLeft, Configuration.baudRateLeft); // 115200 9600
                portLeft.write('r');
            } catch (Exception e) {
                e.printStackTrace();
            }
            try {
                portRight = new Serial(this, Configuration.portNameRight, Configuration.baudRateRight); // 115200 9600
                portRight.write('r');
            } catch (Exception e) {
                e.printStackTrace();
            }
            // send single character to trigger DMP init/start
            // (expected by MPU6050_DMP6 example Arduino sketch)
        }
        /*if (Configuration.replayData && Configuration.replayRefPathFile != null) {
            Thread t1 = new Thread(new RePlayer(Configuration.replayDataFile, Configuration.replayRefPathFile));
            t1.start();
        } else */
        if (Configuration.replayRefPathFile != null) {
            refHandsGesture = new BothHandsGesture(Configuration.replayRefPathFile);
        }
        if (Configuration.replayData) {
            Thread t1 = new Thread(new RePlayer(Configuration.replayDataFile));
            t1.start();

        }
        b = new Button("Button", -width / 2 + 100, height / 2 - 100, 150, 80);

        new Thread() {
            public void run() {
                while(true) {
                    try {
                        Serial p = null;//portLeft;//portRight; // portLeft;
                    if(p != null && p.active()) {
                        //portRight.write(new byte[] { '$', (byte) 0x99, (byte) rightHandSensorIndex });
                        for(int i = 0; i < 10; i++) { 
                            p.write(new byte[] { '$', /*(byte) 0x99,*/ (byte) rightHandSensorIndex });
                            Thread.sleep(1);
                        }
                        p.write(new byte[] { '$', (byte) 0x99, (byte) rightHandSensorIndex++ });
                        if (rightHandSensorIndex >= Sensor.values().length) {
                            rightHandSensorIndex = 0;
                        }
                       // System.out.println(".");
                    }
                  
                        Thread.sleep(1);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            };
        }.start();
    }

    Color mapColor(float _x, float _y, float _z) {
        int r = (int) (map(_x, 0, x_axis, 0, 255));
        int g = (int) (map(_y, 0, y_axis, 0, 255));
        int b = (int) (map(_z, 0, z_axis, 0, 255));
        return new Color(r, g, b);
    }

    void axis() {
        strokeWeight(2);
        stroke(255, 255, 255);
        line(0, 0, 0, x_axis, 0, 0);
        stroke(224, 224, 224);
        line(0, 0, 0, 0, y_axis, 0);
        stroke(192, 192, 192);
        line(0, 0, 0, 0, 0, z_axis);
    }

    public static float distanceTo(Point3D p, Point3D p2) {
        return (float) Math.sqrt(Math.pow(p2.x - p.x, 2) + Math.pow(p2.y - p.y, 2) + Math.pow(p2.z - p.z, 2));
    }
    
    public static float betweenPointAndLine(float[] point, float[] lineStart, float[] lineEnd) {
        float[] PointThing = new float[3];
        float[] TotalThing = new float[3];
        PointThing[0] = lineStart[0] - point[0];
        PointThing[1] = lineStart[1] - point[1];
        PointThing[2] = lineStart[2] - point[2];

        TotalThing[0] = (PointThing[1] * lineEnd[2] - PointThing[2] * lineEnd[1]);
        TotalThing[1] = -(PointThing[0] * lineEnd[2] - PointThing[2] * lineEnd[0]);
        TotalThing[2] = (PointThing[0] * lineEnd[1] - PointThing[1] * lineEnd[0]);

        float distance = (float) (Math
                .sqrt(TotalThing[0] * TotalThing[0] + TotalThing[1] * TotalThing[1] + TotalThing[2] * TotalThing[2])
                / Math.sqrt(lineEnd[0] * lineEnd[0] + lineEnd[1] * lineEnd[1] + lineEnd[2] * lineEnd[2]));
        return distance;
    }


    // mouse button clicked
    /*
     * public void mousePressed() { if (on_button.MouseIsOver()) {
     * 
     * // print some text to the console pane if the button is clicked
     * System.out.print("Clicked: "); System.out.println(clk++); } }
     */

    // the Button class
    /*
     * public class Button { String label; // button label float x; // top left
     * corner x position float y; // top left corner y position float w; // width of
     * button float h; // height of button
     * 
     * // constructor Button(String labelB, float xpos, float ypos, float widthB,
     * float heightB) { label = labelB; x = xpos; y = ypos; w = widthB; h = heightB;
     * }
     * 
     * void Draw() { //translate(0, 0); fill(218); stroke(141);
     * 
     * rect(x, y, w, h, 10); textAlign(CENTER, CENTER); fill(0); text(label, x + (w
     * / 2), y + (h / 2)); }
     * 
     * boolean MouseIsOver() { if (mouseX > x && mouseX < (x + w) && mouseY > y &&
     * mouseY < (y + h)) { return true; } return false; } }
     */

    public int getColorForSensor(int i) {
        return getColorForSensor(i, 255);
    }
    
    public int getColorForSensor(int i, int alpha) {
        switch (i) {
        case 0:
            return color(0, 191, 255, alpha);
        case 1:
            return color(124, 252, 0, alpha);
        case 2:
            return color(255, 255, 255, alpha);
        case 3:
            return color(255, 0, 0, alpha);
        case 4:
            return color(255, 140, 0, alpha);
        case 5:
            return color(124, 140, 124, alpha);
        }
        return 0;
    }

    public void draw() {

        /*if (gfx != null) {
            translate(width / 2f, height / 6);
        }*/
        // draw the button in the window
        on_button.draw();

        if (millis() - interval > 1000) { // resend single character to trigger DMP init/start // in case the MPU is
            interval = millis();
        }

        // black background
        background(0);

        noStroke();


        
        for (Hand hi : Hand.values()) {
            for (int i = 1; i < sensorsCount + 1; i++) {

           
                // translate everything to the middle of the viewport
                noStroke();

                pushMatrix();
                translate(i * (width / 4) - (width * 1.65f) + hi.ordinal() * width * 1.6f, -height + 150, -1000);
                // to rotate all arrows up
                rotateX(-PI / 2);
                
                // translate(i*(width/2)-width/2, height/2, -200);
                // translate(-width/2, height / 2);
                // translate(i*(width/2), 0, -200);
                // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
                // ...and other weirdness I haven't figured out yet
                // rotateY(-ypr[0]);
                // rotateZ(-ypr[1]);
                // rotateX(-ypr[2]);

                // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
                // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
                // different coordinate system orientation assumptions between Processing
                // and InvenSense DMP)

                HandData currentHand = hi == Hand.LEFT ? leftHandData : rightHandData;
                int nmb = i - 1;
                // System.out.println(currentHand.getSensorData(nmb).toString());

                float[] axis = currentHand.getSensorData(nmb).quatO.toAxisAngle();
                //rotate(axis[0], -axis[1], axis[3], axis[2]);
                rotate(axis[0], axis[1], axis[2], axis[3]);

                // print(leftHandData.getSensorData(i).toString() + "\n");
                if (Configuration.showGlobesDots || Configuration.showGlobesLines) {
                    Point3D newP = new Point3D(axis[1] * scale, axis[2] * scale, axis[3] * scale);
                    currentHand.points[nmb].add(newP);
                    if (Configuration.dotsKeepN > 0 && Configuration.dotsKeepN < currentHand.points[nmb].size()) {
                        currentHand.points[nmb].remove(0);
                    }
                }

                // draw polygonal line

                // draw main body in red
                // fill(255 - nmb * 50, i * 50, nmb * 50, 255);
                fill(getColorForSensor(nmb));
                box(10, 10, 200);

                // draw front-facing tip in blue
                fill(0, 0, 255, 200);
                pushMatrix();
                translate(0, 0, -120);
                rotateX(PI / 2);
                drawCylinder(0, 20, 20, 8);
                popMatrix();

                // draw wings and tail fin in green
                fill(0, 255, 0, 200);
                beginShape(TRIANGLES);
                vertex(-100, 2, 30);
                vertex(0, 2, -80);
                vertex(100, 2, 30); // wing top layer
                vertex(-100, -2, 30);
                vertex(0, -2, -80);
                vertex(100, -2, 30); // wing bottom layer
                vertex(-2, 0, 98);
                vertex(-2, -30, 98);
                vertex(-2, 0, 70); // tail left layer
                vertex(2, 0, 98);
                vertex(2, -30, 98);
                vertex(2, 0, 70); // tail right layer
                endShape();
                beginShape(QUADS);
                vertex(-100, 2, 30);
                vertex(-100, -2, 30);
                vertex(0, -2, -80);
                vertex(0, 2, -80);
                vertex(100, 2, 30);
                vertex(100, -2, 30);
                vertex(0, -2, -80);
                vertex(0, 2, -80);
                vertex(-100, 2, 30);
                vertex(-100, -2, 30);
                vertex(100, -2, 30);
                vertex(100, 2, 30);
                vertex(-2, 0, 98);
                vertex(2, 0, 98);
                vertex(2, -30, 98);
                vertex(-2, -30, 98);
                vertex(-2, 0, 98);
                vertex(2, 0, 98);
                vertex(2, 0, 70);
                vertex(-2, 0, 70);
                vertex(-2, -30, 98);
                vertex(2, -30, 98);
                vertex(2, 0, 70);
                vertex(-2, 0, 70);
                endShape();

                popMatrix();

            }

            pushMatrix();

            translate(-(width * 0.4f) + hi.ordinal() * (width * 0.6f) + hi.ordinal() * 300 - 100, 250f, -700);
            rotateX(Configuration.globesRotationCoeficientX);
            rotateY(Configuration.globesRotationCoeficientY);
            axis();
            if (Configuration.showGlobesBody) {
                fill(255, 255, 255, 50);
                stroke(255, 255, 255, 10);
                sphere(scale - 15);
                noStroke();
            }


            if (Configuration.showRefHandsSensorValues) {
                // draw ref data
                if(refHandsGesture != null) {
                    pushMatrix();
                    stroke(255, 255, 255, 50);
    
                    for (Hand hval : Hand.values()) {
                        for (int i = 1; i < sensorsCount + 1; i++) {
                            //for(int si = 0; si < refHandsGesture.left.sensoresGestrues.length; si++) {
                                ArrayList<GestLineData> data = refHandsGesture.left.sensoresGestrues[i-1].data;
                                GestLineData line = null;
                                Iterator<GestLineData> iterator = data.iterator();
                                
                                while(iterator.hasNext() && (line = iterator.next()) != null) {
                                    pushMatrix();
                                    noStroke();
                                    fill(getColorForSensor(i-1,20));
                                    stroke(color(20,20,20,10));
                                    //stroke(getColorForSensor(i-1,10));
                                    float[] axis = line.quatO.toAxisAngle();
                                    Point3D newP = new Point3D(axis[1] * scale, axis[2] * scale, axis[3] * scale);
                                    
                                    Iterator<Point3D> it =
                                            (hval == Hand.LEFT ? leftHandData : rightHandData).points[i-1].iterator();
                                    while (it.hasNext()) {
                                        Point3D toCmp = it.next();
                                        if(distanceTo(toCmp,newP) < Configuration.refSensorValueDistanceLimit) {
                                            stroke(getColorForSensor(i-1));
                                        }
                                    }
                                    //Point3D newP = new Point3D(axis[1], axis[2], axis[3]);
                                    //rotate(axis[0], -axis[1], axis[3], axis[2]);
                                    translate(newP.x, newP.y, newP.z);
                                    sphere(Configuration.refSensorValueDistanceLimit);
                                    noStroke();
    
                                    //System.out.println(line.hand+" "+line.sensor+" "+newP.x + "" + newP.y+ " " + newP.z);
                                    popMatrix();
                                }
                           // }
    
                        }
                    }
                    popMatrix();
    
                }
            }

            for (int i = 0; i < sensorsCount; i++) {

                HandData currentHand = hi == Hand.LEFT ? leftHandData : rightHandData;
                /*
                 * if ((clk % 2) == 0) { System.out.print(clk); noStroke(); lights();
                 * sphere(10); }
                 */

                if (Configuration.showGlobesDots) {
                    stroke(getColorForSensor(i));

                    // translate(-60*i,0,0);
                    Point3D beg = null; // new Point3D(0, 0, 0);
                    Point3D prev = null;
                    Iterator<Point3D> it = currentHand.points[i].iterator();
                    while (it.hasNext()) {
                        Point3D end = it.next();
                        pushMatrix();
                        translate(end.x, end.y, end.z);
                        // stroke(255 - (i) * 50, 255 - i * 50, (i) * 50);
                        // stroke(mapColor(end.x, end.y, end.z));
                        sphere(2);
                        popMatrix();

                        if (prev != null) {
                            // Arc3D(end.x, end.y, 50, 50, 0, PI / 2.0);
                            // Arc3D a3d = new Arc3D(end.x,end.y, end.z, 1, 1, color(255,255,60,250));
                            // arc
                            // Arc3
                        }
                        if (Configuration.showGlobesLines) {
                            if (beg != null) {
                                line(beg.x, beg.y, beg.z, end.x, end.y, end.z);
                            }
                        }
                        beg = end;
                        prev = end;
                    }
                }
                stroke(255);
                strokeWeight(1);
            }
            popMatrix();

        }

        stroke(255);
        // drawButton();
        // background(0);
        // rotateY(radians(frameCount));

    }

    public static int rightHandSensorIndex = 0;

    public void serialEvent(Serial port) {

        new Thread() {
            public void run() {

                // System.out.println("serialEvent");
                if (port == null) {
                    return;
                }
                interval = millis();

               // port.write(new byte[] { '$', (byte) 0x99, (byte) rightHandSensorIndex++ });
                //port.write(new byte[] { '$', (byte) 0x99, (byte) rightHandSensorIndex++ });
                //port.write(new byte[] { '$', (byte) 0x99, (byte) rightHandSensorIndex++ });
              //  if (rightHandSensorIndex >= Sensor.values().length) {
               //     rightHandSensorIndex = 0;
               // }
                while (port.available() > 0) {
                    // byte[] bytes = port.readBytesUntil('*');
                    // print();
                    // print('\n');

                    // if(true) {
                    // continue;
                    // }

                    // byte[] bytes = port.readBytes(1);
                    int ch = port.read();// '.';
                    // char ch = parseChar(bytes[0]);
                    if (ch == 'ÿ' || ch == 0x00) {
                        // continue;
                    } else {
                        print((char) ch);
                    }

                    // out.close();
                    if (ch == '*' || ch == '$') {
                        serialCount = 0;
                    } // this will help with alignment
                      // else { continue; }
                    if (aligned < 4) {
                        // make sure we are properly aligned on a 15-byte packet
                        if (serialCount == 0) {
                            if (ch == '*' || ch == '$')
                                aligned++;
                            else
                                aligned = 0;
                        } else if (serialCount == 1) {
                            if (ch == 0x99)
                                aligned++;
                            else
                                aligned = 0;
                        } else if (serialCount == Configuration.packetSize - 3) {
                            // println("MPU: " + ch);
                        } else if (serialCount == Configuration.packetSize - 2) {
                            if (ch == '\r')
                                aligned++;
                            else
                                aligned = 0;
                        } else if (serialCount == Configuration.packetSize - 1) {
                            if (ch == '\n')
                                aligned++;
                            else
                                aligned = 0;
                        }
                        // println(ch + " " + aligned + " " + serialCount);
                        serialCount++;
                        if (serialCount == Configuration.packetSize) {
                            serialCount = 0;
                        }
                    } else {
                        if (serialCount > 0 || ch == '*' || ch == '$') {
                            teapotPacket[serialCount++] = (char) ch;

                            /*
                             * if(false) { print('.'); if (serialCount == 21) { serialCount = 0; } if(true)
                             * serialCount =0; continue; }
                             */
                            if (serialCount == Configuration.packetSize) {
                                serialCount = 0; // restart packet byte position

                                int sensor = (int) (teapotPacket[2]);

                                // get quaternion from data packet
                                float q0 = ((teapotPacket[3] << 8) | teapotPacket[4]);
                                float q1 = ((teapotPacket[5] << 8) | teapotPacket[6]);
                                float q2 = ((teapotPacket[7] << 8) | teapotPacket[8]);
                                float q3 = ((teapotPacket[9] << 8) | teapotPacket[10]);

                                /*
                                 * float q0 = ((teapotPacket[4] << 8) | teapotPacket[3]); float q1 =
                                 * ((teapotPacket[6] << 8) | teapotPacket[5]); float q2 = ((teapotPacket[8] <<
                                 * 8) | teapotPacket[7]); float q3 = ((teapotPacket[10] << 8) |
                                 * teapotPacket[9]);
                                 */
                                float[] localQuat = new float[4];
                                float[] localAcc = new float[3];
                                localQuat[0] = q0 / 16384.0f;
                                localQuat[1] = q1 / 16384.0f;
                                localQuat[2] = q2 / 16384.0f;
                                localQuat[3] = q3 / 16384.0f;
                                // println("MPU: %d", sensor);

                                for (int i = 0; i < 4; i++)
                                    if (localQuat[i] >= 2)
                                        localQuat[i] = -4 + localQuat[i];

                                // set our toxilibs quaternion to new data
                                // localQuatO.set(localQuat[0], localQuat[1],
                                // localQuat[2], localQuat[3]);

                                if (Configuration.receiveAcc) {
                                    localAcc[0] = ((teapotPacket[11] << 8) | teapotPacket[12]);// - (gravity *8192f);
                                    localAcc[1] = ((teapotPacket[13] << 8) | teapotPacket[14]);// - (gravity *8192f);
                                    localAcc[2] = ((teapotPacket[15] << 8) | teapotPacket[16]);// - (gravity *8192f);
                                }
                                // println("sensor" + sensor);
                                if (sensor > Sensor.values().length - 1 || sensor < 0) {
                                    aligned = 0;
                                    return;
                                }
                                LineData dl = null;
                                try {
                                    if (teapotPacket != null) {
                                        if (teapotPacket[0] == '*') {
                                            dl = new LineData(Calendar.getInstance().getTime(), localQuat, localAcc,
                                                    Sensor.values()[sensor], Hand.LEFT);
                                            // println("Left: " + dl.toString());
                                            if (Configuration.useFilter && (dl.quatO.w > 1 || dl.quatO.x > 1
                                                    || dl.quatO.y > 1 || dl.quatO.z > 1 || dl.quatO.w < -1
                                                    || dl.quatO.x < -1 || dl.quatO.y < -1 || dl.quatO.z < -1)) {
                                                dl = null;
                                                // println("Left: " + dl.toString());
                                            } else {
                                                leftHandData.setSensorData(dl);
                                            }
                                        } else if (teapotPacket[0] == '$') {
                                            dl = new LineData(Calendar.getInstance().getTime(), localQuat, localAcc,
                                                    Sensor.values()[sensor], Hand.RIGHT);
                                            // println("Right: " + dl.toString());
                                            if (Configuration.useFilter && (dl.quatO.w > 1 || dl.quatO.x > 1
                                                    || dl.quatO.y > 1 || dl.quatO.z > 1 || dl.quatO.w < -1
                                                    || dl.quatO.x < -1 || dl.quatO.y < -1 || dl.quatO.z < -1)) {
                                                // println("Left: " + dl.toString());

                                                dl = null;
                                            } else {
                                                rightHandData.setSensorData(dl);
                                            }

                                        }
                                        if (out != null) {
                                            if (dl != null && !Configuration.replayData) {
                                                //System.out.println(dl.toFileString());
                                                out.println(dl.toFileString());
                                            }
                                        }
                                    }
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }

                            }
                        }
                    }
                }

            };
        }.start();
    }

    void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
        float angle = 0;
        float angleIncrement = TWO_PI / sides;
        beginShape(QUAD_STRIP);
        for (int i = 0; i < sides + 1; ++i) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();

        // If it is not a cone, draw the circular top cap
        if (topRadius != 0) {
            angle = 0;
            beginShape(TRIANGLE_FAN);

            // Center point
            vertex(0, 0, 0);
            for (int i = 0; i < sides + 1; i++) {
                vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
                angle += angleIncrement;
            }
            endShape();
        }

        // If it is not a cone, draw the circular bottom cap
        if (bottomRadius != 0) {
            angle = 0;
            beginShape(TRIANGLE_FAN);

            // Center point
            vertex(0, tall, 0);
            for (int i = 0; i < sides + 1; i++) {
                vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
                angle += angleIncrement;
            }
            endShape();
        }
    }

    KeyboardRecorder keyboardRecorder;

    public void keyPressed() {
        if (keyboardRecorder != null) {
            // keyboardRecorder.recordKey(key);
        }
        if (keyCode == ' ') {
            Configuration.paused = !Configuration.paused;
        }
        if (keyCode == 'D') {
            Configuration.showGlobesDots = !Configuration.showGlobesDots;
        }
        if (keyCode == 'B') {
            Configuration.showGlobesBody = !Configuration.showGlobesBody;
        }
        if (keyCode == 'R') {
            Configuration.showRefHandsSensorValues = !Configuration.showRefHandsSensorValues;
        }
        if (keyCode == 'I') {
            Configuration.showGlobesLines = !Configuration.showGlobesLines;
        }

        if (keyCode == 'S') {
            for (int i = 0; i < Sensor.values().length; i++) {
                leftHandData.points[i].clear();
                rightHandData.points[i].clear();
            }

        }
        if (keyCode == 'L') {
            printCamera();
            // camera(1, 0.3f, 1, 0, 0, 0, 1, 1, 0);
        }

        if (keyCode == 'R') {
            printCamera();
            // camera(-0.1f, 0.3f, 1f, 0, 0, 0, 1, 1, 0f);
        }
        if (keyCode == LEFT) {
            Configuration.globesRotationCoeficientY += 0.2;
        }
        if (keyCode == RIGHT) {
            Configuration.globesRotationCoeficientY -= 0.2;
        }
        if (keyCode == UP) {
            Configuration.globesRotationCoeficientX += 0.2;
        }
        if (keyCode == DOWN) {
            Configuration.globesRotationCoeficientX -= 0.2;
        }

    }
    Button b = null;
    
    public void drawButton() {
        // background (255, 255, 255);

        hint(DISABLE_DEPTH_TEST);
        pushMatrix();

        if (b.isIn(mouseX, mouseY)) {
            b.overRegion = true;
            // button = true;
        } else {
            b.overRegion = false;
            // button = false;
        }
        stroke(0, 0, 0);
        fill(255, 0, 0);
        textSize(12);
        if (b.button) {
            fill(0, 0, 255);
            // line(0, 0, 100, 100);
            b.draw();
        } else {
            fill(0, 255, 0);
            b.draw();
        }

        popMatrix();

        hint(ENABLE_DEPTH_TEST);

    }

    public void mousePressed() {
        if (b.overRegion) {
            b.pressed = true;
        } else {
            b.pressed = false;
        }
        if (mouseButton == LEFT) {
            if (b != null && b.isIn(mouseX, mouseY)) {
                b.button = !b.button;
            }
        }
    }

    public void mouseReleased() {
        b.pressed = false;
    }

}

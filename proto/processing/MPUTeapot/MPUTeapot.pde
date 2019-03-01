/*
Copyright (c) 2018 Vojtěch Průša
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;
//import java.io.; // PrintStream
import java.io.*;
import java.text.SimpleDateFormat;
import java.text.DateFormat;
import java.util.Calendar;
import java.util.Date;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

boolean useSerial = true;

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[21];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int aligned = 0;
int interval = 0;

float[][] q = new float[][]{
  new float[4],
  new float[4],
  new float[4],
  new float[4],
  new float[4]
};
Quaternion [] quat = new Quaternion[] {
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0)
};

float[][] qAcc = new float[][]{
  new float[4],
  new float[4],
  new float[4],
  new float[4],
  new float[4]
};
Quaternion [] quatAcc = new Quaternion[] {
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0),
  new Quaternion(1, 0, 0, 0)
};

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];
DateFormat df = new SimpleDateFormat("MM-dd-yyyy-HH:mm:ss.SSS");
int teapotCount = 5;
PrintWriter out = null;
void setup() {
    // 300px square viewport using OpenGL rendering
    size(900, 400, OPENGL);
    gfx = new ToxiclibsSupport(this);
    String device = "ttyUSB0";
    String portName = "/dev/" + device;

    // Create an instance of SimpleDateFormat used for formatting
    // the string representation of date (month/day/year)


    // Get the date today using Calendar object.
    Date today = Calendar.getInstance().getTime();
    // Using DateFormat format method we can create a string
    // representation of a date with the defined format.
    String reportDate = df.format(today);

    // Print what date is today!
    //System.out.println("Report Date: " + reportDate);

  try {
            out = new PrintWriter(new FileWriter("output-"+device+"-"+reportDate+".txt", true), true);
            out.write("Test run " + reportDate);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    //out.close();

    // setup lights and antialiasing
    lights();
    smooth();

    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)

    // get a specific serial port (use EITHER this OR the first-available code above)
    //String portName = "COM4";


    // open the serial port
    //port = new Serial(this, portName, 115200);
    if(useSerial){
    port = new Serial(this, portName, 115200);
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)

    port.write('r');
    }
}

void draw() {
 if (millis() - interval > 1000) {
      // resend single character to trigger DMP init/start
      // in case the MPU is halted/reset while applet is running
      if(useSerial){
        port.write('r');
      }
      interval = millis();
  }

  // black background
  background(0);

noStroke();

  for (int i = 1 ; i<teapotCount+1; i++){

    // translate everything to the middle of the viewport

    pushMatrix();
    translate(i*(width/4)-(width/4), height/4 - 100, -400);
    // to rotate all arrows up
    rotateX(-PI/2);
    //translate(i*(width/2)-width/2, height/2, -200);
    //translate(-width/2, height / 2);
    //translate(i*(width/2), 0, -200);
    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quat[i-1].toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    // draw main body in red
    fill(255-i*50, 0, i*50, 200);
    box(10, 10, 200);

    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();

    // draw wings and tail fin in green
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();

    popMatrix();

  // translate(-60*i,0,0);
 }

 stroke(255);

   for (int i = 1 ; i<teapotCount+1; i++){

     int startX = 150 + i*100;
     int startY = 300;

     //line(startX,startY,startX,startY-50);

    pushMatrix();
    translate(i*(width/4)-(width/4), height/2 + 100, -400);
    // to rotate all arrows up
    //rotateX(-PI/2);

    rotateX(-0.3);
    rotateY(PI/4);
    switch(i){
      case 1:
      rotateY(0.6);//rotateY(-0.2);
      break;
      case 2:
      rotateY(0.3);
      break;
      case 3:
      //rotateY(0);
      break;
      case 4:
      rotateY(-0.3);
      break;
      case 5:
      rotateY(-0.6);
      break;
    }

    // draw main body in red
    //fill(255-i*50, 0, i*50, 200);
    noFill();
    //box(100, 100, 100);
    line(100,100,200,200);

     beginShape();
     //size(100, 100, P3D);
//translate(250, 200, -100);
//rotateY(0.5);
//box(100);
 endShape();
    popMatrix();
  //noStroke();
    // translate everything to the middle of the viewport

/*    pushMatrix();
    translate(i*(width/4)-(width/4), height/2 + 100, -400);
    // to rotate all arrows up
    rotateX(-PI/2);
    //translate(i*(width/2)-width/2, height/2, -200);
    //translate(-width/2, height / 2);
    //translate(i*(width/2), 0, -200);
    // 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
    // ...and other weirdness I haven't figured out yet
    //rotateY(-ypr[0]);
    //rotateZ(-ypr[1]);
    //rotateX(-ypr[2]);

    // toxiclibs direct angle/axis rotation from quaternion (NO gimbal lock!)
    // (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
    // different coordinate system orientation assumptions between Processing
    // and InvenSense DMP)
    float[] axis = quatAcc[i-1].toAxisAngle();
    rotate(axis[0], -axis[1], axis[3], axis[2]);

    // draw main body in red
    fill(255-i*50, 0, i*50, 200);
    box(10, 10, 200);

    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();

    // draw wings and tail fin in green
    fill(0, 255, 0, 200);

    popMatrix();

  // translate(-60*i,0,0);
  */
 }
}

void serialEvent(Serial port) {
    interval = millis();
    while (port.available() > 0) {
        int ch = port.read();
        print((char)ch);

        //out.close();
        if (ch == '$') {serialCount = 0;}  // this will help with alignment
        //  else { continue; }
        if (aligned < 4) {
            // make sure we are properly aligned on a 15-byte packet
            if (serialCount == 0) {
                if (ch == '$') aligned++; else aligned = 0;
            } else if (serialCount == 1) {
                if (ch == 2) aligned++; else aligned = 0;
            } else if (serialCount == 18) {
                //println("MPU: " + ch);
            } else if (serialCount == 19) {
                if (ch == '\r') aligned++; else aligned = 0;
            } else if (serialCount == 20) {
                if (ch == '\n') aligned++; else aligned = 0;
            }
            //println(ch + " " + aligned + " " + serialCount);
            serialCount++;
            if (serialCount == 21) serialCount = 0;
        } else {
            if (serialCount > 0 || ch == '$') {
                teapotPacket[serialCount++] = (char)ch;
                if (serialCount == 21) {
                    serialCount = 0; // restart packet byte position

                    int finger = int(teapotPacket[10]);

                    // get quaternion from data packet
                    float q0 = ((teapotPacket[2] << 8) | teapotPacket[3]);
                    float q1 = ((teapotPacket[4] << 8) | teapotPacket[5]);
                    float q2 = ((teapotPacket[6] << 8) | teapotPacket[7]);
                    float q3 = ((teapotPacket[8] << 8) | teapotPacket[9]);

                    q[finger][0] = q0/ 16384.0f;
                    q[finger][1] = q1/ 16384.0f;
                    q[finger][2] = q2/ 16384.0f;
                    q[finger][3] = q3/ 16384.0f;
                    //println("MPU: %d", finger);

                    for (int i = 0; i < 4; i++) if (q[finger][i] >= 2) q[finger][i] = -4 + q[finger][i];

                    // set our toxilibs quaternion to new data
                    quat[finger].set(q[finger][0], q[finger][1], q[finger][2], q[finger][3]);


                    q0 = 1; //((teapotPacket[2] << 8) | teapotPacket[3]);
                    q1 = ((teapotPacket[4] << 8) | teapotPacket[5]);
                    q2 = ((teapotPacket[6] << 8) | teapotPacket[7]);
                    q3 = ((teapotPacket[8] << 8) | teapotPacket[9]);
                    qAcc[finger][0] = 1f;//q0 - (gravity *8192f);
                    qAcc[finger][1] = q1;// - (gravity *8192f);
                    qAcc[finger][2] = q2;// - (gravity *8192f);
                    qAcc[finger][3] = q3;// - (gravity *8192f);

                    quatAcc[finger].set(qAcc[finger][0], qAcc[finger][1], qAcc[finger][2], qAcc[finger][3]);
                    String report = "MPU f: " + finger + " Acc:" + qAcc[finger][0] + " " + qAcc[finger][1] + " " +
                    qAcc[finger][2] + " " + qAcc[finger][3] +" Gyro: " +q[finger][0] + " " + q[finger][1] + " " +
                    q[finger][2] + " " + q[finger][3];
                    println(report);

                    // Get the date today using Calendar object.
                    Date today = Calendar.getInstance().getTime();
                    // Using DateFormat format method we can create a string
                    // representation of a date with the defined format.
                    String reportDate = df.format(today);
                    String reportWithDate = reportDate + report;
                   if (out != null) {
                            out.println(reportWithDate);
                        }
                }
            }
        }
    }
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
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

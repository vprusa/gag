/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.io.BufferedReader;

import cz.gag.common.Hand;
import cz.gag.common.ProcessingApplet;

public class RePlayer extends DataFileParser<ReplayLine> implements Runnable {

    public RePlayer(String file) {
        super(file);
    }

    @Override
    public void run() {
        System.out.println("Player running");
        // serialEvent(mySerial);
        ReplayLine flNew = null;
        ReplayLine flOld = null;

        while (true) {
            // System.out.println(".");

            if (Configuration.paused) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                continue;
            }
            flNew = (ReplayLine) parseLine(ReplayLine.class);

            if (flNew == null) {
                br = new BufferedReader(fr);
                continue;
            }
            if (Configuration.useFilter && (flNew.quatO.w > 1 || flNew.quatO.x > 1 || flNew.quatO.y > 1
                    || flNew.quatO.z > 1 || flNew.quatO.w < -1 || flNew.quatO.x < -1 || flNew.quatO.y < -1
                    || flNew.quatO.z < -1))
                continue;

            //System.out.println(flNew.toString());
            // System.out.println(flNew.toFileString());
            // hand.getSensorData(flNew.sensor.ordinal()).quatO.set(flNew.quat[0],
            // flNew.quat[1], flNew.quat[2],flNew.quat[3]);
            if (flNew.hand == Hand.LEFT) {
                ProcessingApplet.leftHandData.setSensorData(flNew);
            } else {
                ProcessingApplet.rightHandData.setSensorData(flNew);
            }
            // quaternionObj[flNew.sensor.ordinal()].set(flNew.quat[0], flNew.quat[1],
            // flNew.quat[2], flNew.quat[3]);
            long timeDifference = flOld == null ? 1 : flNew.date.getTime() - flOld.date.getTime();
            System.out.println(line);

            flOld = flNew;

            try {
                Thread.sleep(timeDifference);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}


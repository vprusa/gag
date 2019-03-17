/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.io.BufferedReader;

import cz.gag.common.Configuration;
import cz.gag.common.Hand;
import cz.gag.common.ProcessingApplet;

/**
 * @author Vojtech Prusa
 * 
 * This class instance replays data to view
 */
public class RePlayer extends DataFileParser<ReplayLine> implements Runnable {

    public RePlayer(String file) {
        super(file);
    }

    @Override
    public void run() {
        System.out.println("Player running");
        ReplayLine flNew = null;
        ReplayLine flOld = null;

        while (true) {
            if (Configuration.paused) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                continue;
            }
            flNew = (ReplayLine) parseLine();

            if (flNew == null) {
                br = new BufferedReader(fr);
                continue;
            }
            if (Configuration.useFilter && (flNew.quatO.w > 1 || flNew.quatO.x > 1 || flNew.quatO.y > 1
                    || flNew.quatO.z > 1 || flNew.quatO.w < -1 || flNew.quatO.x < -1 || flNew.quatO.y < -1
                    || flNew.quatO.z < -1))
                continue;

            if (flNew.hand == Hand.LEFT) {
                ProcessingApplet.leftHandData.setSensorData(flNew);
            } else {
                ProcessingApplet.rightHandData.setSensorData(flNew);
            }
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


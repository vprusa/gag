/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.util.ArrayList;
import java.util.Calendar;

import cz.gag.common.Hand;
import cz.gag.recognition.Sensor;

/**
 * @author Vojtech Prusa
 *
 *  Stores information about line data of hand, hand id. and 
 *
 */
public class HandData {
    DataLine[] sensors = new DataLine[Sensor.values().length];
    Hand hand;

    public ArrayList<Point3D>[] points = new ArrayList[Sensor.values().length];

    public HandData(Hand hand) {
        for (int i = 0; i < Sensor.values().length; i++) {
            sensors[i] = new DataLine(Calendar.getInstance().getTime(), new float[] { 1f, 0f, 0f, 0f }, Sensor.values()[i], hand);
            points[i] = new ArrayList<Point3D>();
            // points[i].add(new Point3D(0, 0, 0));
        }
        this.hand = hand;
    }

    public DataLine getSensorData(Sensor f) {
        // TODO check that it is really what it seems to be
        return sensors[f.ordinal()];
    }

    public DataLine getSensorData(int f) {
        return getSensorData(Sensor.values()[f]);
    }

    public void setSensorData(DataLine data) {
        // TODO do not overwrite but use the same object and just recalculate data..
        data.quatO.set(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
        // TODO check that it is really what it seems to be
        sensors[data.sensor.ordinal()] = data;
    }

}

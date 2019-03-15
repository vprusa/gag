/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestLineData;
import toxi.geom.Quaternion;

public abstract class SensorGestureA implements SensorGesture {

    private Sensor sensor;
    private DataFileParser parser;

    SensorGestureA(Sensor sensor) {
        this.sensor = sensor;
    }

    public Sensor getSensor() {
        return sensor;
    }

    @Override
    public float matchesBy(Map<Date, GestLineData> data) {
        return 0.5f;
    }
    
}

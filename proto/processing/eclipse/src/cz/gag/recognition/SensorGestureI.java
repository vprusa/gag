/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

public class SensorGestureI extends SensorGestureA {

    SensorGestureI(Sensor sensor) {
        super(sensor);
    }

    /*
     * public float matches(HashMap<Date, Quaternion> data, Sensor sensor) { if
     * (sensor != this.getSensor()) return 0; return matches(data); }
     */
}

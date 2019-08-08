/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.Date;
import java.util.Map;

import cz.gag.visualization.GestDataLine;

public class SensorGestureI extends SensorGestureA {

    SensorGestureI(Sensor sensor) {
        super(sensor);
    }

    /*
      public float matches(HashMap<Date, Quaternion> data, Sensor sensor) { if
      (sensor != this.getSensor()) return 0; return matches(data); }
     */
  

    @Override
    public float matchesBy(Map<Date, GestDataLine> data) {
        return 0.5f;
    }
}

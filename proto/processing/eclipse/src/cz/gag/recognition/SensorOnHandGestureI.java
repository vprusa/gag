/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import cz.gag.global.Hand;
import cz.gag.visualization.DataFileParser;

public class SensorOnHandGestureI extends SensorOnHandGestureA {

    SensorOnHandGestureI(Hand hand, Sensor sensor, DataFileParser parser) {
        super(hand, sensor, parser);
    }

    /**
     * Instead of parent method with this one I do not know if to filter
     * 
     * public float matches(HashMap<Date, Quaternion> data, Sensor sensor) { //
     * actually values for other sensors should be filtered in parent method
     * matches() if (sensor != this.getSensor()) return 0; return matches(data); }
     */

}

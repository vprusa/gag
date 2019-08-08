/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.common.Hand;
import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestDataLine;

public class SensorOnHandGestureI extends SensorOnHandGestureA {

    SensorOnHandGestureI(Hand hand, Sensor sensor, DataFileParser<GestDataLine> parser) {
        super(hand, sensor, parser);
    }

    @Override
    public float matchesBy(Map<Date, GestDataLine> data) {
        // TODO Auto-generated method stub
        return 0;
    }

    /**
     * Instead of parent method with this one I do not know if to filter
     * 
     * public float matches(HashMap<Date, Quaternion> data, Sensor sensor) { //
     * actually values for other sensors should be filtered in parent method
     * matches() if (sensor != this.getSensor()) return 0; return matches(data); }
     */

}

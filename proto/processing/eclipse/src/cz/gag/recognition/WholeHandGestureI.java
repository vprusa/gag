package cz.gag.recognition;

import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.global.Hand;
import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestLineData;
import toxi.geom.Quaternion;

public class WholeHandGestureI extends HandGestureA {

    public SensorOnHandGestureI[] sensoresGestrues = new SensorOnHandGestureI[Sensor.values().length];
    private DataFileParser parser;

    WholeHandGestureI(Hand hand, String file) {
        super(hand);
        parser = new DataFileParser(file);  
        for (int i = 0; i < sensoresGestrues.length; i++) {
            sensoresGestrues[i] = new SensorOnHandGestureI(hand, Sensor.values()[i], parser);
        }
    }

    /**
     * Calculate average match of matches for all sensors
     * 
     * (non-Javadoc)
     * 
     * @see ProcessingApplet.Basic.Gesture#matchesBy(java.util.HashMap)
     */
    @Override
    public float matchesBy(Map<Date, GestLineData> data) {
        // because I have not used lambda in some time..
        // for (SensorOnHandGestureI sensorGesture : sensoresGestrues) {
        // float sensorMatch = sensorGesture.matches(data);
        return (float) Arrays.asList(sensoresGestrues).stream().map(x -> x.matchesBy(data)).mapToDouble(x -> x)
                .average().orElse(0);
    }

}

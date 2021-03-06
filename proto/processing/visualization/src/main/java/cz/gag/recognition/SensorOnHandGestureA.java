/*
Copyright (c) 2018 Vojtěch Průša
*/

package cz.gag.recognition;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.common.Hand;
import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestDataLine;

/**
 * @author Vojtech Prusa
 *
 * Used to extend class for loading data from reference file and recognizing hand gesture for each sensor 
 *
 */
public abstract class SensorOnHandGestureA implements SensorGesture, HandGesture {

    private Sensor sensor;
    private Hand hand;

    private DataFileParser<GestDataLine> parser;

    public ArrayList<GestDataLine> data;

    SensorOnHandGestureA(Hand hand, Sensor sensor) {
        this.sensor = sensor;
        this.hand = hand;
    }

    SensorOnHandGestureA(Hand hand, Sensor sensor, DataFileParser<GestDataLine> parser) {
        this(hand, sensor);
        this.parser = parser;
        loadData();
    }

    public Sensor getSensor() {
        return sensor;
    }

    @Override
    public Hand getHand() {
        return this.hand;
    }

    @Override
    public float matchesBy(Map<Date, GestDataLine> data) {
        // actually i should read from all values just those related to current
        // sensor...
        // data.
        return 0.5f;
    }

    public void loadData() {
        parser.reset();
        System.out.println("Loading ref data");
        data = new ArrayList<GestDataLine>();
        GestDataLine rl = null;
        while ((rl = parser.parseLine()) != null) {
            if (rl.sensor == sensor && rl.hand == hand) {
                data.add(rl);
            }
        }
    }

}

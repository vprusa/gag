package cz.gag.recognition;

import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.common.Hand;
import cz.gag.visualization.DataFileParser;
/*
Copyright (c) 2018 Vojtěch Průša
*/
import cz.gag.visualization.GestLineData;
import toxi.geom.Quaternion;

public abstract class SensorOnHandGestureA implements SensorGesture, HandGesture {

    private Sensor sensor;
    private Hand hand;
    
    private DataFileParser parser;
    
    public ArrayList<GestLineData> data;

    SensorOnHandGestureA(Hand hand, Sensor sensor) {
        this.sensor = sensor;
        this.hand = hand;
    }
    
    SensorOnHandGestureA(Hand hand, Sensor sensor, DataFileParser parser) {
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
    public float matchesBy(Map<Date, GestLineData> data) {
        // actually i should read from all values just those related to current
        // sensor...
        //data.
        return 0.5f;
    }
    
    public void loadData() {
        parser.reset();
        System.out.println("Loading ref data");
        data = new ArrayList<GestLineData>();
        GestLineData rl = null;
        while ((rl = (GestLineData) parser.parseLine(GestLineData.class)) != null) {
            if(rl.sensor == sensor && rl.hand == hand) {
                data.add(rl);
            }
        }
    }

}

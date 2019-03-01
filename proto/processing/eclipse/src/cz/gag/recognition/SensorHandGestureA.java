/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import cz.gag.global.Hand;

public abstract class SensorHandGestureA extends HandGestureA {

    private Sensor sensor;

    SensorHandGestureA(Hand hand, Sensor sensor) {
        super(hand);
        this.sensor = sensor;
    }

    public Sensor getSensor() {
        return sensor;
    }
}

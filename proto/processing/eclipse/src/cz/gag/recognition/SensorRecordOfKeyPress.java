/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.ArrayList;
import java.util.List;

import toxi.geom.Quaternion;

// TODO match missing classes for real-time handling of matching - with "sliding
// window" and related parameters, etc.

// TODO implements appropriate interfaces/extends appropriate abstract classes
class SensorRecordOfKeyPress {
    int key;
    Sensor sensor;
    public static final int AROUND_VALUES_MAXIMUM = 50;
    List aroundValues = new ArrayList<Quaternion>();
    // TODO ...
}

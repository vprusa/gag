/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import cz.gag.global.Hand;
import cz.gag.recognition.Sensor;

public class GestLineData extends LineData {

    public GestLineData(float[] quat, Sensor sensor, Hand hand) {
        super(null, quat, null, sensor, hand);
    }
    
}

/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import cz.gag.global.Hand;

public class GestLineData extends LineData {

    public GestLineData(float[] quat,int sensor, Hand hand) {
        super(null, quat, null, sensor, hand);
    }
    
}

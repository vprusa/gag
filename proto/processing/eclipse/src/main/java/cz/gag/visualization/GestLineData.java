/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.util.Date;

import cz.gag.common.Hand;
import cz.gag.recognition.Sensor;

/**
 * @author Vojtech Prusa
 *
 * Used in gesture recognition to as one record in gesture
 * TODO fix access to variables across inheritance
 */
public class GestLineData extends LineData {

    public GestLineData(Date date, float[] quat, Sensor sensor, Hand hand) {
        super(date, quat, null, sensor, hand);
    }
    
}

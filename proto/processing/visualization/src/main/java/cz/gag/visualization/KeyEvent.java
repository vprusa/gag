/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.util.Date;

import cz.gag.common.Configuration;

/**
 * @author Vojtech Prusa
 * 
 * TODO used in keyboard recognition learning alg. 
 * @KeyboardRecorder
 */
class KeyEvent {
    int key;
    Date time;

    // TODO add method that can get values in interval (related on speed of click?)
    public KeyEvent(int key) {
        this.time = new Date();
        this.key = key;
    }

    // terrible serialization..
    public String asString() {
        String reportDate = Configuration.df.format(time);
        return reportDate + " " + key;
    }
}

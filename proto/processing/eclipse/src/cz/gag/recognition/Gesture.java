/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.visualization.GestLineData;
import toxi.geom.Quaternion;

public interface Gesture {
    
    /**
     * This method consumes data and returns statistical match <0,1>
     * 
     * Also this method consumes HashMap<Date, Quaternion> parsed of Date and
     * quaternions
     * 
     * @param data
     * @return
     */
    float matchesBy(Map<Date, GestLineData> data);

    /**
     * If matches gesture
     * 
     * @param data
     * @return
     */
    default boolean matches(Map<Date, GestLineData> data) {
        return matchesBy(data) > getAllowedMatch();
    }

    float allowedMatch = 0.9f;

    default float getAllowedMatch() {
        return allowedMatch;
    }

}

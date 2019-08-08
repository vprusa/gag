/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import java.util.Date;
import java.util.HashMap;
import java.util.Map;

import cz.gag.common.Hand;
import cz.gag.visualization.GestDataLine;
import toxi.geom.Quaternion;

public class BothHandsGesture implements Gesture  {

    public WholeHandGestureI left;
    public WholeHandGestureI right;
    
    public BothHandsGesture(String file){
        right = new WholeHandGestureI(Hand.RIGHT,file);
        left = new WholeHandGestureI(Hand.LEFT,file);
    }
    
    @Override
    public float matchesBy(Map<Date, GestDataLine> data) {
        return 0.5f;
    }
    
}

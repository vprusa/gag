/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import cz.gag.common.Hand;

public interface HandGesture extends Gesture {
    
    /**
     * returns hand for which gesture matches
     * 
     * @return Hand of gesture
     */
    public Hand getHand();
}

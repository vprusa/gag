/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.recognition;

import cz.gag.common.Hand;

// public abstract class GestureA implements Gesture {}

public abstract class HandGestureA implements HandGesture {
    private Hand hand;

    HandGestureA(Hand hand) {
        this.hand = hand;
    }

    @Override
    public Hand getHand() {
        return hand;
    }

}

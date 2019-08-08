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
 *  Used in @RePlayer
 *  
 */
class ReplayDataLine extends DataLine {
    char[] fakeTeapotPacket = new char[21];

    ReplayDataLine(Date date, float[] quat, Sensor sensor, Hand hand) {
        super(date, quat, sensor, hand);

        // TODO (from times long before men started to use unit tests)
        // -> refactor inheritance and remove, keep the message in commits so whoever
        // finds it can be praised when mentioning this ??
        fakeTeapotPacket[0] = '*';
        fakeTeapotPacket[1] = 0x02;
        fakeTeapotPacket[18] = 0; // counter
        fakeTeapotPacket[19] = '\r';
        fakeTeapotPacket[20] = '\n';
    }
}
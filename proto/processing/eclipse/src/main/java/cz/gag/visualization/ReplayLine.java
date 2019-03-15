/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.util.Date;

import cz.gag.common.Hand;
import cz.gag.recognition.Sensor;

class ReplayLine extends LineData {
    char[] fakeTeapotPacket = new char[21];

    ReplayLine(Date date, float[] quat, Sensor sensor, Hand hand) {
        super(date, quat, sensor, hand);
        fakeTeapotPacket[0] = '*';
        fakeTeapotPacket[1] = 0x02;
        fakeTeapotPacket[18] = 0; // counter
        fakeTeapotPacket[19] = '\r';
        fakeTeapotPacket[20] = '\n';
    }
}
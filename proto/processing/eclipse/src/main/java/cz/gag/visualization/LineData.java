package cz.gag.visualization;

import java.util.Date;

import cz.gag.common.Configuration;
import cz.gag.common.Hand;
import cz.gag.recognition.Sensor;
import toxi.geom.Quaternion;

/**
 * @author Vojtech Prusa
 *
 * Used for storing information about sensor records (on speific hand)
 * 
 * TODO 
 *  - should also contain smth about MPU90** magnetometer
 *  - should also contain smth about vibration feedback? -> brainstorm
 */
public class LineData {
    public final Date date;
    public final float[] quat;
    public final Quaternion quatO;
    public final float[] acc;
    public final Sensor sensor;
    public final Hand hand;

    public LineData(Date date, float[] quat, float[] acc, Sensor sensor, Hand hand) {
        this.date = date;
        this.quat = quat;
        // TODO check array size == 4
        this.quatO = new Quaternion(quat[0], quat[1], quat[2], quat[3]);
        this.acc = acc;
        this.hand = hand;
        this.sensor = sensor;
    }

    public LineData(Date date, float[] quat, Sensor sensor, Hand hand) {
        this(date, quat, new float[] { 0, 0, 0 }, sensor, hand);
    }

    public String toString() {
        String report = "MPU " + hand + " f: " + sensor.ordinal() + acc!=null ? (" Acc:" + acc[0] + " " + acc[1] + " " + acc[2]
               ) : "" + " Gyro: " + quat[0] + " " + quat[1] + " " + quat[2] + " " + quat[3];
        return report;
    }

    public String toFileString() {
        //String reportData = date != null ? Configuration.dfDate.format(date) : "" + " " + (hand == Hand.LEFT ? "*" : "$") + " "
        //        + sensor.ordinal() + " " + quat[0] + " " + quat[1] + " " + quat[2] + " " + quat[3] + acc != null ? (" " + acc[0]
        //        + " " + acc[1] + " " + acc[2] ) : "";
        String reportData = (date != null ? Configuration.dfDate.format(date) : "") + " " + (hand == Hand.LEFT ? "*" : "$") + " "
                + sensor.ordinal() + " " + quat[0] + " " + quat[1] + " " + quat[2] + " " + quat[3] + " " + acc[0]
                + " " + acc[1] + " " + acc[2];
                
        return reportData;
    }

}

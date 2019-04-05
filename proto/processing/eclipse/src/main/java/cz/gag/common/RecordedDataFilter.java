/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.common;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cz.gag.recognition.Sensor;
import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestLineData;
import toxi.geom.Quaternion;

/**
 * @author Vojtech Prusa, Lukrecia Mertova
 *
 *         This class takes filepath as input and filters just necessary data
 *         out of it based on input parameters
 * 
 *         Input & output file line format:
 * 
 *         <YYYY-MM-DD_HH:mm:ss.SSS> <handId> <sensorId> <q0> <q1> <q2> <q3>
 *         <a0> <a1> <a2>
 * 
 *         g.e.:
 * 
 *         2019-01-08_11:55:39.386 * 1 0.70202637 -0.30603027 -0.31201172
 *         0.56225586 0.0 0.0 0.0
 *
 */
public class RecordedDataFilter extends DataFileParser<GestLineData> {
    // TODO add class variables for filtering data ... do not pass them via method parameters ...
    
    
    public static void main(String... args) {
        if (args.length > 1 && (new File(args[1])).exists()) {
            RecordedDataFilter dataFilter = new RecordedDataFilter(args[1]);
            // dataFilter.filter(); // TODO add parameters
        } else {
            System.err.println("Existing files path must be as first atribute!");
        }
    }

    public RecordedDataFilter(String file) {
        super(file);
    }

    class HistoryArrayList<T> extends ArrayList<T> {

        int historyCount = 25;

        public HistoryArrayList(int historyCount) {
            this.historyCount = historyCount;
        }

        @Override
        public boolean add(T e) {
            if (this.size() > historyCount) {
                this.remove(this.size() - 1);
            }
            return super.add(e);
        }
    }

    class BothHandsWrapper {
        // HandData leftHandData = new HandData(Hand.LEFT);
        // HandData rightHandData = new HandData(Hand.RIGHT);

        int historyCountPerSensorOnHand = 25;

        public BothHandsWrapper(int historyCountPerSensorOnHand) {
            this.historyCountPerSensorOnHand = historyCountPerSensorOnHand;
        }

        public Map<Hand, Map<Sensor, List<GestLineData>>> lastNDataOfEachSensorOfHand = new HashMap<Hand, Map<Sensor, List<GestLineData>>>();

        public BothHandsWrapper() {
            for (Hand h : Hand.values()) {
                HashMap<Sensor, List<GestLineData>> lastNDataOfEachSensor = new HashMap<Sensor, List<GestLineData>>();
                lastNDataOfEachSensorOfHand.put(h, lastNDataOfEachSensor);
                for (Sensor s : Sensor.values()) {
                    // Map<Sensor,List<GestLineData>>
                    lastNDataOfEachSensor.put(s, new HistoryArrayList<GestLineData>(historyCountPerSensorOnHand));
                }
            }
        }

        public void add(GestLineData line) {
            lastNDataOfEachSensorOfHand.get(line.hand).get(line.sensor).add(line);
        }

        public GestLineData getTop(Hand h, Sensor s) {
            if (lastNDataOfEachSensorOfHand.get(h).get(s).size() > 0)
                return lastNDataOfEachSensorOfHand.get(h).get(s).get(0);
            return null;
        }
    }

    List<GestLineData> filteredData = null;
    BothHandsWrapper bhw = null;

    /**
     * Pls document this method.. also with ideas, TODO, etc. Parametrize whatever
     * you seem useful
     * Using float type for samplesPerSensorPerSecond so i could have gestures with rate in minutes as well
     */
    private void filter(float samplesPerSensorPerSecond, boolean findEdges /* parameters */) {
        // TODO Lukrecias magic
        filteredData = new ArrayList<GestLineData>();
        bhw = new BothHandsWrapper(25);

        // use this.parseLine() for loading per line or anything else, although it would
        // be useful to have 2 implementation
        // 1. can consume whatever amount of ram
        // 2. should not load everything in memory at once but somehow store just last n
        // lines at most... for future real-time comparing? idk here
        GestLineData line = null;
        while ((line = this.parseLine()) != null) {
            // TODO Lukrecias magic
            bhw.add(line);

            if (this.isLineValid(line, samplesPerSensorPerSecond, findEdges)) {
                filteredData.add(line);
            }
        }
    }

    private boolean isLineValid(GestLineData line, float samplesPerSensorPerSecond, boolean findEdges) {
        // check if time matches filtered samples rate per sensor on hand
        if() {
            
        }
        
        // check if this value is edge value because then this and/or last one in
        // history of this sensor of hand has to added as well
            
        return true;
    }

    // TODO add methods that call filter(<parameters>) and name them smth like
    // filter<Basic|Minimal|Maximal|TimeOnly|etc.>(<parameters>) based on parameters
    // example
    private void filterBasic(/* less parameters */) {
        filter(5, false);
    }

}

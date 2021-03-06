/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.common;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;

import cz.gag.recognition.Sensor;
import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestDataLine;

/**
 * @author Vojtech Prusa
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
 *         TODO fix samplesPerSensorPerSecond - its messed up and do not
 *         contains values reliable considering variables name.. rename it?
 *
 */
public class RecordedDataFilter extends DataFileParser<GestDataLine> {
    // TODO add class variables for filtering data ... do not pass them via method
    // parameters ...
    public final static Logger log = Logger.getLogger(RecordedDataFilter.class.getSimpleName());

    public static void main(String... args) {
        if (args.length > 1 && (new File(args[1])).exists()) {
            RecordedDataFilter dataFilter = new RecordedDataFilter(args[1]);
            // dataFilter.filter(); // TODO add parameters
        } else {
            log.log(Level.SEVERE, "Existing files path must be as first atribute!");
        }
    }

    public RecordedDataFilter(String file) {
        super(file, GestDataLine.class);
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

        int historyCountPerSensorOnHand = 25;

        public Map<Hand, Map<Sensor, List<GestDataLine>>> lastNDataOfEachSensorOfHand = new HashMap<Hand, Map<Sensor, List<GestDataLine>>>();

        public Map<Hand, Map<Sensor, GestDataLine>> lastFilteredDataOfEachSensorOfHand = new HashMap<Hand, Map<Sensor, GestDataLine>>();

        public BothHandsWrapper(int historyCountPerSensorOnHand) {
            this.historyCountPerSensorOnHand = historyCountPerSensorOnHand;
            for (Hand h : Hand.values()) {
                HashMap<Sensor, List<GestDataLine>> lastNDataOfEachSensor = new HashMap<Sensor, List<GestDataLine>>();
                lastNDataOfEachSensorOfHand.put(h, lastNDataOfEachSensor);

                HashMap<Sensor, GestDataLine> lastFilteredDataOfEachSensor = new HashMap<Sensor, GestDataLine>();
                lastFilteredDataOfEachSensorOfHand.put(h, lastFilteredDataOfEachSensor);

                for (Sensor s : Sensor.values()) {
                    // Map<Sensor,List<GestDataLine>>
                    lastNDataOfEachSensor.put(s, new HistoryArrayList<GestDataLine>(historyCountPerSensorOnHand));
                    lastFilteredDataOfEachSensor.put(s, null);
                }
            }
        }

        public void add(GestDataLine line) {
            lastNDataOfEachSensorOfHand.get(line.hand).get(line.sensor).add(line);
        }

        public GestDataLine getTop(Hand h, Sensor s) {
            return get(h, s, 0);
        }

        public GestDataLine get(Hand h, Sensor s, int i) {
            int size = lastNDataOfEachSensorOfHand.get(h).get(s).size();
            if (i >= 0 && size > i)
                return lastNDataOfEachSensorOfHand.get(h).get(s).get(size - 1 - i);
            return null;
        }

        public GestDataLine getLastFiltered(Hand h, Sensor s) {
            return bhw.lastFilteredDataOfEachSensorOfHand.get(h).get(s);
        }

        public void setLastFiltered(GestDataLine line) {
            lastFilteredDataOfEachSensorOfHand.get(line.hand).put(line.sensor, line);
        }
    }

    List<GestDataLine> filteredData = null;
    BothHandsWrapper bhw = null;

    /**
     * Pls document this method.. also with ideas, TODO, etc. Parametrize whatever
     * you seem useful Using float type for samplesPerSensorPerSecond so i could
     * have gestures with rate in minutes as well
     */
    public void filter(float samplesPerSensorPerSecond, boolean findEdges /* parameters */) {
        // TODO Lukrecias magic
        filteredData = new ArrayList<GestDataLine>();
        // assuming that GestDataLine time has +- equal distribution rate per second it
        // sould be enough to multiple samplesPerSensorPerSecond with constant derived
        // from this rate ... g.e. 25 * 1 where 1 is sensor?
        bhw = new BothHandsWrapper((int) samplesPerSensorPerSecond * 2);

        // use this.parseLine() for loading per line or anything else, although it would
        // be useful to have 2 implementation
        // 1. can consume whatever amount of ram
        // 2. should not load everything in memory at once but somehow store just last n
        // lines at most... for future real-time comparing? idk here
        GestDataLine line = null;
        while ((line = this.parseLine()) != null) {
            // TODO Lukrecias magic
            if (this.isLineValid(line, samplesPerSensorPerSecond, findEdges)) {
                filteredData.add(line);
                bhw.setLastFiltered(line);
            }
            bhw.add(line);
        }
        int i = 1;
    }

    private boolean isLineValid(GestDataLine line, float samplesPerSensorPerSecond, boolean findEdges) {
        // check if time matches filtered samples rate per sensor on hand
        GestDataLine prevLine = bhw.getLastFiltered(line.hand, line.sensor);
        long timeToSkip = (long) (1000.0f * samplesPerSensorPerSecond);
        if (prevLine == null) {
            return true;
        }
        long lineDate = line.date.getTime();
        long prevLineDate = prevLine.date.getTime();
        long diffDates = lineDate - prevLineDate;
        if (diffDates > timeToSkip) {
            return true;
        }

        // TODO check if this value is edge value because then this and/or last one in
        // history of this sensor of hand has to added as well
        // using:
        // bhw.lastNDataOfEachSensorOfHand ..BothHandsWrapper.
        // bhw.get()
        // bhw.getTop(h, s)

        return false;
    }

    // TODO add methods that call filter(<parameters>) and name them smth like
    // filter<Basic|Minimal|Maximal|TimeOnly|etc.>(<parameters>) based on parameters
    // example
    public void filterBasic(/* less parameters */) {
        filter(0.5f, false);
    }

    public void saveFilteredToFile(String filePath) throws IOException {
        saveFilteredToFile(filePath, false);
    }

    public void saveFilteredToFile(String filePath, boolean append) throws IOException {
        File file = new File(filePath);
        FileWriter fr = new FileWriter(file, append);
        BufferedWriter br = new BufferedWriter(fr);

        Iterator<GestDataLine> iter = filteredData.iterator();
        while (iter.hasNext()) {
            br.write(iter.next().toFileString() + (iter.hasNext() ? '\n' : ""));
        }

        br.close();
        fr.close();
    }
}

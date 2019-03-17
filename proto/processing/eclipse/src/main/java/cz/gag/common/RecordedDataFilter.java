/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.common;

import java.io.File;

import cz.gag.visualization.DataFileParser;
import cz.gag.visualization.GestLineData;

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

    public static void main(String... args) {
        if (args.length > 1 && (new File(args[1])).exists()) {
            RecordedDataFilter dataFilter = new RecordedDataFilter(args[1]);
            dataFilter.filter(); // TODO add parameters
        } else {
            System.err.println("Existing files path must be as first atribute!");
        }
    }

    public RecordedDataFilter(String file) {
        super(file);
    }

    /**
     * Pls document this method.. also with ideas, TODO, etc. Parametrize whatever
     * you seem useful
     */
    private void filter(/* parameters */) {
        // TODO Lukrecias magic

        // use this.parseLine() for loading per line or anything else, although it would
        // be usefull to have 2 implementation
        // 1. can consume whatever amount of ram
        // 2. should not load everything in memory at once but somehow store just last n
        // lines at most... for future real-time comparing? idk here
        GestLineData line = null;
        while ((line = this.parseLine()) != null) {
            // TODO Lukrecias magic
        }
    }

    // TODO add methods that call filter(<parameters>) and name them smth like
    // filter<Basic|Minimal|Maximal|TimeOnly|etc.>(<parameters>) based on parameters
    // example
    private void filterBasic(/* less parameters */) {
        filter();
    }

}

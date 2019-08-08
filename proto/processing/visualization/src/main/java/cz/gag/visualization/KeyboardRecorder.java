/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import cz.gag.common.Configuration;

/**
 * @author Vojtech Prusa
 *
 * TODO used in keyboard recognition learning alg.
 */
public class KeyboardRecorder {
    List<KeyEvent> keys = new ArrayList<KeyEvent>();
    PrintWriter kout;

    public KeyboardRecorder(String file) {
        if (Configuration.logToFile) {
            try {
                kout = new PrintWriter(new FileWriter(file, true), true);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void recordKey(int key) {
        KeyEvent ke = new KeyEvent(key);
        // storing it for realtime statistics??
        keys.add(ke);
        kout.println(ke.asString());
    }
}

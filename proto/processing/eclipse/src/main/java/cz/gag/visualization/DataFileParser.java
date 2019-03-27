/*
Copyright (c) 2018 Vojtěch Průša
*/
package cz.gag.visualization;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.Date;

import cz.gag.common.Configuration;
import cz.gag.common.Hand;
import cz.gag.recognition.Sensor;

/**
 * @author Vojtech Prusa
 *
 * @param <T> todo fix?
 * 
 *        This class is used to handle parsing data files of row format
 * 
 *        <YYYY-MM-DD_HH:mm:ss.SSS> <handId> <sensorId> <q0> <q1> <q2> <q3> <a0>
 *        <a1> <a2>
 * 
 *        into LineData class base for further operations line replaying file,
 *        etc.
 * 
 *        Why to use generic class T: because then i can use LineDate children
 *        class instances to store just necessary data and/or implement specific
 *        methods in given context. TODO .. fix constructors
 * 
 */
public class DataFileParser<T extends LineData> {
    FileReader fr;
    BufferedReader br;

    String line;
    int replayChar = 0;
    boolean EOL = false;
    String file;
    Class lineDataClass = null;

    public DataFileParser(String file) {
        this.file = file;
        System.out.println(file);
        try {
            fr = new FileReader(file);
            br = new BufferedReader(fr);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    // TODO hack java with not using Class c in constructor .. because this is ugly.
    public DataFileParser(String file, Class c) {
        this(file);
        lineDataClass = c;
    }

    public T parseLine() {
        try {
            if ((line = br.readLine()) != null) {
                String[] words = line.split(" ");
                String startDateString = words[0];
                Date fakeDate = Configuration.dfDate.parse(startDateString);
                // String newDateString = df.format(fl.fakeDate);
                char hand = words[1].charAt(0);
                int sensor = Integer.parseInt(words[2]);

                float[] fakequaternionArr = new float[4];

                float[] fakequaternionODataArr = new float[4];

                for (int i = 0; i < 4; i++) {
                    fakequaternionArr[i] = Float.parseFloat(words[3 + i]);
                    fakequaternionODataArr[i] = fakequaternionArr[i];
                    if (fakequaternionArr[i] >= -2)
                        fakequaternionArr[i] = 4 + fakequaternionArr[i];
                    fakequaternionArr[i] = fakequaternionArr[i] * 16384.0f;

                }
                // Hand thisHand = hand == '*' ? Hand.LEFT : hand == '$' ? Hand.RIGHT : null;

                Hand thisHand = hand == '*' ? Hand.LEFT : Hand.RIGHT;
                // TODO fix hand
                // System.out.println( (this.getClass()
                // .getGenericSuperclass()).getTypeName());
                /*
                 * try { java.lang.reflect.Method foo = this.getClass().getMethod("parseLine");
                 * Class<?> type = foo.getReturnType(); System.out.println("Return Type: " +
                 * type.getName()); } catch (NoSuchMethodException | Sec urityException e) {
                 * e.printStackTrace(); }
                 */
                // System.out.println("Return Type: " + c.getName());
                if (lineDataClass != null) {
                    if (lineDataClass.equals(ReplayLine.class)) {
                        return (T) new ReplayLine(fakeDate, fakequaternionODataArr, Sensor.values()[sensor], thisHand);
                    } else if (lineDataClass.equals(GestLineData.class)) {
                        return (T) new GestLineData(fakeDate, fakequaternionODataArr, Sensor.values()[sensor],
                                thisHand);
                    }
                }
                return (T) new LineData(fakeDate, fakequaternionODataArr, Sensor.values()[sensor], thisHand);

                // return new (fakeDate, fakequaternionODataArr, Sensor.values()[sensor],
                // thisHand);
            } else {
                return null;
            }
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
        return null;
    }

    public void reset() {
        try {
            fr = new FileReader(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        br = new BufferedReader(fr);
    }

}

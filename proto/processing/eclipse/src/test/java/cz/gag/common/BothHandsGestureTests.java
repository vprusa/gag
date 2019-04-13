package cz.gag.common;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.logging.Logger;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import cz.gag.recognition.BothHandsGesture;

public class BothHandsGestureTests {

    public final static Logger log = Logger.getLogger(BothHandsGestureTests.class.getSimpleName());

    BothHandsGesture bhg;

    String testFilePath = System.getProperty("user.dir")
            + "/src/test/resources/test-output-basic-data-for-filter-1.log";
    String testRefFilePath = System.getProperty("user.dir")
            + "/src/test/resources/test-output-basic-data-for-filter-1-filtered.log";

    @BeforeEach
    public void setUp() {
    }

    @Test
    public void SimpleDataFilterTest() {
        boolean passedCatch = false;
        try {
            bhg = new BothHandsGesture(testRefFilePath);
            passedCatch = true; // TODO fix this stupid way..
        } catch (Exception e) {
            e.printStackTrace();
        }
        assertTrue(passedCatch, "Should not happen");
    }

}

package cz.gag.common;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.logging.Logger;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RecordedDataFilterTests {

    public final static Logger log = Logger.getLogger(RecordedDataFilterTests.class.getSimpleName());

    RecordedDataFilter rdf;

    String testFilePath = System.getProperty("user.dir")
            + "/src/test/resources/test-output-basic-data-for-filter-1.log";
    String testOutFilePath = System.getProperty("user.dir")
            + "/src/test/resources/test-output-basic-data-for-filter-1-filtered.log";

    @BeforeEach
    public void setUp() {
        rdf = new RecordedDataFilter(testFilePath);
    }

    @Test
    public void SimpleDataFilterTest() {
        boolean passedCatch = false;
        try {
            rdf.filterBasic();
            rdf.saveFilteredToFile(testOutFilePath);
            passedCatch = true; // TODO fix this stupid way..
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        assertTrue(passedCatch, "Should not happen");
    }

}

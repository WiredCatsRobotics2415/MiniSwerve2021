package frc.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import java.util.List;
import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.util.logging.CSVSaver;

public class CSVSaverTest {
    @Test
    public void CSVWriteTest() {
        try {
            List<double[]> data = new ArrayList<double[]>();
            double[] array = new double[3];
            array[0] = 1;
            array[1] = 2;
            array[2] = 3;
            data.add(array.clone());
            array[0] = 1.5;
            array[1] = 2.5;
            array[2] = 3.5;
            data.add(array.clone());
            File file = new File("csv_test.csv");
            file.deleteOnExit();
            CSVSaver.saveFile(file, data);
            Scanner scan = new Scanner(file);
            assertEquals("1.0,2.0,3.0", scan.nextLine());
            assertEquals("1.5,2.5,3.5", scan.nextLine());
        } catch (FileNotFoundException e) {
            assertEquals(0, 1);
        }
    }
}
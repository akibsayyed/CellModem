/*
 * Simple program to test modulator
 *
 * Use Audacity to import and view waveform
 *
 * Frames are in little-endian format
 * 16 bit signed 8 kHz rate
 *
 * Licensed under GNU LGPL V3.0
 * See LICENSE file for information
 *
 * Java Version by Object Toolworks 2019
 */
package jpair;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Random;

public class Main {

    private static short[] Frame;
    private static byte[] Data;
    private static FileOutputStream fout;
    private static Random rand;

    public static void main(String[] args) {
        Frame = new short[720];
        Data = new byte[9];         // 72 bits

        rand = new Random();
        rand.setSeed(System.currentTimeMillis());

        /*
         * Create modem instance
         */
        PulseModem modem = new PulseModem();

        try {
            fout = new FileOutputStream("/tmp/waveform.raw");
        } catch (FileNotFoundException | SecurityException e0) {
            System.err.println("Fatal: Couldn't open raw output file: /tmp/waveform.raw");
            System.exit(-1);
        }

        /*
         * record a random long audio so we can see what it sounds like
         */
        for (int k = 0; k < 50; k++) {
            /*
             * Create data byte array
             */
            rand.nextBytes(Data);

            for (int j = 0; j < 9; j++) {
                System.out.printf("%02X ", Data[j]);
            }
            
            System.out.println();
            
            modem.modulate(Data, Frame);

            try {
                for (int i = 0; i < Frame.length; i++) {
                    fout.write(Frame[i] & 0xFF);            // LSB
                    fout.write((Frame[i] >>> 8) & 0xFF);    // MSB
                }
            } catch (IOException e1) {
                System.err.println("Fatal: Couldn't write to audio file " + e1.toString());
                System.exit(-1);
            }
        }
    }
}

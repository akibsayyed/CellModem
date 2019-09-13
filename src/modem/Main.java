/*
 * Simple program to test modulator
 *
 * Use Audacity to import and view waveform
 *
 * Frames are in little-endian format
 * 16 bit signed 48 kHz rate
 *
 * Licensed under GNU LGPL V3.0
 * See LICENSE file for information
 *
 * Object Toolworks 2019
 */
package modem;

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
        Frame = new short[3240];
        Data = new byte[12];
        
        rand = new Random();
        rand.setSeed(System.currentTimeMillis());

        /*
         * Create modem instance
         */
        Modem modem = new Modem();

        try {
            fout = new FileOutputStream("/tmp/waveform.raw");
        } catch (FileNotFoundException | SecurityException e0) {
            System.err.println("Fatal: Couldn't open raw output file: /tmp/waveform.raw");
            System.exit(-1);
        }

        /*
         * record a long audio so we can see what it sounds like
         */
        for (int k = 0; k < 100; k++) {
            /*
             * Create data byte array
             */
            rand.nextBytes(Data);

            Data[10] = 1;       // bit 81 is LSB
            Data[11] = 0;       // blank it

            modem.modulate(Data, Frame);

            try {
                for (int i = 0; i < Frame.length; i++) {
                    fout.write(Frame[i] & 0xFF);            // LSB
                    fout.write((Frame[i] >>> 8) & 0xFF);    // MSB
                }
            } catch (IOException e1) {
                System.err.println("Fatal: Couldn't write to audio file");
                System.exit(-1);
            }
        }

        /*
         * See if we can demodulate the frame (not likely with this small test)
         */
//        int offset = 0;
//
//        while (true) {
//            int i = modem.demodulate(frame, offset, data);
//            offset += i;
//
//            if ((data[11] & (byte) 0x80) == 1) {
//                break;
//            }
//        }
//
//        for (i = 0; i < data.length; i++) {
//            System.out.printf("%02X ", data[i]);
//        }
//
//        System.out.println();
    }
}

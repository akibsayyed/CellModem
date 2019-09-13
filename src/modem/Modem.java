/*
 * Copyright (C) 2016 Van Gegel
 *
 * All rights reserved
 *
 * Licensed under GNU LGPL V3.0
 * See LICENSE file for information
 *
 * Java Version by Object Toolworks 2019
 */
package modem;

/*
 * Algorithms to modulate and demodulate data using
 * a pseudo-voice modem, suitable for GSM-FR compressed channel
 *
 * This is a BPSK modem with carrier parameters strongly adapted
 * for the GSM FR codec engine using Voice Activity Detection (VAD)
 */
public final class Modem {

    private static final byte[] MASK = new byte[]{(byte) 1, (byte) 2, (byte) 4, (byte) 8,
        (byte) 16, (byte) 32, (byte) 64, (byte) 128};

    /*
     * The waveforms are 36 samples each for 48 kHz sample rate
     */
    private static final short[][] WAVE = new short[][]{
        // Normal
        // bit 0
        {0, 2778, 5472, 7999, 10284, 12256, 13856, 15035, 15756,
            16000, 15756, 15035, 13856, 12256, 10284, 7999, 5472, 2778,
            0, -2778, -5472, -7999, -10284, -12256, -13856, -15035, -15756,
            -16000, -15756, -15035, -13856, -12256, -10284, -8000, -5472, -2778
        },
        // bit 1
        {0, -2778, -5472, -7999, -10284, -12256, -13856, -15035, -15756,
            -16000, -15756, -15035, -13856, -12256, -10284, -8000, -5472, -2778,
            0, 2778, 5472, 7999, 10284, 12256, 13856, 15035, 15756,
            16000, 15756, 15035, 13856, 12256, 10284, 7999, 5472, 2778
        },
        // Shaped
        // bit 0
        {0, 244, 965, 2144, 3744, 5716, 8001, 10528, 13222,
            16000, 13222, 10528, 8001, 5716, 3744, 2144, 965, 244,
            0, -244, -965, -2144, -3744, -5716, -8001, -10528, -13222,
            -16000, -13222, -10528, -8001, -5716, -3744, -2144, -965, -244
        },
        // bit 1
        {0, -244, -965, -2144, -3744, -5716, -8001, -10528, -13222,
            -16000, -13222, -10528, -8001, -5716, -3744, -2144, -965, -244,
            0, 244, 965, 2144, 3744, 5716, 8001, 10528, 13222,
            16000, 13222, 10528, 8001, 5716, 3744, 2144, 965, 244
        },
        // Muted
        // bit 0
        {0, 1389, 2736, 3999, 5142, 6128, 6928, 7517, 7878,
            8000, 7878, 7517, 6928, 6128, 5142, 3999, 2736, 1389,
            0, -1389, -2736, -3999, -5142, -6128, -6928, -7517, -7878,
            -8000, -7878, -7517, -6928, -6128, -5142, -4000, -2736, -1389
        },
        // bit 1
        {0, -1389, -2736, -3999, -5142, -6128, -6928, -7517, -7878,
            -8000, -7878, -7517, -6928, -6128, -5142, -4000, -2736, -1389,
            0, 1389, 2736, 3999, 5142, 6128, 6928, 7517, 7878,
            8000, 7878, 7517, 6928, 6128, 5142, 3999, 2736, 1389
        },
        // Shaped and muted
        // bit 0
        {0 / 2, 244 / 2, 965 / 2, 2144 / 2, 3744 / 2, 5716 / 2, 8001 / 2, 10528 / 2, 13222 / 2,
            16000 / 2, 13222 / 2, 10528 / 2, 8001 / 2, 5716 / 2, 3744 / 2, 2144 / 2, 965 / 2, 244 / 2,
            0 / 2, -244 / 2, -965 / 2, -2144 / 2, -3744 / 2, -5716 / 2, -8001 / 2, -10528 / 2, -13222 / 2,
            -16000 / 2, -13222 / 2, -10528 / 2, -8001 / 2, -5716 / 2, -3744 / 2, -2144 / 2, -965 / 2, -244 / 2
        },
        // bit 1
        {0 / 2, -244 / 2, -965 / 2, -2144 / 2, -3744 / 2, -5716 / 2, -8001 / 2, -10528 / 2, -13222 / 2,
            -16000 / 2, -13222 / 2, -10528 / 2, -8001 / 2, -5716 / 2, -3744 / 2, -2144 / 2, -965 / 2, -244 / 2,
            0 / 2, 244 / 2, 965 / 2, 2144 / 2, 3744 / 2, 5716 / 2, 8001 / 2, 10528 / 2, 13222 / 2,
            16000 / 2, 13222 / 2, 10528 / 2, 8001 / 2, 5716 / 2, 3744 / 2, 2144 / 2, 965 / 2, 244 / 2
        }
    };

    private final double[] g_fr;        // weights of lags
    private final double[] g_fd;        // metrics of bits
    private final double[][] g_ffg;

    private double g_qq;                // quality of fixing 'thin" lag   (bit aligned - phase lock)
    private double g_f180;              // average delta Pi phase value
    private double g_falign;            // average raw BER as a quality of frame synch

    private final int[] g_r;            // shifted symbols for parity check
    private int g_rr;                   // shifted parity bits

    private int g_lag;                  // lag of block (last bit position in the stream)
    private int g_cnt;                  // counter of PCM frames in block
    private int g_u;                    // average DC level
    private int g_lastb;                // value of the last transmitted bit for ISI
    private int g_oldq;                 // 'thin' phase correction value of last processed frame

    private boolean g_vadtr;            // flag of the periodic whole frame muting for anti-VAD trick
    private boolean g_blk;              // block ready flag
    private boolean g_lock;             // carrier locked flag
    private boolean g_align;            // frame synchronization locked

    public Modem() {
        g_fr = new double[90];
        g_fd = new double[90];
        g_ffg = new double[4][36];
        g_r = new int[9];

        g_vadtr = false;
        g_blk = false;
        g_lock = false;
        g_align = true;

        g_falign = 50.0;
    }

    /**
     * Modulator for BPSK 1333 Hz carrier, with asymmetric, ISI, and periodic
     * muting
     *
     * Modulate 81 bits in byte array to 3240 short PCM samples
     *
     * @param data 81 data bits in data[0]..data[10](LSB)
     * @param frame 3240 PCM Sign+15-bit samples
     */
    public void modulate(byte[] data, short[] frame) {
        int i, j, k, b, ii;
        int sp;                         // pointer to PCM (16-bit) frame
        byte[] p = new byte[9];         // 9 symbol parity bit array
        boolean isi;                    // flag of controlled ISI

        /*
         * Modulator: 81 bits in 9 symbols, add parity bit to each symbol
         * Interleaves symbols: transmit bits 0 of each symbol,
         * then bits 1, ... last parity bits
         */
        sp = 0;                         // initialize pointer
        p[8] = 1;                       // parity is odd for last symbol

        for (i = 0; i < 10; i++) {      // output bit counter: 10 bits per symbol
            for (j = 0; j < 9; j++) {   // output symbol counter: 9 symbols
                if (i == 9) {
                    b = p[j];           // use parity bit (the last in symbol)
                } else {
                    k = j * 9 + i;      // input bit number (0..80)

                    if ((data[k / 8] & MASK[k % 8]) != 0) {
                        b = 1;
                    } else {
                        b = 0;          // use input bit
                    }

                    p[j] ^= b;          // add bit to parity
                }

                // check for current bit is different previous bit
                isi = ((b ^ g_lastb) != 0);
                g_lastb = b;            // save current bit for next

                if (isi == true) {
                    b += 2;             // ISI trick: shaping waveform
                }

                if (g_vadtr == true) {
                    b += 4;                         // VAD trick: muting waveform
                }

                for (ii = 0; ii < 36; ii++) {
                    frame[sp++] = WAVE[b][ii];      // modulate bit to the 36 samples PCM waveform
                }

                sp -= 36;

                for (ii = 0; ii < 18; ii++) {
                    frame[sp++] /= 2;               // applies asymmetric amplitude to wave
                }

                sp += 18;                           // move PCM 16-bit pointer to the next bit position
            }
        }

        /*
         * Change muting flag for next frame
         * 67.5 mS normal, 67.5 mS muted
         */
        g_vadtr = !g_vadtr;
    }

    /*
     * Demodulate PCM Samples to data bits and status byte
     *
     * Input: must be at least 36 * 7 samples
     * Output: 81 bits of data in data[0]..data[10](LSB) (valid when ready flag is set)
     *
     * 7 MSB of data[10] is synchronization lag (0-90)
     * 4 LSB of data[11] is number of symbol errors in block (0-9)
     *
     * MSBs of data[11] are:
     *
     * Bit 7 is a flag of payload ready (receiving event)
     * Bit 6 is a flag of block synchronization locked
     * Bit 5 is a flag of phase locked
     * Bit 4 is a flag of frequency locked
     *
     * Returns the number of processed samples
     */
    public int demodulate(short[] frame, int offset, byte[] data) {
        double[] spn = new double[36];      // normalized waveform
        double f, ge, ffg0, ffg1;
        int[] eg = new int[36];             // correlation for each sample during period of carrier
        int i, j, k, b, p, pj;
        int ii, jj, kk, ll, bb, bbb;
        
        int pp = 0;
        int lastbit = 0;
        int q = 0;

        /*
         * coherent demodulator of BPSK 1333 Hz carrier
         * (36 cycles of 48 kHz PCM samples per bit)
         *
         * output 81 bits aligned to block of 67.5 mS (6 * 540 PCM 48 kHz samples)
         *
         * First we correlate input stream with square signal 1333Hz
         * for phase locking and frequency adjustment
         *
         * The phase corrects with step +/= 2pi / 36
         * (we have 36 48KHz PCM samples per carrier period)
         *
         * Frequency correction by skipping/doubling samples in input stream
         *
         * To tune frequency, first transform phase jitter to wander
         *
         * Time of this filter is adaptive with absolute sampling rate difference
         * of modulator / demodulator
         */
        int sp = offset + 9;                    // pointer with space for 'thin' phase correction

        /*
         * Correlate 36 * 6 samples with 1333 Hz (carrier frequency) by shifting steps Pi / 18
         */
        for (i = 0; i < 36; i++) {
            eg[i] = 0;                          // clear for new correlation results
        }

        for (i = 0; i < 24; i++) {              // look for 4 subframes (24 periods)
            for (j = 0; j < 36; j++) {          // each period (36 samples)
                k = i * 36 + j;                 // pointer for next step

                // averages the results of multiplying with square signal
                eg[j] += Math.abs(frame[sp + k] - frame[sp + k + 18]);
            }
        }

        // search the best (probably correct) phase for this frame
        k = 0;

        for (j = 0; j < 36; j++) {            // 36 possibles shifts with step PI/18
            if (eg[j] > k) {                // search best correlation value
                k = eg[j];                // best value
                q = j;                // best phase pointer
            }
        }

        // search correct Pi shifting lag of continuous input stream
        g_f180 *= 0.9;        // average +/- PI shifting lag

        if (q > 17) {            // +PI the best
            q -= 18;            // shift stream back
            g_f180 -= 1.0;            // averages
        } else {            // the result is positive for lag 0 and negative for lag Pi
            g_f180 += 1.0;
        }

        if (Math.abs(g_f180) < 1.0) {            // no good carrier detected
            if (g_lock == true) {
                for (i = 0; i < 90; i++) {
                    g_fr[i] = 0.0;                    // lost carrier: clear lags array
                }
            }

            g_lock = false;                     // clear lock flag
        } else if (Math.abs(g_f180) > 9.0) {
            g_lock = true;                      //carrier excellent: set lock flag
        }

        // lock the phase
        q -= 9;

        if (g_f180 < 0.0) {
            sp += 18;            // correct frame pointer to actual Pi phase lag
        }

        sp += q;        // correct frame pointer to actual Pi/18 phase lag

        /*
         * frequency tuning (by sampling rate difference between modulator and demodulator)
         * The first transform phase jitter to wander
         */
        g_qq *= 0.999;                          // averages quality of 'thin' synch

        if (g_oldq == q) {
            g_qq += 1.0;                        // phase not changed compared previous frame: increase synch quality
        } else {
            g_oldq = q;                         // store current phase (lag)
        }

        /*
         * now input is 36*6 samples frame aligned to correct phase point
         * and can be demodulated coherently
         */
        data[11] &= 0x7F;        // clear data ready flag on output bytes array

        // process 6 * 2 triplets (6*6 samples to bit and 6 bits in 36*6 samples)
        for (k = 0; k < 6; k++) {
            pj = k * 36;            // pointer to first sample in current triplet of processed frame

            // Average DC level of input stream: add current period
            g_u *= 504;

            for (i = 0; i < 36; i++) {
                g_u += (int) frame[sp + pj + i];
            }

            g_u /= 540;

            /*
             * To compensate for channel characteristics we must use
             * equalizing coefficients during correlation
             *
             * These coefficients are dynamically updated depending channel statistics
             *
             * The updates correspond to the GSM codecs frame and is set empirically
             *
             * Note: The modulator applied controlled ISI: the bit just changed
             * producing wave period applying filter
             *
             * Demodulator must equalize this pre-distortion based on the previously
             * received bit
             */
            for (i = 0; i < 36; i++) {
                spn[i] = (double) (frame[sp + pj + i] - g_u);           // eliminate DC
            }

            /*
             * correlation with 4 dynamic tables depend on previous received
             * and expected current bit
             */
            ffg0 = 0.0;            // correlation for case current bit equal last bit
            ffg1 = 0.0;            // not equal

            for (i = 0; i < 36; i++) {
                ffg0 += spn[i] * g_ffg[0][i]; // with 0 adaptive table 0
            }

            for (i = 0; i < 36; i++) {
                ffg0 -= spn[i] * g_ffg[1][i]; // with 1 adaptive table 0
            }

            for (i = 0; i < 36; i++) {
                ffg1 += spn[i] * g_ffg[2][i];       // with 0 adaptive table 1
            }

            for (i = 0; i < 36; i++) {
                ffg1 -= spn[i] * g_ffg[3][i];       // with 1 adaptive table 1
            }

            if (Math.abs(ffg1) > Math.abs(ffg0)) {
                ffg0 = ffg1;                        // maximal absolute correlation
            }

            if (ffg0 >= 0.0) {
                bbb = 0;
            } else {
                bbb = 1;                            // hard bit decision
            }

            lastbit = bbb + ((lastbit ^ bbb) << 1); //index of selected correlation table
            f = 0.0;

            for (i = 0; i < 36; i++) {              // 36 coefficients (for 36 samples) in a carrier period
                g_ffg[lastbit][i] *= 0.95;    // adjusting time corresponds GSM codec properties
                g_ffg[lastbit][i] += (double) (frame[sp + pj + i] - g_u); // average equalizing coefficients
                f += Math.abs(g_ffg[lastbit][i]); // averages amplitude of period
            }

            f /= 48.0;

            if (f == 0.0) {
                f = 1.0;                    // prevention division by zero on start
            }

            ffg0 /= f;                      // normalizing

            b = lastbit = bbb;              // store current bit for next

            /*
             * Now we have a received bit and can be check parity of received
             * data
             *
             * So each block contains 540*6 48KHz samples (15 frames 36*6
             * samples/6 bits each) we have 90 bits total and we split them to
             * 9 symbols of 10 bits each.
             *
             * Each symbol contain 9 data bits and 1 parity bit The parity bit
             * of the last symbol in the block is odd, others are even
             *
             * Before transmission, the data bits are interleaved: First
             * transmit bit 0 of all 9 symbols, follow bits 1... and then the
             * parity bits
             *
             * During receive we will check parity of previously received
             * 89 bits so assuming a current bit as a last bit in the block
             *
             * The result is comparing with correct parity word 000000001
             * complete only block fully received. For checking the level of
             * correctness we can compute the number of matching parity bits
             * So we average all possible bit-aligned lags (90 positions)
             * during bit stream received and can find the best lag pointed
             * the position of the last bit in block This synchronization will
             * probably be correct after 2 full blocks and we can output the
             * first correct block after maximum 270 mS of stream processing
             * starting at any time without any synch-sequences or other bit
             * rate overheads "thick" synch using parity bits
             */
            j = g_cnt * 6 + k;                      // current stream lag (aligned to bit)
            i = j % 9;                              // the virtual number of probably symbol in block
            g_r[i] = (g_r[i] << 1) | b;             // push received bit to virtual symbol

            //check parity of 10-bits symbol for now
            p = 0x3FF & g_r[i];
            p ^= (p >> 1);
            p ^= (p >> 2);
            p ^= (p >> 4);
            p ^= (p >> 8);
            g_rr = (g_rr << 1) | (p & 1);           // push probable parity bit to virtual parity word

            //averages matches parity block
            if (g_lock == true) {
                g_fr[j] *= 0.999;                   // average weight of current lag
            } else {
                g_fr[j] *= 0.99;                    // for fast synch while phase not locked good
            }

            if ((g_rr & 1) == 1) {                         // if current parity 1 (probably the end of packet)
                // calculate the number of previous zero parity (must be 8)
                p = g_rr;

                for (i = 0; i < 8; i++) {
                    p >>= 1;

                    if ((p & 1) == 0) {
                        g_fr[j] += 1.0;             // check for zero and add to current position lag metrics array
                    }
                }
            }

            i = (j - g_lag) - 1;                    // current bit number in block

            if (i < 0) {
                i += 90;                            // over boundaries of block
            }

            /*
             * compute common energy of bit for correlation:
             * for first and second half-period of BPSK waveform
             * computes LLR
             */
            ge = 0.0;                               // common energy of bit for normalization

            for (ii = 0; ii < 36; ii++) {
                ge += (spn[ii] * spn[ii]);
            }

            ge = Math.sqrt(ge);

            if (ge < 1.0) {
                ge = 1.0;                           // prevent divide by zero
            }

            g_fd[i] = (ffg0 / ge);                  // soft metric of bit (LLR)

            if (j == g_lag) {                       // this was the last bit of packet: output packet
                for (ll = 0; ll < 12; ll++) {
                    data[ll] = 0;                   // clear bytes output
                }

                kk = 0;                             // clear output bits counter
                bb = 0;                             // clear BER counter

                for (ii = 0; ii < 9; ii++) {        // for input symbol counter
                    // ii is number of 9-bits symbol (0-8)
                    f = 100000.0;                   // maximal possible metric value for fec

                    if (ii == 8) {
                        p = 1;
                    } else {
                        p = 0;                      // init parity bit (parity in last symbol is odd for syn purpose)
                    }

                    //note: input bit-stream is interleaved, de-interleave there
                    for (jj = 0; jj < 10; jj++) {   // for input bit counter in symbol
                        //jj is number of bit in symbol (0-8 info and 9 is parity)
                        j = jj * 9 + ii;            // pointer to input stream bit

                        if (g_fd[j] > 0.0) {
                            b = 1;
                        } else {
                            b = 0;                  // hard decision of the bit
                        }

                        if (Math.abs(g_fd[j]) <= f) {   // check for input bit with minimal metric in symbol
                            f = Math.abs(g_fd[j]);      // this is a minimal metric
                            pp = kk;                    // output position of bit with minimal metric
                        }

                        // output information bits in symbols and check last parity bit
                        if (jj < 9) {                   // this is information bit: output it
                            if (b == 1) {
                                data[kk / 8] ^= MASK[kk % 8]; // store in bytes output
                            }

                            p ^= b;                     // add to parity
                            kk++;                       // move output pointer to next bit
                        } else if (b != p) {            // this is parity bit (last in symbol)
                            // if parity is wrong

                            bb++;                       // count bit error

                            if (pp != kk) {
                                data[pp / 8] ^= MASK[pp % 8];        // flip bit with minimal metric
                            }
                        }
                    }
                }

                data[11] = (byte) bb;                   // output raw BER counter

                g_falign *= 0.9;                        // averages raw BER
                g_falign += bb;                         // add the BER of current block to average raw BER value

                if ((g_falign > 40.0) && (g_align == true)) {     // check for raw BER is hight and synch flag was set: probably lost of frame synch
                    g_align = false;                    // clear synch flag

                    for (i = 0; i < 90; i++) {
                        g_fr[i] = 0.0;                  // the lost of synch: clear lags metrics
                    }

                    for (i = 0; i < 36; i++) {          // set default dynamic equalizer
                        g_ffg[0][i] = WAVE[0][i];       // for bit 0
                        g_ffg[1][i] = WAVE[1][i];       // for bit 1
                        g_ffg[2][i] = g_ffg[0][i];      // copy for case expected bit not equal previous bit received
                        g_ffg[3][i] = g_ffg[1][i];
                    }
                } else if ((g_falign < 30.0) && (g_align == false)) {
                    g_align = true;                     // small BER: set synch flag
                }

                /*
                 * Now we have full block of payload ready for output
                 */
                if (g_blk == false) {
                    // output block only once per time: check flag
                    /*
                     * software must check this flag after every demodulator process
                     * and read 81 bit of output data if flag is set
                     * flag will be clear by demodulator in next processing cycle
                     */

                    data[11] |= 0x80;           // set ready flag (receiving event) for output this block
                    g_blk = true;               // set flag: block has been output
                }
            }
        }

        // frame processed: prepares for next frame
        g_cnt++;        // counter of frames 36*6 samples each

        /*
         * We split incoming stream to virtual blocks of 540 * 6 samples
         * for cryptographic strong synchronization properties
         */
        if (g_cnt >= 15) {

            // 15 frame processed
            g_cnt = 0;            // reset frame counter

            // correct lag: search best value for alignment to block boundaries
            f = 0.0;            // minimal possible value

            for (i = 0; i < 90; i++) {
                if (f < g_fr[i]) {                // search for all possible lags
                    f = g_fr[i];                  // new maximal value
                    g_lag = i;                    // the best lag
                }
            }

            /*
             * We must output one block every 67.5 mS
             *
             * Immediately after, last bit of block will be received
             *
             * If lag condition was not matched during last block
             * we must still output incorrect block for synchronization
             */
            // check block has been output per this 540*6 samples frame
            if (g_blk == false) {
                data[11] |= 0x8F;             // if no output incorrect block (continuous block counting)
            } else {
                g_blk = false;                // clear flag for new frame
            }
        }

        // add status to output data array
        data[10] += (g_lag << 1);        // 7 MSB of byte 10 is actual lag of block (0-89)

        if (g_align == true) {
            data[11] |= 0x40;            // block lag lock flag
        }

        if (g_lock == true) {
            data[11] |= 0x20;            // phase lock flag
        }

        if (g_qq > 50.0) {
            data[11] |= 0x10;            // frequency lock flag
        }

        /*
         * returns the number of actually processed samples (corrected for frequency adjustment)
         * Software must use this value to move sample pointer for next processing cycle
         */
        return 216 + q;
    }
}

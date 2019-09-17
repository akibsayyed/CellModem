/*
 * Copyright (C) June, 2018 Van Gegel <http://torfone.org>
 *
 * All rights reserved
 *
 * Licensed under GNU LGPL V3.0
 * See LICENSE file for information
 *
 * 800 bps pulse modem V3.1
 *
 * Java Version by Object Toolworks 2019
 */
package jpair;

/*
 * This file contains the methods for modulate and demodulate of data using
 * a pseudo-voice pulse modem suitable for AMR and GSM_FR compressed channels.
 *
 * This version has carrier parameters strongly adapted for codec engine with
 * VAD. Some tricks are empirically discovered and can be not optimal.
 *
 * Modulator:
 * Input frame is 9 bytes (72 data bits) split into 18 subframes with
 * 4 data bits in each. For each subframe, one parity bit is also added so one
 * subframe contains 5 bits total and 18 * 5 = 90 raw bits to be modulated.
 *
 * During modulation, it first outputs the 18 bit 0's of all 18 subframes,
 * then bits 1's, and then last, the 18 parity bits. Thus an interleave.
 *
 * 90 modulated bits split into 30 symbols 3 bits each. Each symbol contains
 * 24 8 kHz PCM samples, so whole frame contains 720 samples (90 mS).
 *
 * Each symbol contains one pulse wave. This pulse is placed in one of four
 * possible sample positions, using 2 of the bits for coding:
 *
 *    0           1             2              3
 *  0,1,2,x,x,x,6,7,8,x,x,x,12,13,14,x,x,x,18,19,20,x,x,x
 *
 * The third bit is used to set the polarity: 0 for positive, 1 for negative.
 *
 * Demodulator:
 *
 * First the demodulator computes the energy of the frame as a difference of
 * 0-3, 1-4, 2-5 ... samples over whole frame (for normalisation of correlation
 * coefficients)
 *
 * After this, all 720 samples will be processed sample by sample.
 *
 * While next sample processes we consider that this is the first sample of
 * symbol and provide fast detecting of symbol's hard bits.
 *
 * Four correlation coefficients will be computed for each possible pulse
 * position. For fast we search absolute greats coefficient and get 2 bits
 * depends position and third bit depend sign (polarity).
 *
 * Note: only one of 24 subsequent symbols (in real start position) will be
 * correct, other will be wrong.
 *
 * We store symbols separately for all 24 positions in 24 arrays.
 *
 * After symbol was processed we suppose this was a last symbol in the frame.
 * So we check parity of all subframes in array and compute number of parity
 * errors. We add this number to one of 720 FIFOs correspond current position
 * of sample in the frame.
 *
 * Only one position will be best -  the start of last symbol in the frame.
 * In this position the number of error for last 10 frames in FIFO will
 * be minimal.
 *
 * After all 720 samples of the next frame will be processed we search the
 * best position (best lag) in the range 0-719 samples and consider this is
 * the start of last symbol of the frame.
 *
 * After new frame will be processed, at modulus 24 of this position we
 * provide fine detecting of symbols and store soft bits in array.
 *
 * Exactly in best lag position (the end of frame) we  extract all soft bits
 * from array normalize for output, provide hard decision and soft FEC.
 *
 * So our modem is stream modem with fast self synchronizing (less then 10
 * frames needs for correct output) with minimal overhead 4/5 used both for
 * synchronizing and soft FEC.
 *
 * Pulse carrier contains 1 pulse to each 24 samples. This is suitable for
 * algebraic code book of AMR codec in 4.75kbps mode: for one AMR codec
 * subframe of 40 samples codebook can code 2 pulses each in 8 possible
 * positions with sign.
 *
 * So our carrier is robust for AMR compression and can be used via
 * cellular phones.
 */
public final class PulseModem {

    private static final int SAMPLES_IN_FRAME = 720;    // 8Khz PCM samples in 90 mS frame
    private static final int SAMPLES_IN_SYMBOL = 24;    // samples in 3-bits symbol
    private static final int SUBFRAMES_IN_FRAME = 18;   // subframes in frame
    private static final int BITS_IN_FRAME = 90;        // raw bits in frame
    private static final int SYMBOLS_IN_FRAME = 30;     // symbols in the frame
    private static final int POS_IN_SYMBOL = 4;         // possible pulse positions in symbol
    private static final int BITS_IN_SYMBOL = 3;        // bits coded by symbol (by pulse position and polarity)
    private static final int DATA_IN_SUBFRAME = 4;      // data bits in subframe  (and one bit is parity)
    private static final int MAXTUNE = 8;               // coefficient for sample rate tuning
    private static final int MRATE = 0;                 // fixed shift of timer (constant difference of RX/TX samplin rates)
    private static final int BERBITS = 50;              // number of error for twice decresing values
    private static final int LOCK_TRS = 100;            // quality level for set lock flag (1-64)
    private static final int UNLOCK_TRS = 100;          // quality level for clear lock flag(0-63)
    private static final int LAGSEQ = 4;                // number of mautched lags for apply to modem (0-15)
    private static final int FIXBER = 2;                // maximal number of errors in received frame for immediately set syn flag
    private static final int CANDIDATE_BER = 4;         // maximal number of errors of lag position for immediately set new lag
    private static final int CANDIDATE_DIST = 6;        // * 10 sec correctable sync lost (1' max)
    private static final int LLR_MAX = ((int) (0x7fff - 1));  // maximal LLR
    private static final int LOGEXP_RES = 401;          // resolution of logexp table
    private static final int POLTEST = 10;              // frames for change polarity

    private class State {

        short[] tail;       // overlapped array between frames
        byte[][] parity;    // array for received bits on every of 24 lags
        int[] wfr;          // parity errors in last 10 frames for each lag
        int[] fd;           // demodulated soft bits in frame
        byte avad;          // java defaults to 0

        public State() {
            tail = new short[2 * SAMPLES_IN_SYMBOL];
            parity = new byte[SAMPLES_IN_SYMBOL][SUBFRAMES_IN_FRAME];
            wfr = new int[SAMPLES_IN_FRAME];
            fd = new int[BITS_IN_FRAME];
        }
    };

    private final State t_md;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // undefine option for detect pulse by rising edge
    // now I don't undestand what is better in such cases
    // that's why this option require additional research yet
    private static final int MIRROR = 1;
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // lag searching values
    private int pbestlag; // (0-719) // best lag points to 696-th sample in the frame (start of last symbol)
    private byte plagflg; // flag of carrier locked
    private final int[] plagseq; // array of errors in 16 last frames
    private byte plagseqptr; // pointer to last frame entry in this array

    // counters for estimate parity errors rate will be near 5% for noise
    private int pbit_cnt; // counter of receiver raw bits (must be non-zero for division)
    private int pbit_err; // counter of parity bits errors (max is 8, noise is 4 errors to 80 raw bits)

    // Channel polarity autodetect
    private byte plt;     // channel polarity (0/1)
    private int pltcnt;  // counter of symbols with wrong polarity

    // Sample rate fine tuning
    private int ar; // amplitudes of left samples of peaks
    private int al; // amplitudes of right samples of peak

    // order of bits reversed in symbol for modulator

    private final byte[] revtab = {
        (byte) 0b000, (byte) 0b100, (byte) 0b010, (byte) 0b110,
        (byte) 0b001, (byte) 0b101, (byte) 0b011, (byte) 0b111
    };

    // order of bits in symbol for fast hard demodulator
    private final byte[] fasttab = {
        (byte) 0b011, (byte) 0b000, (byte) 0b001, (byte) 0b010,
        (byte) 0b100, (byte) 0b111, (byte) 0b110, (byte) 0b101
    };

    // number of set bits in nibble (for check parity)
    private final byte[] SetBitTable = {
        (byte) 0, (byte) 1, (byte) 1, (byte) 2,
        (byte) 1, (byte) 2, (byte) 2, (byte) 3,
        (byte) 1, (byte) 2, (byte) 2, (byte) 3,
        (byte) 2, (byte) 3, (byte) 3, (byte) 4
    };

    // bit's mask in byte
    private final byte[] mask = {
        (byte) 0b0000_0001, (byte) 0b0000_0010, (byte) 0b0000_0100, (byte) 0b0000_1000,
        (byte) 0b0001_0000, (byte) 0b0010_0000, (byte) 0b0100_0000, (byte) 0b1000_0000
    };

    // lag quality depends number of bits errors (for demodulator)
    private final byte[] ltab = {
        (byte) 64, (byte) 32, (byte) 16, (byte) 8, (byte) 4, (byte) 2, (byte) 1, (byte) 0
    };

    // modulation symbol waveform
    private final short[] ulPulse
            = {
                0, 0, 0, 0, 40, -200,
                560, -991, -1400, 7636, 15000, 7636,
                -1400, -991, 560, -200, 40, 0,
                0, 0, 0, 0, 0, 0
            };

    // index of bits in symbol for fine soft demodulator
    private final short[] indexBits
            = {
                0, 0, 0,
                0, 0, 1,
                0, 1, 0,
                0, 1, 1,
                1, 0, 0,
                1, 0, 1,
                1, 1, 0,
                1, 1, 1
            };

    // Jacobian table (Q8)
    private final short[] logExpTable
            = {
                177, 175, 173, 172, 170, 168, 166, 164, 162, 160,
                158, 156, 155, 153, 151, 149, 147, 146, 144, 142,
                141, 139, 137, 136, 134, 132, 131, 129, 128, 126,
                124, 123, 121, 120, 118, 117, 115, 114, 113, 111,
                110, 108, 107, 106, 104, 103, 102, 100, 99, 98,
                96, 95, 94, 93, 92, 90, 89, 88, 87, 86,
                85, 83, 82, 81, 80, 79, 78, 77, 76, 75,
                74, 73, 72, 71, 70, 69, 68, 67, 66, 65,
                64, 64, 63, 62, 61, 60, 59, 59, 58, 57,
                56, 55, 55, 54, 53, 52, 52, 51, 50, 49,
                49, 48, 47, 47, 46, 45, 45, 44, 43, 43,
                42, 42, 41, 40, 40, 39, 39, 38, 38, 37,
                37, 36, 35, 35, 34, 34, 33, 33, 32, 32,
                32, 31, 31, 30, 30, 29, 29, 28, 28, 28,
                27, 27, 26, 26, 26, 25, 25, 25, 24, 24,
                23, 23, 23, 22, 22, 22, 21, 21, 21, 21,
                20, 20, 20, 19, 19, 19, 18, 18, 18, 18,
                17, 17, 17, 17, 16, 16, 16, 16, 15, 15,
                15, 15, 14, 14, 14, 14, 14, 13, 13, 13,
                13, 13, 12, 12, 12, 12, 12, 12, 11, 11,
                11, 11, 11, 11, 10, 10, 10, 10, 10, 10,
                9, 9, 9, 9, 9, 9, 9, 8, 8, 8,
                8, 8, 8, 8, 8, 7, 7, 7, 7, 7,
                7, 7, 7, 7, 7, 6, 6, 6, 6, 6,
                6, 6, 6, 6, 6, 6, 5, 5, 5, 5,
                5, 5, 5, 5, 5, 5, 5, 5, 5, 4,
                4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                4, 4, 4, 4, 4, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
                3, 3, 3, 3, 3, 3, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                0
            };

    public PulseModem() {
        t_md = new State();

        plagseq = new int[16];
        pbit_cnt = 1;
        plt = 1;
    }

    private int SIGN(int val) {
        if (val < 0) {
            return -1;
        } else {
            return 1;
        }
    }

    // MAX function for sum of LLRs in log domain
    private int JacLog(int a, int b) {
        int ind; // index in Jacobian table depend difference of values
        int maxab; // correction value from table

        // choose maximal value and compute absolute difference between values
        if (a > b) {
            maxab = a;         // set maximal value to output
            ind = (a - b) >>> 2; // normalize for table format
        } else {
            maxab = b;          // set maximal value to output
            ind = (b - a) >>> 2;  // normalize for table format
        }

        // correct output with value extracted from table by index
        maxab += (ind >= LOGEXP_RES) ? 0 : logExpTable[ind];

        if (maxab >= LLR_MAX) {
            return LLR_MAX;  // saturate
        } else {
            return maxab; // return resulting LLR
        }
    }

    /*
     * Original C code used pointers, so lack of pointers
     * in Java forces us to use indexes from the caller
     *
     * Symbol processing:
     * Modulate symbol 3-bits (0-7) to 24 PCM S+16 samples
     */
    private void SymbolMod(byte[] symbol, int symbolIndex, short[] mPulse, int mPulseIndex) {
        int i, shifter;

        // initialize memory
        for (i = 0; i < SAMPLES_IN_SYMBOL; i++) {
            mPulse[mPulseIndex + i] = (short) 0;
        }

        // check MSB for set negative polarity
        if (symbol[symbolIndex] < 4) {  // positive symbols
            shifter = 6 * symbolIndex; // set pointer to wave from the start of table

            for (i = 0; i < SAMPLES_IN_SYMBOL; i++) {
                mPulse[mPulseIndex + ((i + shifter) % SAMPLES_IN_SYMBOL)] = ulPulse[i];
            }
        } else { // creates negative symbols
            shifter = 6 * (7 - symbolIndex); // set pointer to wave from the end of table

            for (i = 0; i < SAMPLES_IN_SYMBOL; i++) {
                mPulse[mPulseIndex + ((i + shifter) % SAMPLES_IN_SYMBOL)] = (short)(-1 * ulPulse[i]);
            }
        }
    }

    // convert 4 correlation coefficients to 3 soft bits
    // output demodulated symbol contain 3 hard bits
    private byte SymbolLLR(byte sym, int[] rr, int[] softBits, int index) {
        int i, j, k, m; // counters
        byte c;  // bits
        int Lacc; // LLR, correlation coeeficient
        int[] lp = new int[BITS_IN_SYMBOL]; // array for positive LLR
        byte[] bp = new byte[BITS_IN_SYMBOL];  // flags data already exist
        int[] lm = new int[3]; // array for negative LLR
        byte[] bm = new byte[BITS_IN_SYMBOL]; // flags data already exist

        // for each pulse position
        for (i = 0; i < POS_IN_SYMBOL; i++) {
            int r = rr[(i + 1) & 3]; // get correlation coefficient depends modulator's waves
            // consider positive and negative copy

            for (j = 0; j <= 1; j++) { // 0-positive, 1-negative
                if (j != 0) {
                    m = BITS_IN_SYMBOL * (7 - i);
                    r = -r; // invert correlation coefficient for negative copy
                } else {
                    m = BITS_IN_SYMBOL * i; // m is position in bit table
                }
                
                // compute LACC from r with saturation
                Lacc = r;

                if (Math.abs(Lacc) > LLR_MAX) {
                    Lacc = SIGN(Lacc) * LLR_MAX;
                }

                // cumulative sum of logexp values for 3 bits in symbol
                for (k = 0; k < BITS_IN_SYMBOL; k++) { // for each bit
                    if (indexBits[m + k] != 0) { // check bit flag in index table
                        // if flag is set use LLL+ array
                        if (bp[k] == 0) { // check this cell is empty
                            lp[k] = Lacc; // store LLR in cell
                            bp[k] = 1;    // clear empty flag
                        } else {
                            lp[k] = JacLog(lp[k], Lacc); // if cell is not empty add new LLR to existed
                        }
                    } else { // if bit flag is clear use LLR- array
                        if (bm[k] == 0) { // check this cell is empty
                            lm[k] = Lacc; // store LLR in cell
                            bm[k] = 1; // clear empty flag
                        } else {
                            lm[k] = JacLog(lm[k], Lacc); // if cell is not empty add new LLR to existed
                        }
                    } // end of no LLR in cell yet
                } // end of bit loop for 3 symbol's bits
            }  // end of loop for positive/negative copy
        } // end of loop of processing 4 correlation coefficients

        if (sym == 0xFF) { // provide hard decision of LLR for output hard bits
            // determine final LLR values using specified hard decision
            c = 0;  // symbol's hard bits
            for (k = 0; k < 3; k++) { // for each bit
                Lacc = lm[k] - lp[k]; // compute resulting LLR as difference between positive and negative copy

                if (Math.abs(Lacc) > LLR_MAX) {
                    softBits[index + (2 - k)] = (SIGN(Lacc) * LLR_MAX); // saturate
                } else if (Lacc == 0) {
                    softBits[index + (2 - k)] = 1; // must be non-zero for obtain sign
                } else {
                    softBits[index + (2 - k)] = Lacc; // output LLR value with sign (soft bit)
                }

                c <<= 1; // shift bits

                if (softBits[index + (2 - k)] < 0) {
                    c |= 1; // add hard decision to symbol
                }
            }
        } else {// use external hard decision to set sign of soft bits
            // compute hard decision and determine final LLR values
            c = sym;  // external hard bits of symbol

            for (k = 0; k < 3; k++) { // process for 3 bits
                Lacc = Math.abs(lm[2 - k] - lp[2 - k]); // compute resulting LLR as absolute difference between positive and negative copy

                if (Lacc == 0) {
                    Lacc++; // must be non-zero for obtain sign
                }

                if (Lacc > LLR_MAX) {
                    Lacc = LLR_MAX; // saturate
                }

                if ((sym & 1) != 0) {
                    Lacc = -Lacc; // set sign of soft bit depends external hard bit
                }

                sym >>>= 1;  // shift external hard bits to next
                softBits[index + k] = Lacc;  // set soft bit
            }
        }

        return c; // returns 3 hard bits of symbol (first bit is bit 0)
    }

    // fast demodulate 24 samples (symbol):
    // compute 4 correlation coefficients for possible pulse positions and polarity
    // returns 3 hard bits of demodulated symbol
    private byte SymbolDemod(short[] sym, int sym_ptr, int[] r) {
        int d, k; // best and current amplitude of pulse
        int i, b; // counter and best pulse position

        if (plt != 0) {
            // compute correlation coefficients for each possible pulse position
            r[0] = (int) sym[sym_ptr]
                    + (((int) sym[sym_ptr + 1]) << 1) + (int) sym[sym_ptr + 2] - (int) sym[sym_ptr + 3] - (((int) sym[sym_ptr + 4]) << 1) - (int) sym[sym_ptr + 5];
            r[1] = (int) sym[sym_ptr + 6]
                    + (((int) sym[sym_ptr + 1 + 6]) << 1) + (int) sym[sym_ptr + 2 + 6] - (int) sym[sym_ptr + 3 + 6] - (((int) sym[sym_ptr + 4 + 6]) << 1) - (int) sym[sym_ptr + 5 + 6];
            r[2] = (int) sym[sym_ptr + 12]
                    + (((int) sym[sym_ptr + 1 + 12]) << 1) + (int) sym[sym_ptr + 2 + 12] - (int) sym[sym_ptr + 3 + 12] - (((int) sym[sym_ptr + 4 + 12]) << 1) - (int) sym[sym_ptr + 5 + 12];
            r[3] = (int) sym[sym_ptr + 18]
                    + (((int) sym[sym_ptr + 1 + 18]) << 1) + (int) sym[sym_ptr + 2 + 18] - (int) sym[sym_ptr + 3 + 18] - (((int) sym[sym_ptr + 4 + 18]) << 1) - (int) sym[sym_ptr + 5 + 18];
        } else {
            // compute correlation coefficients for each possible pulse position
            r[0] = (int) sym[sym_ptr + 3] + (((int) sym[sym_ptr + 4]) << 1) + (int) sym[sym_ptr + 5] - (int) sym[sym_ptr + 0] - (((int) sym[sym_ptr + 1]) << 1) - (int) sym[sym_ptr + 2];
            r[1] = (int) sym[sym_ptr + 3 + 6] + (((int) sym[sym_ptr + 4 + 6]) << 1) + (int) sym[sym_ptr + 5 + 6] - (int) sym[sym_ptr + 0 + 6] - (((int) sym[sym_ptr + 1 + 6]) << 1) - (int) sym[sym_ptr + 2 + 6];
            r[2] = (int) sym[sym_ptr + 3 + 12] + (((int) sym[sym_ptr + 4 + 12]) << 1) + (int) sym[sym_ptr + 5 + 12] - (int) sym[sym_ptr + 0 + 12] - (((int) sym[sym_ptr + 1 + 12]) << 1) - (int) sym[sym_ptr + 2 + 12];
            r[3] = (int) sym[sym_ptr + 3 + 18] + (((int) sym[sym_ptr + 4 + 18]) << 1) + (int) sym[sym_ptr + 5 + 18] - (int) sym[sym_ptr + 0 + 18] - (((int) sym[sym_ptr + 1 + 18]) << 1) - (int) sym[sym_ptr + 2 + 18];
        }

        // search position with best amplitude
        d = 0;
        b = 0;

        for (i = 0; i < 4; i++) { // check for each position
            k = Math.abs(r[i]); // amplitude of pulse in this position

            if (k > d) { // if this is a best amplitude
                d = k; // set this amplitude as best
                b = i; // set this position as best
            }
        }

        if (r[b] < 0) {
            b += 4;  // check polarity of best pulse and set third bit of symbol
        }

        return fasttab[b]; // convert to actual symbol bits
    }

    /*
     * Modulate 72 data bits to 720 PCM samples (30 symbols per frame)
     * add 18 parity bits at the end of frame (90 bits total)
     *
     * pulse modulator: 3 bit codes for one pulse in 4 possible positions
     * positive or negative. (90 / 3 = 30 3-bit symbols)
     *
     * 24 PCM S+15 bit output samples per 3-bit symbol
     */
    public void modulate(byte[] data, short[] frame) {
        int i, j, k;
        byte[] txp = new byte[SUBFRAMES_IN_FRAME]; // 18
        byte b = 0; // bits of output symbol

        // process all bits will be modulated
        for (k = 0; k < BITS_IN_FRAME; k++) { // 90 = number of output bits in frame
            j = k % SUBFRAMES_IN_FRAME; // source subframe for current output
            i = k / SUBFRAMES_IN_FRAME; // source data bit for current output in subframe
            b <<= 1; // shift symbol's bits

            if (i != DATA_IN_SUBFRAME) { // 4, check this is data bit or parity bit
                int bn = j * DATA_IN_SUBFRAME + i; // number of requested input data bit

                if ((data[bn / 8] & mask[bn % 8]) != 0) {
                    b |= 1; // get data bit by number from input
                }

                txp[j] ^= (b & 1); // add this bit to parity of this subframe
            } else {
                b |= txp[(SUBFRAMES_IN_FRAME - 1) - j];  // this is parity bit for subframe (transmit reverse)
            }
            
            // modulate every 3 bits to 24 pcm samples (one 3-bit symbol)
            if ((k % 3) == 2) {
                SymbolMod(revtab, b & 7, frame, 8 * (k - 2));
            }
        }
    }

    /*
     * Demodulate 720 pcm samples to 9 bytes
     * Corresponding LLR output array must be 16 bytes (for flags and statistics)
     * returns signed value +/- 8 to fine tune samplerate by hardware
     */
    public int demodulate(short[] frame, byte[] data, byte[] fout) {
        int i, j, kk, dd; // general counters
        int sym_ptr; // pointer to processed symbol
        short[] sym;
        int[] rr = new int[POS_IN_SYMBOL]; // correlation coefficients for every pulse position
        byte a, b, c; // bits
        byte[] tsym = new byte[SUBFRAMES_IN_FRAME]; // table for inverse parity bit location
        int d, q; // 24-pcm lag and number of virtual subframe
        int bnum; // number of current bit
        int z; // average LLR in this frame (frame quality)
        byte ber; // errors counter in outputted frame
        int u; // FIFO of number of errors in last 10 frames

        // Compute frame energy on 1333Hz for normalize correlation coefficients
        int e = 8; // initial value must be non-zero for always allow to be a divider
        
        for (i = 0; i < SAMPLES_IN_FRAME - 3; i++) { // process all samples in the frame
            e += (Math.abs((int) frame[i] - (int) frame[i + 3])); // sum wave amplitudes
        }

        e >>>= 3; // divide for match Q

        // Copy first symbol to second half of tail
        System.arraycopy(frame, 0, t_md.tail, SAMPLES_IN_SYMBOL, SAMPLES_IN_SYMBOL);

        // process 720 pcm positions in loop
        for (int scnt = 0; scnt < SAMPLES_IN_FRAME; scnt++) {
            // set pointer to symbol (24 pcm) will be demodulated
            if (scnt < SAMPLES_IN_SYMBOL) {
                sym = t_md.tail;
                sym_ptr = scnt;
            } else {
                sym = frame;
                sym_ptr = scnt - SAMPLES_IN_SYMBOL; // first 24 pcm is in tail
            }

            // fast demodulate symbol to 3 bits
            a = SymbolDemod(sym, sym_ptr, rr);

            // set values for check parity at lag of this sample
            b = a; // demodulated bits of symbol in current lag
            
            int lag = scnt % SAMPLES_IN_SYMBOL; // lag of current virtual symbol in samples from frame start, 0-23
            int sbf = ((scnt / SAMPLES_IN_SYMBOL) % (SAMPLES_IN_SYMBOL / POS_IN_SYMBOL)) * BITS_IN_SYMBOL; // number of current virtual subframe 0-15

            // add 3 bits to set of virtual subframe for current lag
            t_md.parity[lag][sbf] <<= 1; // shift bits of subframe for this lag
            t_md.parity[lag][sbf++] |= (b & 1);
            b >>>= 1; // add received bit, shift received bits
            t_md.parity[lag][sbf] <<= 1;
            t_md.parity[lag][sbf++] |= (b & 1);
            b >>>= 1;
            t_md.parity[lag][sbf] <<= 1;
            t_md.parity[lag][sbf++] |= (b & 1);
            b >>>= 1;

            // set reverse order of parity bits
            for (i = 0; i < SUBFRAMES_IN_FRAME; i++) {// 18 parity bits: one for each subframe
                tsym[(sbf + i) % SUBFRAMES_IN_FRAME] = (byte) ((sbf + SUBFRAMES_IN_FRAME - 1 - i) % SUBFRAMES_IN_FRAME); // set in reverse order
            }

            // count parity errors for this lag(0-18), add to FIFO
            b = 0; // clear counter
            for (i = 0; i < SUBFRAMES_IN_FRAME; i++) { // check parity of all subframes
                c = SetBitTable[0x0F & (t_md.parity[lag][i] >>> 1)]; // checksum of 4 info bits in the frame
                c ^= (t_md.parity[lag][tsym[i]] & 1);  // check control bit, result 0 is OK, 1 is parity error
                b += (c & 1); // add to errors counter
            }

            if (b > 7) {
                b = 7;  // saturate to 7 errors maximum
            }

            t_md.wfr[scnt] <<= 3; // shift FIFO
            t_md.wfr[scnt] |= b;  // add lag errors to FIFO (contains last 10 frames)

            // search DC level of current symbol
            int m = 0; // DC level
            
            for (i = 0; i < SAMPLES_IN_SYMBOL; i++) {
                m += sym[sym_ptr + i]; // averages samples in the symbol
            }

            m /= SAMPLES_IN_SYMBOL; // set DC level

            // search position of maximal pulse relation DC in the symbol
            int r = 0; // max value
            int k = 0; // position of max pulse (0-23)

            for (i = 0; i < SAMPLES_IN_SYMBOL; i++) { // check all samples
                int l = Math.abs(m - sym[sym_ptr + i]); // absolute amplitude of sample over DC

                if (l > r) { // if best
                    r = l; // save best amplitude
                    k = i; // save position
                }
            }

            kk = k / 3;
            kk *= 3;
            dd = scnt % 3;

            switch (dd) {
                case 0:
                    al += Math.abs((int) sym[sym_ptr + kk + 1] - (int) sym[sym_ptr + kk]); // collect left shift
                    ar += Math.abs((int) sym[sym_ptr + kk + 1] - (int) sym[sym_ptr + kk + 2]); // collect right shift
                    break;
                case 1:
                    al += Math.abs((int) sym[sym_ptr + kk] - (int) sym[sym_ptr + kk + 2]); // collect left shift
                    ar += Math.abs((int) sym[sym_ptr + kk] - (int) sym[sym_ptr + kk + 1]); // collect right shift
                    break;
                default:
                    al += Math.abs((int) sym[sym_ptr + kk + 2] - (int) sym[sym_ptr + kk + 1]); // collect left shift
                    ar += Math.abs((int) sym[sym_ptr + kk + 2] - (int) sym[sym_ptr + kk]); // collect right shift
            }

            // check is the current lag is exactly the 24 pcm symbol start
            if (lag != (pbestlag % SAMPLES_IN_SYMBOL)) {
                continue;
            }

            // obtain channel polarity
            k /= 3; // position of wave (3 pulses), must be even (or odd???)

            if (MIRROR == 1) {
                if ((k & 1) == 0) {
                    pltcnt++;  // increment counter for wrong position
                } else if (pltcnt != 0) {
                    pltcnt--; // decrement to 0 for good position
                }
            } else {
                if ((k & 1) != 0) {
                    pltcnt++;  // increment counter for wrong position
                } else if (pltcnt != 0) {
                    pltcnt--; // decrement to 0 for good position
                }
            }

            // set correct channel polarity
            if (pltcnt > (SYMBOLS_IN_FRAME * POLTEST)) { // check level of wrong positions
                plt ^= 1; // change polarity
                pltcnt = 0; // reset counter
            }

            // compute actual bit number in the real frame using value of best lag position
            bnum = (scnt - pbestlag) / SAMPLES_IN_SYMBOL - 1;  // actual triplet number minus one

            if (bnum < 0) {
                bnum += SYMBOLS_IN_FRAME;   // ring    29
            }

            bnum *= BITS_IN_SYMBOL; // actual bit number

            // compute LLR for symbol's bits
            for (i = 0; i < POS_IN_SYMBOL; i++) {
                rr[i] = rr[i] * 15000 / e; // normalize correlation coefficients
            }

            SymbolLLR((byte) 0xFF, rr, t_md.fd, bnum); // compute soft bits from hard bits and normalized correlation coefficients

            // check the current lag is exactly the new frame start
            if (scnt != pbestlag) {
                continue;
            }

            // now all 90 bits received and set on his places in fb ready to output
            // find average LLR in this frame
            int m_s = 0;
            
            for (i = 0; i < BITS_IN_FRAME; i++) {
                m_s += Math.abs(t_md.fd[i]); // sum LLR of all bits in frame
            }

            m_s /= BITS_IN_FRAME; // average LLR for this frame

            if (m_s == 0) {
                m_s = 1; // must be non-zero for division
            }

            // clear  data for frame output processing
            for (i = 0; i < 16; i++) {
                data[i] = (byte) 0; // clear bytes output array
            }

            bnum = 0; // init output bits counter
            ber = 0;  // clear parity errors counter
            q = 0;

            // process all subframes in the frame
            for (i = 0; i < SUBFRAMES_IN_FRAME; i++) { // 18  process next subframe
                c = (byte) 0;   // clear subframe parity
                d = 0x7FFF;     // set maximal possible metric value

                // process this subframe
                for (j = 0; j < DATA_IN_SUBFRAME; j++) { // 4  process all bits in subframe
                    k = t_md.fd[j * SUBFRAMES_IN_FRAME + i];  // 18  set soft output bit value from input array

                    z = Math.abs((int) k * 128 / m_s);  // LLR normalized to unsigned char
                    
                    if (z > 255) {
                        z = 255; // saturate
                    }

                    fout[bnum] = (byte) z; // output normalized LLR

                    if (k < 0) { // hard bit decision: bit=1
                        data[bnum / 8] ^= mask[bnum % 8]; // set hard output bit
                        c ^= 1; // add bit to parity
                    }

                    if (Math.abs(k) < d) { // check level of metric of this bit: if lowest in this symbol
                        q = bnum;          // remember number of bit with lowest metric
                        d = Math.abs(k);   // set lowest metric level
                    }

                    bnum++; // next output bit
                } // end of processing all bits in subframe

                // FEC
                if (t_md.fd[BITS_IN_FRAME - 1 - i] < 0) {
                    c ^= 1; // check parity of processed subframe
                }

                if ((c != 0) && (d < Math.abs(t_md.fd[BITS_IN_FRAME - 1 - i]))) { // if parity wrong and worse bit is not a parity bit
                    data[q / 8] ^= mask[q % 8]; // flip corresponds hard output bit
                    ber++;  // count bit error
                }
            } // end of processing all subframes in the frame

            // compute parity error rate in percent
            if (pbit_err > BERBITS) {// check is total number of errors so large
                pbit_err >>>= 1; // twice decrease total number of errors
                pbit_cnt >>>= 1; // twice decrease total number of received bits

                if (pbit_cnt == 0) {
                    pbit_cnt++;	// number of received bits must be non-zero (for division)
                }
            }

            pbit_cnt += BITS_IN_FRAME; // add number of received bits
            pbit_err += ber; // add number of error

            z = (1000000 / BITS_IN_FRAME) * pbit_err / pbit_cnt; // compute bit errors rate in percent*100

            if (z > 4095) {
                z = 4095; // saturate to 12 bits
            }

            // search best lag value
            d = 0; // set minimal possible level for search maximal value
            for (i = 0; i < SAMPLES_IN_FRAME; i++) { // look all lags (for every sample position in frame)
                u = t_md.wfr[i];  // get parity errors FIFO value for this lag
                k = 0; // weight of lag

                for (j = 0; j < 10; j++) { // count for last 10 frames in fifo
                    k += ltab[u & 7]; // 3 bits (0-7 errors) convert to log weight
                    u >>>= 3; // set FIFO to next frame
                }

                if (k > d) {// check weight of this lag is the best
                    d = k; // set best weight
                    q = i; // set best lag
                }
            }

            // set carrier locked flag
            if (d > LOCK_TRS) { // check best quality is sufficient
                if (plagflg == 0) {
                    pbestlag = q; // if carrier was unlocked immediately set new modem lags
                }

                plagflg = 1; // set lag flag
            } else if (d < UNLOCK_TRS) {
                plagflg = 0; // if carrier poor clear lock flag
            }

            // new fast fix procedure
            // check finded lag is stable and set general lag
            plagseq[plagseqptr++] = q; // add finded lag to candidates array, move pinter
            plagseqptr &= 0x0F; // ring pointer for 16 entries in array

            b = 0; // clear match counter
            for (i = 0; i < 16; i++) {
                if (q == plagseq[i]) {
                    b++; // count matches values with currently finded lag
                }
            }

            if (b >= LAGSEQ) {
                pbestlag = q; // if some values were matched set currently finded lag is a best lag
            }

            // fast search of lag in sync lost
            if (plagflg == 0) { // if there is no sync currently, try sync fast
                if (ber < FIXBER) {
                    plagflg = 1; // good BER: current sync OK, set flag immediately
                } else { // poor BER: search good lag of current frame near current lag position
                    k = pbestlag; // best lag for last received frame (set as a current lag)
                    d = SAMPLES_IN_FRAME / 2; // shortest distance to current lag (set maximum)
                    c = 0x07; // number of errors for best lag (set maximum)

                    for (i = 0; i < SAMPLES_IN_FRAME; i++) { // look all lags (for every sample position in frame)
                        b = (byte) (t_md.wfr[i] & 0x07); // number of errors for this lag in the last received frame

                        if (b <= c) { // if number of errors is best or equal for this frame
                            j = Math.abs(i - pbestlag); // distance from finded lag to current lag

                            if (j > (SAMPLES_IN_FRAME / 2 - 1)) {
                                j = SAMPLES_IN_FRAME - j; // distance can be in both sides
                            }

                            if (j < d) { // finded lag is set closely to current lag
                                k = i; // save finded lag candidate
                                d = j; // save distance of lag candidate
                                c = b;	// save number of error of finded lag candidate
                            }	 // end of finded lag distance is the best
                        }  // end of finded error are best or equal
                    }  // end of search loop in whole frame

                    if ((c <= CANDIDATE_BER) && (d <= CANDIDATE_DIST)) {
                        pbestlag = k; // check allowed number of errors and distance and correct current lag
                    }
                } // end of pure BER
            } // end of no-sync

            // output frame statistic
            data[11] = (byte) (ber << 1);    // raw BER counter (5 bit, 0-18)
            data[12] = (byte) (z >>> 4);     // ber 8 MSB
            data[13] = (byte) (z & 0x0F);    // ber 4 LSB

            // current modem lag
            data[13] |= (byte) ((pbestlag >>> 4) & 0x30);  // 4 MSB of lag
            data[14] = (byte) (pbestlag & 0xFF);          // 8 LSB of lag (0-719)

            // flag of carrier locked
            if (plagflg != 0) {
                data[11] |= (byte) 0x40; // block lag lock flag
            }
        } // end of processed all samples

        // compute value for fine samplerate tuning
        int mr = (ar + al);

        if (mr == 0) {
            mr = 0x7FFFFFFF; // compute divider for normalization (nonzero)
        }

        int peak = (MAXTUNE * (ar - al) / mr) + MRATE; // normalized difference between left and right samples of pulse

        ar = 1;
        al = 1; // clear sum for received frame

        data[15] = (byte) peak; // value for fine tuning recorded sample rate

        // overlap frames to one symbol
        System.arraycopy(frame, (SAMPLES_IN_FRAME - SAMPLES_IN_SYMBOL), t_md.tail, 0, SAMPLES_IN_SYMBOL);

        return peak;// returns sync value
    }
}

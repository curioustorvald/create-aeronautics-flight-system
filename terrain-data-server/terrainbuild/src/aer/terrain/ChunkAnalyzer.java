package aer.terrain;

/**
 * Turns one chunk of NBT into the four bytes the flight system reads.
 *
 * RESOLUTION. One record per chunk (16 x 16 blocks) and one Y level per 8 blocks, so the
 * world's -64..320 becomes 48 levels: level L covers Y in [-64+8L, -64+8L+7]. Heights are
 * reported at the TOP of their level, which rounds every answer UP by up to 7 m. That is
 * the safe direction for a GPWS -- it may call ground slightly higher than it is, never
 * lower.
 *
 * TERRAIN vs OBSTACLE. A cell is "occupied" if it holds even one non-air block. Occupied
 * cells form vertical runs; the run holding the ANCHOR -- the level of the chunk's lowest
 * surface column, taken from the MOTION_BLOCKING heightmap -- is the ground. Every
 * occupied run above it, separated by at least one wholly empty cell (>= 8 m of clear
 * air), is a floating mass and is charted as an OBSTACLE.
 *
 * Anchoring on the heightmap rather than on "the run that reaches bedrock" is what keeps
 * caves out of this. A big cavern can empty a cell underground and split the ground run in
 * two; walking up from bedrock would then chart the real surface as a floating island and
 * the GPWS would cry wolf over ordinary countryside. The heightmap anchor is by
 * construction at the surface, above any cave, so the run it names is the ground.
 *
 * The one case the anchor gets wrong is an island that covers the whole chunk footprint:
 * then the lowest surface column IS the island. That is caught afterwards by looking for a
 * large void beneath the anchor run (VOID_GAP cells, >= 24 m, with not one solid block in
 * the chunk's whole 16x16 footprint), which ordinary terrain does not produce. Note the
 * failure is biased the safe way even when it is missed: an island read as terrain still
 * gets its top charted, so clearance stays right and only the wording of the callout is
 * wrong.
 */
final class ChunkAnalyzer {
    static final int MIN_Y = -64, CELL = 8, LEVELS = 48;

    // Flag bits in record byte 3. Mirrored in terrainconf.lua (CFG.FLAG) and in the manual.
    static final int F_MASS   = 0x03;   // 0 none, 1 thin, 2 medium, 3 large
    static final int F_WATER  = 0x04;
    static final int F_MULTI  = 0x08;   // more than one floating layer
    static final int F_TREES  = 0x10;   // terrain top is canopy, not ground
    static final int F_SPIKE  = 0x20;   // terrain top is a thin mast/tower, not a surface
    static final int F_VOID   = 0x40;   // nothing solid anywhere in the column
    static final int F_NODATA = 0x80;   // chunk absent, unfinished or unreadable

    static final int NONE = 0xFF;

    /**
     * Cells of clear air under the anchor run that mean the anchor is itself floating.
     *
     * ONE cell, measured rather than guessed. The islands in this world sit low: chunk
     * -392,20520 is ground to Y 63, one wholly empty cell at Y 64..71, then solid mass from
     * Y 72 to Y 271. At a gap of 2 that island -- and 60 chunks like it in a 59k-chunk
     * sample -- was charted as "terrain at Y 271", which is wrong twice over: it loses the
     * OBSTACLE callout, and it puts the ground 200 m above an aeroplane flying UNDER the
     * island, which would read a negative radio altitude.
     *
     * Loosening this is safe here only because VOID_MIN_LEVEL below is what actually keeps
     * caves out. Clearance is unaffected by the terrain/obstacle split either way -- the
     * corridor reduction takes the max of both -- so the split is about the WORDING of the
     * callout and about groundAt(), not about whether the aeroplane is warned.
     *
     * Overridable as -Daer.voidgap=N for re-tuning against a world that generates
     * differently, without a recompile.
     */
    private static final int VOID_GAP = Integer.getInteger("aer.voidgap", 1);
    /**
     * ...and the void has to be at or above SEA LEVEL to count. This is the whole guard
     * against caves. Y=64 is level 16.
     *
     * Without it the rule is actively dangerous, and not theoretically: chunk -2120,19016
     * in this world has levels 2-4 (Y -48..-17) completely hollow -- a deep cavern under
     * ordinary countryside. An unbounded walk-down put "terrain" at Y -49 and charted the
     * entire real landscape from Y -24 to 159 as one floating obstacle. A GPWS fed that
     * shouts OBSTACLE AHEAD at a grass field, and a GPWS that shouts at grass fields gets
     * switched off. Islands float in the sky; caves do not.
     */
    private static final int VOID_MIN_LEVEL = (64 - MIN_Y) / CELL;
    /** Solid blocks (of 2048 in a cell) under which the terrain top is called a spike. */
    private static final int SPIKE_MAX = 8;
    /** ...and the fill, 2 levels lower, that a spike has to be standing on. */
    private static final int SPIKE_BASE = 512;

    // ---- outputs -----------------------------------------------------------------
    int terr, otop, obas, flag;

    // ---- scratch, reused for every chunk in the region ---------------------------
    private final int[] count = new int[LEVELS];
    private boolean[] solid = new boolean[512];
    private final int[] hmMB = new int[256], hmNL = new int[256], hmOF = new int[256];

    private int secDataOff, secDataLongs, secBits, secPalette, secY;
    private boolean secHasStates;

    private int hmMBoff = -1, hmNLoff = -1, hmOFoff = -1;

    /** Per-level solid-block tally of the chunk just analysed. For `probe` only. */
    int[] counts() { return count; }

    void nodata() { terr = NONE; otop = NONE; obas = NONE; flag = F_NODATA; }

    /**
     * @return false when the chunk is not a finished, readable one -- the caller should
     *         then write the NODATA record.
     */
    boolean analyze(Nbt n, byte[] buf, int len) {
        java.util.Arrays.fill(count, 0);
        hmMBoff = hmNLoff = hmOFoff = -1;
        boolean full = false, sawSections = false;

        n.reset(buf, 0, len);
        if (n.u8() != Nbt.COMPOUND) return false;
        int rootName = n.u16(); n.p += rootName;         // the root tag's (empty) name

        for (;;) {
            if (n.p >= n.len) return false;
            int t = n.u8();
            if (t == Nbt.END) break;
            int nl = n.u16(), no = n.p;
            n.p += nl;

            if (t == Nbt.STRING && n.bytesAre(no, nl, "Status")) {
                int sl = n.u16(), so = n.p;
                n.p += sl;
                full = n.bytesAre(so, sl, "minecraft:full") || n.bytesAre(so, sl, "full");
            } else if (t == Nbt.LIST && n.bytesAre(no, nl, "sections")) {
                sawSections = true;
                readSections(n);
            } else if (t == Nbt.COMPOUND && n.bytesAre(no, nl, "Heightmaps")) {
                readHeightmaps(n);
            } else {
                n.skip(t);
            }
        }
        if (!full || !sawSections) return false;

        classify(n.b);
        return true;
    }

    // ------------------------------------------------------------------------------
    // NBT walk
    // ------------------------------------------------------------------------------

    private void readSections(Nbt n) {
        int et = n.u8(), cnt = n.i32();
        if (et != Nbt.COMPOUND) { for (int i = 0; i < cnt; i++) n.skip(et); return; }

        for (int i = 0; i < cnt; i++) {
            secY = Integer.MIN_VALUE;
            secHasStates = false;
            secDataOff = -1; secDataLongs = 0; secBits = 0; secPalette = 0;

            for (;;) {
                int t = n.u8();
                if (t == Nbt.END) break;
                int nl = n.u16(), no = n.p;
                n.p += nl;

                if (t == Nbt.BYTE && n.bytesAre(no, nl, "Y")) {
                    secY = n.i8();
                } else if (t == Nbt.COMPOUND && n.bytesAre(no, nl, "block_states")) {
                    readBlockStates(n);
                } else {
                    n.skip(t);        // biomes, BlockLight, SkyLight -- 2 KiB each, never read
                }
            }
            if (secHasStates) accumulate(n.b);
        }
    }

    private void readBlockStates(Nbt n) {
        secHasStates = true;
        for (;;) {
            int t = n.u8();
            if (t == Nbt.END) break;
            int nl = n.u16(), no = n.p;
            n.p += nl;

            if (t == Nbt.LIST && n.bytesAre(no, nl, "palette")) {
                readPalette(n);
            } else if (t == Nbt.LONG_ARRAY && n.bytesAre(no, nl, "data")) {
                secDataLongs = n.i32();
                secDataOff = n.p;
                n.p += 8 * secDataLongs;
            } else {
                n.skip(t);
            }
        }
    }

    private void readPalette(Nbt n) {
        int et = n.u8(), cnt = n.i32();
        secPalette = cnt;
        if (solid.length < cnt) solid = new boolean[Integer.highestOneBit(Math.max(cnt, 2) - 1) << 2];
        if (et != Nbt.COMPOUND) { for (int i = 0; i < cnt; i++) n.skip(et); return; }

        for (int i = 0; i < cnt; i++) {
            boolean air = false;
            for (;;) {
                int t = n.u8();
                if (t == Nbt.END) break;
                int nl = n.u16(), no = n.p;
                n.p += nl;
                if (t == Nbt.STRING && n.bytesAre(no, nl, "Name")) {
                    int sl = n.u16(), so = n.p;
                    n.p += sl;
                    air = isAir(n, so, sl);
                } else {
                    n.skip(t);        // Properties
                }
            }
            solid[i] = !air;
        }
        // Block states are packed with at least 4 bits per entry, whatever the palette size.
        int bits = 4;
        while ((1 << bits) < cnt) bits++;
        secBits = bits;
    }

    /**
     * Air-like blocks -- the ones that must not make a cell "occupied". Barrier is
     * deliberately NOT here: it is invisible but solid, and an aircraft stops against it.
     * The ":air" suffix catches modded airs without this needing to know all 328 mods.
     */
    private static boolean isAir(Nbt n, int off, int len) {
        return n.bytesAre(off, len, "minecraft:air")
            || n.bytesAre(off, len, "minecraft:cave_air")
            || n.bytesAre(off, len, "minecraft:void_air")
            || n.bytesAre(off, len, "minecraft:structure_void")
            || n.bytesAre(off, len, "minecraft:light")
            || n.endsWith(off, len, ":air");
    }

    private void readHeightmaps(Nbt n) {
        for (;;) {
            int t = n.u8();
            if (t == Nbt.END) break;
            int nl = n.u16(), no = n.p;
            n.p += nl;
            if (t == Nbt.LONG_ARRAY) {
                int cnt = n.i32(), off = n.p;
                n.p += 8 * cnt;
                if (cnt >= 37) {
                    if (n.bytesAre(no, nl, "MOTION_BLOCKING")) hmMBoff = off;
                    else if (n.bytesAre(no, nl, "MOTION_BLOCKING_NO_LEAVES")) hmNLoff = off;
                    else if (n.bytesAre(no, nl, "OCEAN_FLOOR")) hmOFoff = off;
                }
            } else {
                n.skip(t);
            }
        }
    }

    // ------------------------------------------------------------------------------
    // Occupancy
    // ------------------------------------------------------------------------------

    private static long lng(byte[] b, int o) {
        return ((long) (b[o] & 0xFF) << 56) | ((long) (b[o + 1] & 0xFF) << 48)
             | ((long) (b[o + 2] & 0xFF) << 40) | ((long) (b[o + 3] & 0xFF) << 32)
             | ((long) (b[o + 4] & 0xFF) << 24) | ((long) (b[o + 5] & 0xFF) << 16)
             | ((long) (b[o + 6] & 0xFF) << 8)  | (long) (b[o + 7] & 0xFF);
    }

    /** Fold one 16-block section into the two 8-block cells it covers. */
    private void accumulate(byte[] b) {
        if (secY == Integer.MIN_VALUE) return;
        int lo = 2 * secY + 8;                      // level index of the section's lower half
        if (lo < 0 || lo + 1 >= LEVELS) return;     // lighting-only sections outside the build range

        if (secPalette == 0) return;
        if (secDataOff < 0) {                       // a uniform section: no data array is stored
            if (secPalette >= 1 && solid[0]) { count[lo] += 2048; count[lo + 1] += 2048; }
            return;
        }
        int bits = secBits, per = 64 / bits;
        if (per <= 0) return;
        int need = (4096 + per - 1) / per;
        if (secDataLongs < need) return;            // truncated -- leave the cells empty

        long mask = (1L << bits) - 1;
        int cLo = 0, cHi = 0, idx = 0;
        for (int L = 0; L < need; L++) {
            long v = lng(b, secDataOff + 8 * L);
            for (int k = 0; k < per; k++) {
                if (idx >= 4096) break;
                if (solid[(int) ((v >>> (k * bits)) & mask)]) { if (idx < 2048) cLo++; else cHi++; }
                idx++;
            }
        }
        count[lo] += cLo;
        count[lo + 1] += cHi;
    }

    private void decodeHeightmap(byte[] b, int off, int[] out) {
        for (int i = 0; i < 256; i++) {
            long v = lng(b, off + 8 * (i / 7));
            out[i] = (int) ((v >>> (9 * (i % 7))) & 511);   // "one above the highest block"
        }
    }

    // ------------------------------------------------------------------------------
    // Classification
    // ------------------------------------------------------------------------------

    private void classify(byte[] b) {
        flag = 0;

        // --- the anchor: the level of the LOWEST surface column in the chunk ---------
        int anchor = -1;
        if (hmMBoff >= 0) {
            decodeHeightmap(b, hmMBoff, hmMB);
            int lowest = Integer.MAX_VALUE;
            for (int i = 0; i < 256; i++) if (hmMB[i] > 0 && hmMB[i] < lowest) lowest = hmMB[i];
            if (lowest != Integer.MAX_VALUE) {
                int y = MIN_Y + lowest - 1;
                anchor = clampLevel((y - MIN_Y) / CELL);
            }
        }

        int top = -1;
        for (int L = LEVELS - 1; L >= 0; L--) if (count[L] > 0) { top = L; break; }
        if (top < 0) { terr = NONE; otop = NONE; obas = NONE; flag = F_VOID; return; }

        if (anchor < 0 || count[anchor] == 0) anchor = top;   // no heightmap: fall back to the top

        // --- the run holding the anchor ---------------------------------------------
        int rTop = anchor; while (rTop + 1 < LEVELS && count[rTop + 1] > 0) rTop++;
        int rBot = anchor; while (rBot - 1 >= 0 && count[rBot - 1] > 0) rBot--;

        // --- is the anchor run itself floating? --------------------------------------
        // Catches an island big enough to cover the chunk's whole footprint, where the
        // lowest surface column IS the island and the anchor lands on it.
        //
        // The bar is deliberately high: VOID_GAP whole cells with not one solid block in
        // 16x16x16+ metres, AND the void sitting at or above sea level. Getting this wrong
        // in the permissive direction invents obstacles over open country; getting it
        // wrong in the strict direction merely charts an island as terrain, which still
        // reports the right HEIGHT and so still keeps the aeroplane off it. Given the
        // choice, chart it as terrain.
        for (;;) {
            int gap = 0, L = rBot - 1;
            while (L >= 0 && count[L] == 0) { gap++; L--; }
            if (L < 0 || gap < VOID_GAP || L + 1 < VOID_MIN_LEVEL) break;
            rTop = L;                                       // the ground is what is under the void
            rBot = L; while (rBot - 1 >= 0 && count[rBot - 1] > 0) rBot--;
        }

        terr = rTop;

        // --- everything above the ground run, in runs, is floating --------------------
        long mass = 0;
        int oT = -1, oB = -1, runs = 0;
        boolean inRun = false;
        for (int L = rTop + 2; L < LEVELS; L++) {           // rTop+1 is empty by construction
            if (count[L] > 0) {
                if (!inRun) { runs++; inRun = true; if (oB < 0) oB = L; }
                oT = L;
                mass += count[L];
            } else {
                inRun = false;
            }
        }
        if (oT >= 0) {
            otop = oT; obas = oB;
            flag |= massClass(mass);
            if (runs > 1) flag |= F_MULTI;
        } else {
            otop = NONE; obas = NONE;
        }

        // --- is the reported ground a thin mast rather than a surface? ----------------
        if (count[rTop] < SPIKE_MAX) {
            for (int L = rTop - 2; L >= rBot && L >= 0; L--) {
                if (count[L] >= SPIKE_BASE) { flag |= F_SPIKE; break; }
            }
        }

        // --- water and canopy, straight off the heightmaps ---------------------------
        if (hmMBoff >= 0 && hmOFoff >= 0) {
            decodeHeightmap(b, hmOFoff, hmOF);
            int wet = 0;
            for (int i = 0; i < 256; i++) if (hmMB[i] > hmOF[i]) wet++;
            if (wet >= 128) flag |= F_WATER;
        }
        if (hmMBoff >= 0 && hmNLoff >= 0) {
            decodeHeightmap(b, hmNLoff, hmNL);
            int leafy = 0;
            for (int i = 0; i < 256; i++) if (hmMB[i] - hmNL[i] >= 3) leafy++;
            if (leafy >= 64) flag |= F_TREES;
        }
    }

    private static int massClass(long m) {
        if (m <= 0) return 0;
        if (m < 64) return 1;
        if (m < 1024) return 2;
        return 3;
    }

    private static int clampLevel(int L) { return L < 0 ? 0 : (L >= LEVELS ? LEVELS - 1 : L); }
}

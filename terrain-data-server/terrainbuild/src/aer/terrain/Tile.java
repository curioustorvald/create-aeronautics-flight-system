package aer.terrain;

import java.io.IOException;
import java.nio.file.*;

/**
 * One tile = one region = 32 x 32 chunks, and one file on the CC disk.
 *
 * Tiles are region-aligned on purpose: it makes an incremental pass "one .mca in, one
 * file out", so a rebuild never has to touch a neighbour, and it makes the terrain
 * server's cache unit the same thing as the builder's work unit.
 *
 * Everything multi-byte is LITTLE-endian, because the reader is Lua: b1 + b2*256 is the
 * natural way to put a number back together with string.byte and there is no ntohl.
 *
 *   0   "AGT1"
 *   4   rx        int32
 *   8   rz        int32
 *   12  built     uint32   unix seconds
 *   16  flags     uint16   bit0 = every chunk in the region was readable
 *   18  chunks    uint16   how many of the 1024 carry data
 *   20  (12 bytes reserved, zero)
 *   32  1024 records x 4 bytes, indexed (localZ * 32 + localX)
 */
final class Tile {
    static final int HEADER = 32, RECORDS = 1024, RECSZ = 4, SIZE = HEADER + RECORDS * RECSZ;
    static final String EXT = ".agt";

    final byte[] buf = new byte[SIZE];
    int present;
    boolean complete = true;

    void begin(int rx, int rz) {
        java.util.Arrays.fill(buf, (byte) 0);
        buf[0] = 'A'; buf[1] = 'G'; buf[2] = 'T'; buf[3] = '1';
        le32(4, rx); le32(8, rz);
        present = 0; complete = true;
        // Every record starts as NODATA; analyzed chunks overwrite it.
        for (int i = 0; i < RECORDS; i++) {
            int o = HEADER + i * RECSZ;
            buf[o] = (byte) 0xFF; buf[o + 1] = (byte) 0xFF; buf[o + 2] = (byte) 0xFF;
            buf[o + 3] = (byte) ChunkAnalyzer.F_NODATA;
        }
    }

    void put(int lx, int lz, int terr, int otop, int obas, int flag) {
        int o = HEADER + ((lz << 5) | lx) * RECSZ;
        buf[o] = (byte) terr; buf[o + 1] = (byte) otop; buf[o + 2] = (byte) obas; buf[o + 3] = (byte) flag;
        if ((flag & ChunkAnalyzer.F_NODATA) == 0) present++;
    }

    void write(Path dir, int rx, int rz) throws IOException {
        le32(12, (int) (System.currentTimeMillis() / 1000L));
        buf[16] = (byte) (complete ? 1 : 0); buf[17] = 0;
        buf[18] = (byte) (present & 0xFF); buf[19] = (byte) ((present >>> 8) & 0xFF);

        Path out = dir.resolve(name(rx, rz));
        Path tmp = dir.resolve(name(rx, rz) + ".tmp");
        Files.write(tmp, buf);
        try {
            Files.move(tmp, out, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
        } catch (AtomicMoveNotSupportedException e) {
            Files.move(tmp, out, StandardCopyOption.REPLACE_EXISTING);
        }
    }

    static String name(int rx, int rz) { return rx + "_" + rz + EXT; }

    private void le32(int o, int v) {
        buf[o] = (byte) v; buf[o + 1] = (byte) (v >>> 8);
        buf[o + 2] = (byte) (v >>> 16); buf[o + 3] = (byte) (v >>> 24);
    }
}

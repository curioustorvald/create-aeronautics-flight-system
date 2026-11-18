package aer.terrain;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

/**
 * One .mca (Anvil) region file: 32x32 chunks, an 8 KiB header, then chunk payloads on
 * 4 KiB sector boundaries.
 *
 * The whole file is read in one go. Regions here run 2-20 MB and the host has NVMe and
 * 123 GB of RAM, so streaming would buy nothing and cost seeks.
 *
 * A torn or unreadable chunk is NOT fatal. When this runs against a live server the file
 * is being written underneath us, so a chunk can be caught mid-flush; that chunk is
 * skipped and the region's mtime will bring us back to it on the next pass.
 */
final class Region {
    static final int SECTOR = 4096;

    final int rx, rz;
    private final byte[] file;

    private final Inflater inflater = new Inflater();
    private byte[] out = new byte[1 << 20];

    /** Set when a chunk used a compression scheme we cannot read (LZ4/custom). */
    int unsupported = 0;
    int torn = 0;

    Region(int rx, int rz, Path p) throws IOException {
        this.rx = rx; this.rz = rz;
        this.file = Files.readAllBytes(p);
    }

    private int u8(int i) { return file[i] & 0xFF; }
    private int be32(int i) { return (u8(i) << 24) | (u8(i + 1) << 16) | (u8(i + 2) << 8) | u8(i + 3); }

    /**
     * Inflate chunk (cx,cz) -- both 0..31, local to the region -- into the scratch buffer.
     * Returns the payload length, or -1 when the chunk is absent or unreadable.
     * The bytes live in {@link #buffer()} until the next call.
     */
    int read(int cx, int cz) {
        if (file.length < 8192) return -1;
        int e = 4 * ((cz << 5) | cx);
        int off = (u8(e) << 16) | (u8(e + 1) << 8) | u8(e + 2);
        int cnt = u8(e + 3);
        if (off == 0 || cnt == 0) return -1;                 // never generated

        long base = (long) off * SECTOR;
        if (base + 5 > file.length) { torn++; return -1; }

        int len = be32((int) base);                          // includes the compression byte
        int comp = u8((int) base + 4);
        if (len <= 1 || base + 4 + len > file.length) { torn++; return -1; }

        if ((comp & 0x80) != 0) { unsupported++; return -1; } // payload lives in an .mcc sidecar

        int dataOff = (int) base + 5, dataLen = len - 1;

        try {
            switch (comp) {
                case 1:  return gunzip(dataOff, dataLen);
                case 2:  return inflate(dataOff, dataLen, false);
                case 3:  ensure(dataLen); System.arraycopy(file, dataOff, out, 0, dataLen); return dataLen;
                default: unsupported++; return -1;            // 4 = LZ4, 127 = custom
            }
        } catch (Exception ex) {
            torn++;
            return -1;
        }
    }

    byte[] buffer() { return out; }

    private void ensure(int n) { if (out.length < n) out = new byte[Integer.highestOneBit(n - 1) << 2]; }

    private int inflate(int off, int len, boolean raw) throws DataFormatException {
        inflater.reset();
        inflater.setInput(file, off, len);
        int total = 0;
        for (;;) {
            if (total == out.length) out = java.util.Arrays.copyOf(out, out.length * 2);
            int n = inflater.inflate(out, total, out.length - total);
            if (n == 0) {
                if (inflater.finished() || inflater.needsInput() || inflater.needsDictionary()) break;
            }
            total += n;
        }
        return total;
    }

    /** Chunks written with compression type 1 carry a gzip wrapper rather than a zlib one. */
    private int gunzip(int off, int len) throws IOException {
        try (java.util.zip.GZIPInputStream in = new java.util.zip.GZIPInputStream(
                new java.io.ByteArrayInputStream(file, off, len), 1 << 16)) {
            int total = 0;
            for (;;) {
                if (total == out.length) out = java.util.Arrays.copyOf(out, out.length * 2);
                int n = in.read(out, total, out.length - total);
                if (n < 0) break;
                total += n;
            }
            return total;
        }
    }

    void close() { inflater.end(); }
}

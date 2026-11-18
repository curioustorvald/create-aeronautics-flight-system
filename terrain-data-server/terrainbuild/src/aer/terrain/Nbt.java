package aer.terrain;

/**
 * A cursor over an uncompressed NBT payload.
 *
 * This is deliberately NOT a tree parser. A 1.21 chunk carries block_entities, biome
 * palettes and (on some saves) light arrays that we never look at, and materialising them
 * would dominate the run: 6.5M chunks x ~100 kB of tags is not something to allocate.
 * Instead the analyser walks the tags it wants and calls skip() on everything else, so a
 * chunk costs one inflate plus a scan.
 *
 * NBT is big-endian throughout. Names are compared as raw bytes (see nameIs) so that a
 * walk allocates no Strings at all.
 */
final class Nbt {
    static final int END = 0, BYTE = 1, SHORT = 2, INT = 3, LONG = 4, FLOAT = 5, DOUBLE = 6,
            BYTE_ARRAY = 7, STRING = 8, LIST = 9, COMPOUND = 10, INT_ARRAY = 11, LONG_ARRAY = 12;

    byte[] b;
    int p;
    int len;

    void reset(byte[] buf, int off, int length) { this.b = buf; this.p = off; this.len = off + length; }

    int u8()  { return b[p++] & 0xFF; }
    int i8()  { return b[p++]; }
    int u16() { return ((b[p++] & 0xFF) << 8) | (b[p++] & 0xFF); }
    int i32() { return ((b[p++] & 0xFF) << 24) | ((b[p++] & 0xFF) << 16) | ((b[p++] & 0xFF) << 8) | (b[p++] & 0xFF); }
    long i64() { long v = 0; for (int i = 0; i < 8; i++) v = (v << 8) | (b[p++] & 0xFF); return v; }

    /** Length of the name/string at the cursor; leaves the cursor on the first byte of it. */
    int strLen() { return u16(); }

    /** True if the `n` bytes at the cursor equal the ASCII `s`. Does not move the cursor. */
    boolean bytesAre(int off, int n, String s) {
        if (n != s.length()) return false;
        for (int i = 0; i < n; i++) if ((b[off + i] & 0xFF) != s.charAt(i)) return false;
        return true;
    }

    boolean endsWith(int off, int n, String s) {
        if (n < s.length()) return false;
        int d = n - s.length();
        for (int i = 0; i < s.length(); i++) if ((b[off + d + i] & 0xFF) != s.charAt(i)) return false;
        return true;
    }

    String str(int off, int n) { return new String(b, off, n, java.nio.charset.StandardCharsets.UTF_8); }

    /** Skip a payload of the given tag type. The cursor must sit on the payload. */
    void skip(int type) {
        switch (type) {
            case BYTE: p += 1; break;
            case SHORT: p += 2; break;
            case INT: case FLOAT: p += 4; break;
            case LONG: case DOUBLE: p += 8; break;
            // NB: each of these reads its length FIRST and advances p only afterwards.
            // `p += i32()` would be wrong -- Java evaluates the left operand before the
            // right, so the bytes i32() itself consumed would be thrown away.
            case BYTE_ARRAY:  { int n = i32(); p += n;     break; }
            case STRING:      { int n = u16(); p += n;     break; }
            case INT_ARRAY:   { int n = i32(); p += 4 * n; break; }
            case LONG_ARRAY:  { int n = i32(); p += 8 * n; break; }
            case LIST: {
                int et = u8();
                int n = i32();
                if (et == END) break;
                // Fixed-width payloads can be stepped over in one multiply.
                int w = width(et);
                if (w > 0) { p += w * n; break; }
                for (int i = 0; i < n; i++) skip(et);
                break;
            }
            case COMPOUND: {
                for (;;) {
                    int t = u8();
                    if (t == END) break;
                    int nl = u16(); p += nl;      // the name
                    skip(t);
                }
                break;
            }
            default: throw new IllegalStateException("bad NBT tag " + type);
        }
    }

    private static int width(int type) {
        switch (type) {
            case BYTE: return 1;
            case SHORT: return 2;
            case INT: case FLOAT: return 4;
            case LONG: case DOUBLE: return 8;
            default: return -1;
        }
    }
}

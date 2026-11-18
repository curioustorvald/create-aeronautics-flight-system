package aer.terrain;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.HashMap;
import java.util.Map;

/**
 * "Which regions did we already do, and what did they look like when we did them."
 *
 * Plain text on purpose: it lives next to the jar, not on the CC disk, and being able to
 * grep it is worth more than the few hundred kB it costs.
 */
final class State {
    static Map<Long, long[]> load(Path p) {
        Map<Long, long[]> m = new HashMap<>();
        if (!Files.exists(p)) return m;
        try {
            for (String line : Files.readAllLines(p, StandardCharsets.UTF_8)) {
                if (line.isEmpty() || line.charAt(0) == '#') continue;
                String[] f = line.split(" ");
                if (f.length != 4) continue;
                m.put(Main.key(Integer.parseInt(f[0]), Integer.parseInt(f[1])),
                      new long[]{ Long.parseLong(f[2]), Long.parseLong(f[3]) });
            }
        } catch (Exception e) {
            System.err.println("state unreadable (" + e + ") -- treating as empty");
            return new HashMap<>();
        }
        return m;
    }

    static void save(Path p, Map<Long, long[]> m) throws IOException {
        StringBuilder sb = new StringBuilder(m.size() * 32);
        sb.append("# aer terrainbuild state v1: rx rz mtimeMillis size\n");
        for (Map.Entry<Long, long[]> e : m.entrySet()) {
            sb.append((int) (e.getKey() >> 32)).append(' ').append((int) (long) e.getKey())
              .append(' ').append(e.getValue()[0]).append(' ').append(e.getValue()[1]).append('\n');
        }
        Path tmp = p.resolveSibling(p.getFileName() + ".tmp");
        Files.createDirectories(p.toAbsolutePath().getParent());
        Files.write(tmp, sb.toString().getBytes(StandardCharsets.UTF_8));
        Files.move(tmp, p, StandardCopyOption.REPLACE_EXISTING);
    }
}

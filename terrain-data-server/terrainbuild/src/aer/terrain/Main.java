package aer.terrain;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.atomic.*;

/**
 * terrainbuild -- builds the GPWS terrain database out of the Anvil region files.
 *
 *   build    every region, from scratch. Run it with the server DOWN.
 *   update   only the regions whose .mca changed since last time, newest first, and --
 *            with --near -- only those close to an aircraft that has recently reported.
 *            Meant to run against a LIVE server, so it is throttled and it treats a torn
 *            chunk as "come back next pass" rather than as an error.
 *   stats    what is on the disk now.
 *   probe    one chunk, in full detail, straight from the world. For arguing with the
 *            classifier.
 *   image    a PNG of an area, rendered from the DATABASE. The fastest way to see whether
 *            the thing charted what you think it charted.
 *
 * WHY IT IS A SEPARATE PROCESS. Nothing here runs inside the server JVM: it only ever
 * READS world/region and only ever WRITES the CC disk, so the worst it can do to a live
 * server is compete for page cache. That is what --threads, --throttle-ms and
 * --max-regions are for.
 */
public final class Main {

    // The "world" folder beside the jar: the jar is meant to sit next to the server launcher.
    static final String DEF_WORLD = jarDir().resolve("world").toString();

    // ---- options ---------------------------------------------------------------------
    static Path world, regionDir, outDir, statePath, activityPath;
    static int threads, maxRegions, throttleMs;
    static double radius;
    static boolean force, quiet;
    static List<double[]> near = new ArrayList<>();

    public static void main(String[] args) throws Exception {
        if (args.length == 0) { usage(); return; }
        String cmd = args[0];

        world = Paths.get(DEF_WORLD);
        Path outOverride = null, stateOverride = null, actOverride = null;
        boolean update = cmd.equals("update");
        threads = update ? 2 : Math.min(Runtime.getRuntime().availableProcessors(), 16);
        maxRegions = update ? 64 : Integer.MAX_VALUE;
        throttleMs = update ? 25 : 0;
        radius = 8000;
        String imgOut = "terrain.png";
        int imgSize = 512;
        double px = 0, pz = 0;
        boolean haveP = false;

        for (int i = 1; i < args.length; i++) {
            String a = args[i];
            switch (a) {
                case "--world":       world = Paths.get(args[++i]); break;
                case "--out":         outOverride = Paths.get(args[++i]); break;
                case "--state":       stateOverride = Paths.get(args[++i]); break;
                case "--activity":    actOverride = Paths.get(args[++i]); break;
                case "--threads":     threads = Integer.parseInt(args[++i]); break;
                case "--max-regions": maxRegions = Integer.parseInt(args[++i]); break;
                case "--throttle-ms": throttleMs = Integer.parseInt(args[++i]); break;
                case "--radius":      radius = Double.parseDouble(args[++i]); break;
                case "--near": {
                    String[] xy = args[++i].split(",");
                    near.add(new double[]{ Double.parseDouble(xy[0]), Double.parseDouble(xy[1]) });
                    break;
                }
                case "--force":  force = true; break;
                case "--quiet":  quiet = true; break;
                case "--png":    imgOut = args[++i]; break;
                case "--size":   imgSize = Integer.parseInt(args[++i]); break;
                case "--at": {
                    String[] xy = args[++i].split(",");
                    px = Double.parseDouble(xy[0]); pz = Double.parseDouble(xy[1]); haveP = true;
                    break;
                }
                default:
                    // Bare "x z" for probe/image. Tested by parseability rather than by a
                    // leading "-", because half the interesting coordinates are negative.
                    if (!haveP && isNumber(a) && i + 1 < args.length && isNumber(args[i + 1])) {
                        px = Double.parseDouble(a); pz = Double.parseDouble(args[++i]); haveP = true;
                    } else { System.err.println("unknown option " + a); usage(); return; }
            }
        }

        regionDir    = world.resolve("region");
        outDir       = outOverride  != null ? outOverride  : world.resolve("computercraft/disk/1/terrain");
        statePath    = stateOverride != null ? stateOverride : jarDir().resolve("terrainbuild.state");
        activityPath = actOverride   != null ? actOverride   : world.resolve("computercraft/computer/68/activity.txt");

        switch (cmd) {
            case "build":  run(true);  break;
            case "update": run(false); break;
            case "stats":  stats();    break;
            case "probe":  probe(px, pz); break;
            case "image":  image(px, pz, imgSize, Paths.get(imgOut)); break;
            default: usage();
        }
    }

    static boolean isNumber(String s) {
        try { Double.parseDouble(s); return true; } catch (NumberFormatException e) { return false; }
    }

    static void usage() {
        System.out.println("""
            terrainbuild -- GPWS terrain database builder

              build   [opts]              full rebuild (server should be stopped)
              update  [opts]              only regions whose .mca changed
              stats   [opts]              report what is in the database
              probe   <x> <z> [opts]      analyse one chunk from the world, verbosely
              image   <x> <z> [opts]      render a PNG of the database around x,z

            options
              --world DIR         default <directory holding the jar>/world
              --out DIR           tile directory  (default <world>/computercraft/disk/1/terrain)
              --state FILE        incremental state (default beside the jar)
              --activity FILE     aircraft positions written by CC #68
                                  (default <world>/computercraft/computer/68/activity.txt)
              --near X,Z          restrict/prioritise around this point (repeatable)
              --radius M          how far from --near / activity to bother (default 8000)
              --threads N         workers        (build: min(cores,16), update: 2)
              --max-regions N     cap per run    (build: all, update: 64)
              --throttle-ms N     pause per region per worker (build: 0, update: 25)
              --force             ignore the state file and redo everything
              --quiet
              --png FILE --size N   for `image`
            """);
    }

    static Path jarDir() {
        try {
            Path p = Paths.get(Main.class.getProtectionDomain().getCodeSource().getLocation().toURI());
            return Files.isDirectory(p) ? p : p.getParent();
        } catch (Exception e) { return Paths.get("."); }
    }

    // =====================================================================================
    // BUILD / UPDATE
    // =====================================================================================

    record Job(int rx, int rz, Path path, long mtime, long size, double dist) {}

    static void run(boolean full) throws Exception {
        if (!Files.isDirectory(regionDir)) { System.err.println("no region dir: " + regionDir); System.exit(2); }
        Files.createDirectories(outDir);

        Map<Long, long[]> state = force ? new HashMap<>() : State.load(statePath);
        if (!full || !near.isEmpty()) loadActivity();

        List<Job> jobs = new ArrayList<>();
        int skipped = 0;
        try (DirectoryStream<Path> ds = Files.newDirectoryStream(regionDir, "r.*.mca")) {
            for (Path p : ds) {
                String[] f = p.getFileName().toString().split("\\.");
                if (f.length != 4) continue;
                int rx, rz;
                try { rx = Integer.parseInt(f[1]); rz = Integer.parseInt(f[2]); } catch (Exception e) { continue; }

                long mt, sz;
                try { mt = Files.getLastModifiedTime(p).toMillis(); sz = Files.size(p); } catch (IOException e) { continue; }
                if (sz < 8192) continue;

                double d = distanceToInterest(rx, rz);
                if (!near.isEmpty() && d > radius) continue;

                long key = key(rx, rz);
                long[] st = state.get(key);
                boolean stale = force || st == null || st[0] != mt || st[1] != sz
                        || !Files.exists(outDir.resolve(Tile.name(rx, rz)));
                if (!stale) { skipped++; continue; }

                jobs.add(new Job(rx, rz, p, mt, sz, d));
            }
        }

        // Nearest first when we know where the aircraft are; otherwise most-recently-written
        // first, which is the same instinct -- that is where the players have been.
        if (!near.isEmpty()) jobs.sort(Comparator.comparingDouble(Job::dist));
        else if (!full)      jobs.sort(Comparator.comparingLong(Job::mtime).reversed());

        if (jobs.size() > maxRegions) jobs = jobs.subList(0, maxRegions);

        if (!quiet) System.out.printf("%s: %d region(s) to do, %d already current%s%n",
                full ? "build" : "update", jobs.size(), skipped,
                near.isEmpty() ? "" : String.format(", %d interest point(s) r=%.0fm", near.size(), radius));
        if (jobs.isEmpty()) { State.save(statePath, state); writeMeta(); return; }

        AtomicInteger cursor = new AtomicInteger();
        AtomicLong chunksDone = new AtomicLong(), regionsDone = new AtomicLong();
        AtomicLong obstacles = new AtomicLong(), torn = new AtomicLong(), unsup = new AtomicLong();
        Map<Long, long[]> fresh = Collections.synchronizedMap(new HashMap<>());
        final List<Job> work = jobs;
        long t0 = System.nanoTime();

        Thread[] pool = new Thread[Math.max(1, threads)];
        for (int t = 0; t < pool.length; t++) {
            pool[t] = new Thread(() -> {
                Nbt nbt = new Nbt();
                ChunkAnalyzer an = new ChunkAnalyzer();
                Tile tile = new Tile();
                for (;;) {
                    int i = cursor.getAndIncrement();
                    if (i >= work.size()) return;
                    Job j = work.get(i);
                    try {
                        long[] r = doRegion(j, nbt, an, tile);
                        chunksDone.addAndGet(r[0]);
                        obstacles.addAndGet(r[1]);
                        torn.addAndGet(r[2]);
                        unsup.addAndGet(r[3]);
                        fresh.put(key(j.rx, j.rz), new long[]{ j.mtime, j.size });
                    } catch (Throwable e) {
                        System.err.println("region " + j.rx + "," + j.rz + ": " + e);
                    }
                    long n = regionsDone.incrementAndGet();
                    if (!quiet && (n % 200 == 0 || n == work.size())) {
                        double el = (System.nanoTime() - t0) / 1e9;
                        System.out.printf("  %d/%d regions  %.0f reg/s  %.1fM chunks  %.0fs elapsed%n",
                                n, work.size(), n / el, chunksDone.get() / 1e6, el);
                    }
                    if (throttleMs > 0) { try { Thread.sleep(throttleMs); } catch (InterruptedException ie) { return; } }
                }
            }, "terrain-" + t);
            pool[t].start();
        }
        for (Thread th : pool) th.join();

        state.putAll(fresh);
        State.save(statePath, state);
        writeMeta();

        double el = (System.nanoTime() - t0) / 1e9;
        System.out.printf("done: %d regions, %d chunks, %d with obstacles, %.1fs%s%n",
                regionsDone.get(), chunksDone.get(), obstacles.get(), el,
                (torn.get() + unsup.get() > 0)
                    ? String.format("  (%d torn, %d unsupported compression)", torn.get(), unsup.get()) : "");
    }

    /** @return {chunks, obstacleChunks, torn, unsupported} */
    static long[] doRegion(Job j, Nbt nbt, ChunkAnalyzer an, Tile tile) throws IOException {
        Region reg = new Region(j.rx, j.rz, j.path);
        tile.begin(j.rx, j.rz);
        long n = 0, obs = 0;
        try {
            for (int lz = 0; lz < 32; lz++) {
                for (int lx = 0; lx < 32; lx++) {
                    int len = reg.read(lx, lz);
                    if (len <= 0) continue;
                    boolean ok;
                    try { ok = an.analyze(nbt, reg.buffer(), len); }
                    catch (Throwable e) { ok = false; tile.complete = false; }
                    if (!ok) continue;
                    tile.put(lx, lz, an.terr, an.otop, an.obas, an.flag);
                    n++;
                    if (an.otop != ChunkAnalyzer.NONE) obs++;
                }
            }
            if (reg.torn > 0 || reg.unsupported > 0) tile.complete = false;
            tile.write(outDir, j.rx, j.rz);
            return new long[]{ n, obs, reg.torn, reg.unsupported };
        } finally {
            reg.close();
        }
    }

    static long key(int rx, int rz) { return ((long) rx << 32) | (rz & 0xFFFFFFFFL); }

    static double distanceToInterest(int rx, int rz) {
        if (near.isEmpty()) return 0;
        double cx = rx * 512.0 + 256, cz = rz * 512.0 + 256, best = Double.MAX_VALUE;
        for (double[] a : near) {
            double dx = cx - a[0], dz = cz - a[1];
            double d = Math.sqrt(dx * dx + dz * dz) - 362;   // half-diagonal of a region
            if (d < best) best = d;
        }
        return Math.max(0, best);
    }

    /** Aircraft positions as computer #68 last saw them on atc_broadcast. */
    static void loadActivity() {
        if (!Files.exists(activityPath)) return;
        long now = System.currentTimeMillis();
        int n = 0;
        try {
            for (String line : Files.readAllLines(activityPath, StandardCharsets.UTF_8)) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith(";") || line.startsWith("#")) continue;
                String[] f = line.split("\\s+");
                if (f.length < 5) continue;
                try {
                    double x = Double.parseDouble(f[1]), z = Double.parseDouble(f[2]);
                    long seen = Long.parseLong(f[4]);
                    if (now - seen > 15 * 60_000L) continue;      // quarter of an hour is plenty
                    near.add(new double[]{ x, z });
                    n++;
                } catch (NumberFormatException ignored) { }
            }
        } catch (IOException e) { return; }
        if (!quiet && n > 0) System.out.println("activity: " + n + " aircraft from " + activityPath);
    }

    static void writeMeta() throws IOException {
        int tiles = 0;
        try (DirectoryStream<Path> ds = Files.newDirectoryStream(outDir, "*" + Tile.EXT)) {
            for (Path ignored : ds) tiles++;
        }
        String s = "; aer terrain database -- built by terrainbuild.jar, do not hand-edit\n"
                 + "version=1\n"
                 + "minY=" + ChunkAnalyzer.MIN_Y + "\n"
                 + "maxY=" + (ChunkAnalyzer.MIN_Y + ChunkAnalyzer.CELL * ChunkAnalyzer.LEVELS) + "\n"
                 + "cell=" + ChunkAnalyzer.CELL + "\n"
                 + "levels=" + ChunkAnalyzer.LEVELS + "\n"
                 + "tile=32\n"
                 + "record=" + Tile.RECSZ + "\n"
                 + "header=" + Tile.HEADER + "\n"
                 + "tiles=" + tiles + "\n"
                 + "built=" + (System.currentTimeMillis() / 1000L) + "\n";
        Path tmp = outDir.resolve("meta.txt.tmp");
        Files.write(tmp, s.getBytes(StandardCharsets.UTF_8));
        Files.move(tmp, outDir.resolve("meta.txt"), StandardCopyOption.REPLACE_EXISTING);
    }

    // =====================================================================================
    // STATS / PROBE / IMAGE
    // =====================================================================================

    static void stats() throws IOException {
        long tiles = 0, chunks = 0, obs = 0, water = 0, spike = 0, voids = 0, incomplete = 0, bytes = 0;
        long[] massHist = new long[4];
        int minRx = Integer.MAX_VALUE, maxRx = Integer.MIN_VALUE, minRz = Integer.MAX_VALUE, maxRz = Integer.MIN_VALUE;
        long oldest = Long.MAX_VALUE, newest = 0;

        if (!Files.isDirectory(outDir)) { System.out.println("no database at " + outDir); return; }
        try (DirectoryStream<Path> ds = Files.newDirectoryStream(outDir, "*" + Tile.EXT)) {
            for (Path p : ds) {
                byte[] b = Files.readAllBytes(p);
                if (b.length != Tile.SIZE || b[0] != 'A' || b[1] != 'G' || b[2] != 'T') continue;
                tiles++; bytes += b.length;
                int rx = le32(b, 4), rz = le32(b, 8);
                long built = le32(b, 12) & 0xFFFFFFFFL;
                minRx = Math.min(minRx, rx); maxRx = Math.max(maxRx, rx);
                minRz = Math.min(minRz, rz); maxRz = Math.max(maxRz, rz);
                oldest = Math.min(oldest, built); newest = Math.max(newest, built);
                if (b[16] == 0) incomplete++;
                for (int i = 0; i < Tile.RECORDS; i++) {
                    int o = Tile.HEADER + i * Tile.RECSZ;
                    int fl = b[o + 3] & 0xFF;
                    if ((fl & ChunkAnalyzer.F_NODATA) != 0) continue;
                    chunks++;
                    if ((fl & ChunkAnalyzer.F_VOID) != 0) voids++;
                    if ((fl & ChunkAnalyzer.F_WATER) != 0) water++;
                    if ((fl & ChunkAnalyzer.F_SPIKE) != 0) spike++;
                    if ((b[o + 1] & 0xFF) != ChunkAnalyzer.NONE) { obs++; massHist[fl & 3]++; }
                }
            }
        }
        System.out.printf("database  %s%n", outDir);
        System.out.printf("  tiles      %d  (%.1f MiB on disk, %d incomplete)%n", tiles, bytes / 1048576.0, incomplete);
        System.out.printf("  chunks     %d charted  (%.1f million)%n", chunks, chunks / 1e6);
        System.out.printf("  extent     region x %d..%d  z %d..%d  = blocks x %d..%d  z %d..%d%n",
                minRx, maxRx, minRz, maxRz, minRx * 512, maxRx * 512 + 511, minRz * 512, maxRz * 512 + 511);
        System.out.printf("  obstacles  %d chunks (%.2f%%)   thin %d  medium %d  large %d%n",
                obs, chunks == 0 ? 0 : 100.0 * obs / chunks, massHist[1], massHist[2], massHist[3]);
        System.out.printf("  water      %d (%.1f%%)   spike %d   void %d%n",
                water, chunks == 0 ? 0 : 100.0 * water / chunks, spike, voids);
        if (tiles > 0) System.out.printf("  built      %s .. %s%n", new Date(oldest * 1000), new Date(newest * 1000));
    }

    static void probe(double x, double z) throws IOException {
        int cx = (int) Math.floor(x / 16), cz = (int) Math.floor(z / 16);
        int rx = cx >> 5, rz = cz >> 5, lx = cx & 31, lz = cz & 31;
        Path p = regionDir.resolve("r." + rx + "." + rz + ".mca");
        System.out.printf("world %.1f,%.1f -> chunk %d,%d -> region %d,%d local %d,%d%n  %s%n",
                x, z, cx, cz, rx, rz, lx, lz, p);
        if (!Files.exists(p)) { System.out.println("  region not generated"); return; }

        Region reg = new Region(rx, rz, p);
        int len = reg.read(lx, lz);
        if (len <= 0) { System.out.println("  chunk not generated"); reg.close(); return; }

        Nbt nbt = new Nbt();
        ChunkAnalyzer an = new ChunkAnalyzer();
        boolean ok = an.analyze(nbt, reg.buffer(), len);
        System.out.println("  nbt " + len + " bytes, analysed=" + ok);
        if (ok) {
            System.out.printf("  TERR level %d  -> Y %d..%d (reported %d)%n",
                    an.terr, ChunkAnalyzer.MIN_Y + 8 * an.terr, ChunkAnalyzer.MIN_Y + 8 * an.terr + 7,
                    ChunkAnalyzer.MIN_Y + 8 * an.terr + 7);
            if (an.otop != ChunkAnalyzer.NONE)
                System.out.printf("  OBST base level %d (Y %d)  top level %d (Y %d)%n",
                        an.obas, ChunkAnalyzer.MIN_Y + 8 * an.obas,
                        an.otop, ChunkAnalyzer.MIN_Y + 8 * an.otop + 7);
            else System.out.println("  OBST none");
            System.out.printf("  FLAG 0x%02X  mass=%d%s%s%s%s%s%n", an.flag, an.flag & 3,
                    (an.flag & ChunkAnalyzer.F_WATER) != 0 ? " WATER" : "",
                    (an.flag & ChunkAnalyzer.F_MULTI) != 0 ? " MULTI" : "",
                    (an.flag & ChunkAnalyzer.F_TREES) != 0 ? " TREES" : "",
                    (an.flag & ChunkAnalyzer.F_SPIKE) != 0 ? " SPIKE" : "",
                    (an.flag & ChunkAnalyzer.F_VOID)  != 0 ? " VOID"  : "");
            System.out.println("  occupancy (level: Y range = solid blocks of 2048)");
            int[] c = an.counts();
            for (int L = ChunkAnalyzer.LEVELS - 1; L >= 0; L--) {
                if (c[L] == 0) continue;
                int y0 = ChunkAnalyzer.MIN_Y + 8 * L;
                System.out.printf("    %2d: %4d..%4d  %5d  %s%s%n", L, y0, y0 + 7, c[L],
                        "#".repeat(Math.min(40, 1 + c[L] * 40 / 2048)),
                        L == an.terr ? "  <- TERRAIN TOP" : (L == an.otop ? "  <- OBSTACLE TOP"
                                : (L == an.obas ? "  <- OBSTACLE BASE" : "")));
            }
        }
        reg.close();
    }

    /** Render the DATABASE (not the world) so the picture proves what the aircraft will read. */
    static void image(double x, double z, int sizeChunks, Path png) throws IOException {
        int cx0 = (int) Math.floor(x / 16) - sizeChunks / 2, cz0 = (int) Math.floor(z / 16) - sizeChunks / 2;
        java.awt.image.BufferedImage img =
                new java.awt.image.BufferedImage(sizeChunks, sizeChunks, java.awt.image.BufferedImage.TYPE_INT_RGB);
        Map<Long, byte[]> cache = new HashMap<>();
        int charted = 0, obs = 0;

        for (int j = 0; j < sizeChunks; j++) {
            for (int i = 0; i < sizeChunks; i++) {
                int cx = cx0 + i, cz = cz0 + j;
                byte[] t = cache.computeIfAbsent(key(cx >> 5, cz >> 5), k -> {
                    Path p = outDir.resolve(Tile.name((int) (k >> 32), (int) (long) k));
                    try { return Files.exists(p) ? Files.readAllBytes(p) : new byte[0]; }
                    catch (IOException e) { return new byte[0]; }
                });
                int rgb = 0x101014;
                if (t.length == Tile.SIZE) {
                    int o = Tile.HEADER + (((cz & 31) << 5) | (cx & 31)) * Tile.RECSZ;
                    int terr = t[o] & 0xFF, otop = t[o + 1] & 0xFF, fl = t[o + 3] & 0xFF;
                    if ((fl & ChunkAnalyzer.F_NODATA) == 0) {
                        charted++;
                        int y = ChunkAnalyzer.MIN_Y + 8 * terr;
                        int s = Math.max(0, Math.min(255, (y + 64) * 255 / 256));
                        if ((fl & ChunkAnalyzer.F_WATER) != 0) rgb = (s / 4 << 16) | (s / 2 << 8) | Math.min(255, s + 90);
                        else rgb = (s << 16) | (s << 8) | (s * 3 / 4);
                        if (otop != ChunkAnalyzer.NONE) {           // obstacles paint over the ground
                            obs++;
                            int m = fl & 3;
                            rgb = m >= 3 ? 0xFF2020 : (m == 2 ? 0xFF8000 : 0xFFD000);
                        }
                    }
                }
                img.setRGB(i, j, rgb);
            }
        }
        javax.imageio.ImageIO.write(img, "png", png.toFile());
        System.out.printf("%s  %dx%d chunks (%d x %d blocks) centred %.0f,%.0f  %d charted, %d obstacle%n",
                png, sizeChunks, sizeChunks, sizeChunks * 16, sizeChunks * 16, x, z, charted, obs);
    }

    static int le32(byte[] b, int o) {
        return (b[o] & 0xFF) | ((b[o + 1] & 0xFF) << 8) | ((b[o + 2] & 0xFF) << 16) | ((b[o + 3] & 0xFF) << 24);
    }
}

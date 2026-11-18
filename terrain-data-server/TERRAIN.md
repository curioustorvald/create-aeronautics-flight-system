# GPWS terrain data

The fleet's ground-proximity database: what it is, how it is built, and how an aeroplane
asks it questions.

Before this existed, `67/gpws.lua` carried a documented stub — every terrain-derived mode
was wired, priorities and voice clips and all, behind `TERRAIN.available()`, which always
returned false. There was no terrain data in this world and the GPWS refused to pretend
otherwise. This is the missing half.

> **In this package** the builder's source is in `terrainbuild/` (run `build.sh`, JDK 17+), and `terrain-update.sh` is the cron wrapper. Computer numbers below (#67, #68) are the author's; the builder's `--activity` default points at computer 68's `activity.txt`, so pass `--activity` (and `--out` for the disk) if your terrain server has another id.

## The three pieces

| Piece | Where | What it does |
|---|---|---|
| `terrainbuild.jar` | beside the server launcher (build it with `terrainbuild/build.sh`) | reads `world/region/*.mca` and writes the database |
| the database | a CC floppy/disk, `terrain/` (the builder's default `--out` is disk **#1**) | one tile per region (the author's world: 2596 tiles, ~11 MB) |
| `terrain_server.lua` | the terrain data server computer (`terrain-data-server/`; **#68** in the author's world) | answers `aer_terrain` queries from the fleet |
| the client | the HUD/GPWS computer (`gpws-hud/gpws.lua`; **#67** in the author's world) | asks, caches, and drives the GPWS modes |

Nothing runs inside the server JVM. The builder is a separate process that only ever
**reads** `world/region` and only ever **writes** the CC disk, so the worst it can do to a
live server is compete for page cache.

## Resolution, and which way it errs

One record per **chunk** (16 × 16 m), one level per **8 m** of Y. The world's −64…320
becomes 48 levels; level `L` covers Y `−64+8L` … `−64+8L+7`.

Every height is reported at the **top** of its cell and is the **highest** ground in its
chunk. It therefore errs **high**, always. That is the safe direction for a GPWS —
clearance reads low, calls come early — and it is why `SITE` still outranks terrain as a
height datum near a published field, where the exact pad elevation is known.

## Terrain vs obstacle

A cell is *occupied* if it holds even one non-air block. Occupied cells form vertical runs.
The run holding the **anchor** — the level of the chunk's lowest surface column, taken from
the `MOTION_BLOCKING` heightmap — is the ground. Every occupied run above it, separated by
at least one wholly empty cell, is a floating mass and is charted as an **obstacle**.

Anchoring on the heightmap rather than on "the run that reaches bedrock" is what keeps
caves out of this, and it is not a theoretical concern. Chunk `−2120,19016` has levels 2–4
(Y −48…−17) completely hollow — a deep cavern under ordinary countryside. Walking up from
bedrock charts the real landscape from Y −24 to 159 as one giant floating obstacle, and a
GPWS that shouts OBSTACLE AHEAD at a grass field gets switched off.

The reverse case — an island covering a chunk's whole footprint, so the lowest surface
column *is* the island — is caught by looking for a void beneath the anchor run, bounded to
**at or above sea level**. Islands float in the sky; caves do not.

`VOID_GAP` is **1 cell**, measured rather than guessed. Chunk `−392,20520` is ground to
Y 63, one empty cell at Y 64…71, then solid mass to Y 271. At a gap of 2 that island — and
60 chunks like it in a 59k sample — was charted as "terrain at Y 271", which loses the
obstacle callout *and* puts the ground 200 m above an aeroplane flying underneath.
Override with `-Daer.voidgap=N`.

Note that clearance is unaffected by the split either way: the corridor reduction takes the
max of terrain and obstacle. The split decides the **wording** of the callout and the value
of `groundAt()`, not whether the aeroplane is warned.

## Tile format

One tile per region, `<rx>_<rz>.agt`, 4128 bytes. Little-endian, because the reader is Lua.

```
 0   "AGT1"
 4   rx        int32
 8   rz        int32
12   built     uint32   unix seconds
16   flags     uint16   bit0 = every chunk in the region was readable
18   chunks    uint16   how many of the 1024 carry data
20   (12 reserved)
32   1024 records x 4 bytes, indexed (localZ * 32 + localX)
```

Record: `TERR`, `OTOP`, `OBAS`, `FLAG` — terrain top level, obstacle top level, obstacle
base level (255 = none), flags.

```
FLAG  bit0-1  obstacle mass   0 none, 1 thin, 2 medium, 3 large
      bit2    WATER    the surface is water
      bit3    MULTI    more than one floating layer
      bit4    TREES    the terrain top is canopy, not ground
      bit5    SPIKE    the terrain top is a thin mast, not a surface
      bit6    VOID     nothing solid anywhere in the column
      bit7    NODATA   absent, unfinished or unreadable
```

The flag bits appear in three places — `ChunkAnalyzer.java`, `68/terrainconf.lua`
(`CFG.FLAG`) and `67/gpwsconf.lua` (`CFG.TERRAIN.FLAG`). They are the wire format; all
three must agree. `meta.txt` states the geometry the database was built with, and #68
refuses to start if it disagrees with `CFG.GEO` — a database built at one Y resolution and
read at another produces confident, plausible, completely wrong heights.

## Protocol `aer_terrain`

Fleet-wide, like `aer_navdata` and `atc_broadcast`. Not to be renamed per aircraft: one
database serves every airframe.

```
srv -> all   {t="beacon", srv=true, id, rev, tiles, geo}     every 15 s
cli -> all   {t="hello", callsign, x, y, z}                  answered with a targeted beacon
cli -> srv   {t="corr", id, x, z, dx, dz, step, n, w, alt}
srv -> cli   {t="corr", srv=true, ..., prof}                 4 bytes per sample
cli -> srv   {t="tile", rx, rz}   ->  {t="tile", srv=true, rx, rz, data}
cli -> srv   {t="gnd",  x, z}     ->  {t="gnd",  srv=true, x, z, rec}
```

`srv=true` marks a message as coming **from** the server. Every message on this protocol is
seen by every listener; without it a client would mistake another aircraft's request for an
answer and latch that aircraft as its server.

The **corridor** is the workhorse. The client asks once every 0.5 s for the worst ground in
a swathe along its ground track; the server reduces across the corridor width and returns a
profile; the 20 Hz core tick re-reads that cached profile against live altitude. Alerting
runs at tick rate while the radio runs at half a hertz. The profile is anchored where the
aircraft *was* when it was cut, and every read projects the current position back onto it —
at 320 m/s a half-second-old profile is already 160 m behind.

## Commands

```sh
java -jar terrainbuild.jar build              # full rebuild -- run with the server DOWN
java -jar terrainbuild.jar update             # only regions whose .mca changed
java -jar terrainbuild.jar stats              # what is in the database now
java -jar terrainbuild.jar probe <x> <z>      # one chunk, verbosely, from the world
java -jar terrainbuild.jar image <x> <z> --png out.png --size 512
```

`probe` prints the whole occupancy column and marks the terrain top and obstacle bounds —
it is how you argue with the classifier. `image` renders from the **database**, so the
picture proves what the aircraft will actually read.

A full build of 2596 regions and 2.06 M chunks takes **~17 s** on 24 threads.

## Keeping it current

`terrain-update.sh` is the periodic pass. It is safe against a live server: niced, I/O-idle,
two threads, 64 regions a pass, 25 ms between regions, and a torn chunk is skipped rather
than treated as an error — the region's mtime brings it round again next pass.

It works **near the fleet**. Computer #68 writes `68/activity.txt` with every aircraft it
has heard from — both from terrain queries and from `atc_broadcast` — and the builder
restricts itself to regions within `--radius` (default 8 km) of one of them. With no
aircraft about it falls back to the most recently modified changed regions.

```
*/5 * * * * /path/to/your/server/terrain-update.sh
```

Expect it to find the spawn regions changed on most passes: the server autosaves the
chunks that stay loaded. That is five regions and about half a second.

## Hardware

Computer #68 wants an **ender modem**. Plain wireless modems reach 384 m at best
(`modem_range` / `modem_high_altitude_range`) and the aircraft ask from tens of kilometres
away. It also needs the disk drive holding disk #1; the server searches every drive for one
whose mount holds `terrain/meta.txt`, so the disk can be moved between drives without an
edit.

`floppy_space_limit` in `config/computercraft-server.toml` must exceed the database size —
the stock 125 000 bytes is far too small. It is currently 2 GiB − 1 against ~11 MB used.

# Ford CAN GPS platform support

What each Ford platform actually broadcasts, measured from real logs rather than inferred
from the DBC. `cangpsd` decodes three messages; which of them exist varies by platform, and
the DBC cannot tell you which — `ford_lincoln_base_pt.dbc` is shared by all 13 Ford
platforms and defines all three for every one of them.

| Address | Message | Carries |
|---|---|---|
| `0x462` | `APIMGPS_Data_Nav_1_FD1` | latitude, longitude, hemisphere enums |
| `0x463` | `APIMGPS_Data_Nav_2_FD1` | UTC date/time, PDOP, fault flag, inferred-position flag |
| `0x464` | `APIMGPS_Data_Nav_3_FD1` | altitude, speed, heading, satellites, HDOP, VDOP, 2D/3D |

## Method

Four segments per platform, spread across each platform's full route list in comma's public
[`commaCarSegments`](https://huggingface.co/datasets/commaai/commaCarSegments) dataset,
reached via `tools/lib/comma_car_segments.py`. Roughly 60 s of bus traffic per segment,
transmit echoes (`src >= 128`) excluded. Frames were decoded with `cangpsd`'s own
`make_parser` / `decode_position` / `decode_utc`, so this measures the production path.
`FORD_ESCAPE_MK4_5` has only one public segment. This run covers all 10 platforms with
public data at 4 segments each, 20 of them across the classic-CAN platforms; an earlier run
over just those platforms at 6 segments each (30 segments) reached the same conclusion
independently. Segment counts quoted elsewhere should defer to this document.

## Message availability

| Platform | CAN FD | Segs | `0x462` | `0x463` | `0x464` | Position | Clock |
|---|---|---|---|---|---|---|---|
| `FORD_ESCAPE_MK4_5` | yes | 1 | 60 | 60 | 60 | yes | yes |
| `FORD_F_150_LIGHTNING_MK1` | yes | 4 | 240 | 240 | 240 | yes | yes |
| `FORD_F_150_MK14` | yes | 4 | 240 | 240 | 240 | yes | yes |
| `FORD_MUSTANG_MACH_E_MK1` | yes | 4 | 240 | 240 | 240 | yes | yes |
| `FORD_RANGER_MK2` | yes | 4 | 240 | 240 | 240 | yes | yes |
| `FORD_BRONCO_SPORT_MK1` | no | 4 | 238 | **0** | 238 | yes | **no** |
| `FORD_ESCAPE_MK4` | no | 4 | 238 | **0** | 238 | yes | **no** |
| `FORD_EXPLORER_MK6` | no | 4 | 237 | **0** | 237 | yes | **no** |
| `FORD_MAVERICK_MK1` | no | 4 | 237 | **0** | 237 | yes | **no** |
| `FORD_FOCUS_MK4` | no | 4 | 240 | **0** | **0** | yes | **no** |

**The split is exactly CAN FD vs classic CAN.** Every CAN FD platform sends all three at
1 Hz. No classic-CAN platform sends `0x463` at all — zero frames across 20 segments — so
those cars can supply position but never the wall clock, which is the entire reason the
daemon exists on the Mach-E. `FORD_FOCUS_MK4` is the outlier that also lacks `0x464`: it
sends position and nothing else.

`decode_position` returned non-`None` on 100% of `0x462` frames on every platform — which
means "returned a value", not "was correct". At the time of measurement it had no check for
the minutes sentinels (`GPS_Latitude_Minutes` / `GPS_Longitude_Minutes`,
`GPS_Latitude_Min_dec` / `GPS_Longitude_Min_dec`), each of which decodes to a plausible
in-range position up to ~1.077° (~120 km) wrong. That gap is since fixed, but the fix
postdates these numbers. UTC decoded on 97–100% of `0x463` frames where present; the
shortfall is `Gps_B_Falt` set or PDOP out of range, which `decode_utc` correctly rejects.

### Not covered

`FORD_EDGE_MK2` (classic CAN), `FORD_EXPEDITION_MK4` and `FORD_MONDEO_MK5` (both CAN FD)
have no public segments. Their behaviour is unmeasured — do not assume the CAN FD
correlation holds for the latter two without checking.

## `0x464` field reliability

Share of `0x464` frames where the field carries a real value rather than a DBC
Unknown/Invalid/Fault sentinel:

| Platform | altitude | speed | heading | HDOP | VDOP | satellites |
|---|---|---|---|---|---|---|
| `FORD_ESCAPE_MK4_5` | yes | yes | yes | yes | yes | 72% |
| `FORD_F_150_LIGHTNING_MK1` | yes | yes | yes | yes | yes | 25% |
| `FORD_F_150_MK14` | yes | yes | yes | yes | yes | 27% |
| `FORD_MUSTANG_MACH_E_MK1` | yes | yes | yes | yes | yes | 25% |
| `FORD_RANGER_MK2` | yes | yes | yes | yes | yes | never |
| `FORD_BRONCO_SPORT_MK1` | yes | yes | yes | 99% | yes | yes |
| `FORD_ESCAPE_MK4` | yes | yes | yes | 99% | yes | yes |
| `FORD_EXPLORER_MK6` | yes | yes | yes | yes | yes | yes |
| `FORD_MAVERICK_MK1` | yes | yes | yes | 99% | yes | yes |
| `FORD_FOCUS_MK4` | — | — | — | — | — | — |

Two things worth noting. **Satellite count is the unreliable field, and it is unreliable in
the opposite direction to everything else** — the classic-CAN platforms report it faithfully
while the CAN FD ones mostly emit the Invalid sentinel. It is intermittent rather than
absent: two Mach-E segments from the same vehicle on identical ECU firmware
(`engine PJ98-14C204-AHK`) disagree, one reporting 29 satellites and the other 31, so this
is not a Ford software revision. `cangpsd` reports 0 (unknown) rather than a fabricated 31.

**HDOP occasionally sentinels on three classic-CAN platforms** (99%, i.e. a handful of
frames). Any consumer of these signals has to handle sentinels per frame, not per platform.

## Fix-quality enums

`GPS_dimension` reads a constant `2` (3D fix) on every platform and every frame sampled —
it never indicated a degraded or absent fix, so it is not useful as a `hasFix` input.

`GPS_Actual_vs_Infer_pos` is `0` (actual position) almost everywhere, but
`FORD_RANGER_MK2` produced one inferred (dead-reckoned) sample out of 240. Rare, but the
APIM does dead-reckon and will publish it as a normal fix, so it is real and unhandled.

## What this means for `cangpsd`

- A fix requires a fresh, valid `0x462`. GPS time is required only on platforms that send
  `0x463`; see `NO_TIME_GRACE`.
- `0x464` may be absent for the life of the drive (`FORD_FOCUS_MK4`) or report sentinels on
  any frame. Unknown values are published as 0 with a saturated accuracy rather than as
  fact; see `QUALITY_UNKNOWN`.
- Neither `VFrameFormat` nor the `_FD1` message-name suffix indicates CAN FD framing or
  platform support. The DBC marks 100% of its messages `VFrameFormat` 14/15 (CAN FD) while
  six platforms are classic CAN, and `carstate.py` reads `_FD1` messages unconditionally on
  those same classic-CAN cars. Only real logs answer this.

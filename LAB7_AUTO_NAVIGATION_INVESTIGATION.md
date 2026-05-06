# Lab 7 Auto Navigation Investigation

## Current Project State

- Branch: `ping-ir`
- Latest pushed commit before this note: `768f3ed Improve Lab7 gap navigation`
- Relevant runtime files:
  - `Lab7/lab7_main.c`
  - `Lab7/navigation.c`
  - `Lab7/scan.c`
  - `Lab7/movement.c`
- PuTTY log reviewed: `/Users/sgill/Downloads/pass2.txt`

## Observed Lab Behavior

During physical testing, auto navigation showed these behaviors:

- Pressing `g` caused an initial scan, then another scan immediately afterward.
- Sometimes auto navigation continued into multiple scan cycles.
- Once it found a viable gap, the CyBot appeared to head toward one of the pillars.
- Before physically hitting the pillar, it backed up and turned around about 180 degrees.

The PuTTY output confirms this sequence:

```text
g
--- Pass 1: IR Edge Detection Scan ---
...
======================== Detected Objects =========================
...
=== AUTONOMOUS MODE: Scanning... ===
...
======================== Detected Gaps =========================
...
chosen gap: 2 Angle: 109.1 deg Width: 47.4 cm
  Turning LEFT 19.1 deg toward target.

[CLIFF] Boundary detected — backing up and turning around.
```

## Current Call Sequence

The double scan is expected from the current code.

1. `lab7_main.c` handles the `g` command.
2. Before entering auto mode, it calls `scan_objects()` to identify the smallest-width target object.
3. It sets:
   - `target_cm = objects[smallest_idx].ping_dist`
   - `target_deg = objects[smallest_idx].center_angle`
4. It calls `nav_auto_drive(sensor_data, target_cm, target_deg)`.
5. `nav_auto_drive()` immediately performs another scan inside its `while (1)` loop.
6. Each loop iteration:
   - scans objects
   - computes gaps
   - selects a viable gap closest to `target_deg`
   - turns toward that gap
   - moves forward up to 300 mm
   - checks cliff/boundary
   - checks bump sensors
   - repeats

This means pressing `g` currently does one target-acquisition scan plus one navigation scan before movement.

## Why It Backed Up and Turned Around

The backing-up behavior did not come from object bump detection. It came from `boundrydetection()` in `Lab7/navigation.c`.

`boundrydetection()` triggers if any cliff signal is:

- `>= 2500`, treated as tape/boundary
- `<= 500`, treated as off-floor/drop-off

When triggered, it does this:

```c
oi_setWheels(0, 0);
uart_sendStr("\r\n[CLIFF] Boundary detected — backing up and turning around.\r\n");
move_backward(sensor_data, BACKUP_DIST_MM);
turn_left(sensor_data, 180.0);
```

That matches the observed behavior: stop, back up, and turn around almost 180 degrees.

## Important Timing Issue

Boundary detection is currently checked after `move_forward()` returns:

```c
dist_traveled_mm = move_forward(sensor_data, move_mm);
...
if (boundrydetection(sensor_data))
{
    continue;
}
```

Inside `move_forward()`, the robot only stops early for:

- `command_flag`
- `bumpLeft`
- `bumpRight`

It does not stop early for cliff/tape/boundary values. So the robot can start driving toward a chosen gap and only react to boundary values after completing the forward step or stopping from a bump. With the new 300 mm auto step, this can make boundary response feel late.

## Gap Selection From `pass2.txt`

The second navigation scan detected:

```text
Object 1: start=0,   end=52,  center=26,  distance=40.7 cm
Object 2: start=108, end=126, center=117, distance=37.9 cm
Object 3: start=162, end=168, center=165, distance=72.6 cm
```

It computed these gaps:

```text
Gap 1: 52-108 deg,  width=37.0 cm, viable=1
Gap 2: 126-162 deg, width=47.4 cm, viable=1
Gap 3: 168-180 deg, width=15.2 cm, viable=0
```

It chose Gap 2:

```text
chosen gap: 2 Angle: 109.1 deg Width: 47.4 cm
```

This chosen movement angle is suspicious because Gap 2 spans `126-162 deg`, but the chosen angle printed as `109.1 deg`, which is outside that gap. That comes from `select_gap()` applying the clearance offset in the wrong direction for this case.

For a gap from `126` to `162`, the chosen movement angle should stay inside that range. A value near `109` points the robot left but not actually through the selected gap.

## Likely Issues To Fix Next

1. Avoid the extra scan before auto mode if possible.
   - Instead of scanning in `lab7_main.c` and then scanning again inside `nav_auto_drive()`, either pass the first scan results into auto mode or let auto mode own all scanning.

2. Fix `select_gap()` so `chosen_movement_angle` is always inside the selected gap.
   - For a gap `[start, end]`, the final chosen angle should be clamped to that range.
   - If applying a robot-width offset from an edge, offset inward from the closest edge, not outside the gap.

3. Move boundary detection into the driving loop.
   - `move_forward()` currently does not stop on cliff/tape detection.
   - A safer design is to add a movement helper for auto mode that checks cliff signals during each `oi_update()` and stops immediately.

4. Print cliff signal values when boundary detection triggers.
   - The current message says boundary detected, but not which sensor caused it.
   - Printing `cliffLeftSignal`, `cliffFrontLeftSignal`, `cliffFrontRightSignal`, and `cliffRightSignal` would make field debugging much faster.

5. Re-check the boundary thresholds against the actual field.
   - Current thresholds are `>= 2500` for tape and `<= 500` for off-floor.
   - If normal floor readings sometimes cross those values, the robot will falsely back up and turn around.

## Recommended Next Implementation

The next code change should focus on correctness before tuning:

- Clamp selected gap movement angle to `[gap_start_angle, gap_end_angle]`.
- Add a debug print showing selected gap start, end, chosen angle, and target angle.
- Add cliff-signal debug output when boundary detection fires.
- Add an auto movement path that stops immediately on boundary detection instead of waiting until after the full forward step.

After those changes, rerun the same field and compare:

- whether the chosen angle is inside the chosen gap
- whether the robot starts moving through the intended opening
- whether boundary detection is legitimate or a false positive


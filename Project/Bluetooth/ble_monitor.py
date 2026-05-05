"""
ble_monitor_mobile.py  –  Virtual multi-anchor BLE trilateration
  using a single mobile sensor that moves between known positions.

  The sensor (laptop BLE or Arduino) stops at each anchor position,
  collects RSSI for DWELL_TIME seconds, then signals it is ready to
  move to the next position.  Once all virtual anchors have collected
  enough samples the trilateration runs automatically.

  Positions are either pre-configured or entered via the setup dialog.

Dependencies:
  pip install pyserial bleak matplotlib numpy
"""

import serial
import serial.tools.list_ports
import threading
import asyncio
import time
import statistics
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.widgets as widgets
import numpy as np
from bleak import BleakScanner

# ── Config ────────────────────────────────────────────────────────────────────
BAUD            = 115200
WINDOW          = 200       # rolling RSSI window per virtual anchor
DWELL_TIME      = 5.0       # seconds to collect RSSI at each stop position
MIN_SAMPLES_POS = 200       # minimum samples needed before a position is "ready"
AUTO_ADVANCE    = True      # True = advance automatically after DWELL_TIME
                            # False = wait for user to press "Next Position" button

# ── Filtering config ──────────────────────────────────────────────────────────
ZSCORE_THRESH   = 2.0
IQR_FENCE       = 1.5
EWM_ALPHA       = 0.1    # low since target is stationary
MIN_SAMPLES     = 5

# ── Trilateration config ──────────────────────────────────────────────────────
TX_POWER_1M = -62.0
PATH_LOSS   = 2.5
NUM_ANCHORS = 4       # number of virtual anchor positions to visit

# Default stop positions for the mobile sensor
DEFAULT_ANCHOR_POS = {  # tight ring r=80 centered on expected target area
    0: (-100.0,   0.0),
    1: (100.0,   0.0),
    2: ( 50.0,  87.0),
    3: ( 50.0, -87.0),
}
# ─────────────────────────────────────────────────────────────────────────────

TX_COLORS = {0: "blue", 1: "green", 2: "orange", 3: "purple"}
TX_LABELS = {0: "Stop 0", 1: "Stop 1", 2: "Stop 2", 3: "Stop 3"}

BLE_TX_PREFIX  = "BLE_TX_"
MFR_TYPE_TAG   = 0xA1
MFR_COMPANY_ID = 65535

# ── State shared across threads ───────────────────────────────────────────────
rssi_buffers    = {i: deque(maxlen=WINDOW) for i in range(NUM_ANCHORS)}
anchor_pos      = dict(DEFAULT_ANCHOR_POS)
ema_dist        = {i: None for i in range(NUM_ANCHORS)}
data_lock       = threading.Lock()

# Mobile sensor state
current_stop    = 0          # which virtual anchor position is active (0..NUM_ANCHORS-1)
stop_start_time = time.monotonic()
stop_ready      = {i: False for i in range(NUM_ANCHORS)}  # True when enough samples collected
collecting      = True       # False after all stops are done
advance_event   = threading.Event()   # set by button or timer to move to next stop
state_lock      = threading.Lock()


# ── Port discovery ────────────────────────────────────────────────────────────
def find_arduino_port() -> str | None:
    kws   = ["arduino", "nano", "usb serial", "ch340", "cp210", "ftdi"]
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(kw in (p.description or "").lower() for kw in kws):
            return p.device
    return ports[0].device if ports else None


# ── Arduino serial parser ─────────────────────────────────────────────────────
def parse_arduino_line(line: str) -> dict | None:
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) != 6:
        return None
    try:
        return {"rssi": int(parts[4])}
    except ValueError:
        return None


# ── Laptop BLE reader ─────────────────────────────────────────────────────────
def _laptop_ble_callback(device, advertisement_data):
    name = advertisement_data.local_name or ""
    if not name.startswith(BLE_TX_PREFIX):
        return
    rssi = advertisement_data.rssi
    if rssi is None:
        return
    with state_lock:
        stop = current_stop
        done = not collecting
    if done or stop < 0:   # stop < 0 means in transit
        return
    with data_lock:
        rssi_buffers[stop].append(rssi)

async def _laptop_ble_scan_loop():
    async with BleakScanner(detection_callback=_laptop_ble_callback):
        print("[Sensor] Laptop BLE scanner running…")
        while True:
            await asyncio.sleep(0.1)

def laptop_ble_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_laptop_ble_scan_loop())
    except Exception as e:
        print(f"[Sensor] BLE error: {e}")


def arduino_reader_thread(port: str):
    print(f"[Sensor] Connecting to {port} …")
    while True:
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"[Sensor] Serial open on {port}")
                while True:
                    raw = ser.readline().decode("utf-8", errors="ignore")
                    if not raw.strip():
                        continue
                    row = parse_arduino_line(raw)
                    if not row:
                        continue
                    with state_lock:
                        stop = current_stop
                        done = not collecting
                    if done or stop < 0:
                        continue
                    with data_lock:
                        rssi_buffers[stop].append(row["rssi"])
        except Exception as e:
            print(f"[Sensor] Serial error ({e}), retrying…")
            time.sleep(2)


# ── Stop manager — advances current_stop after dwell time ────────────────────
TRANSIT_TIME = 2.0   # seconds to allow bot to travel between stops

def stop_manager_thread():
    global current_stop, collecting

    for stop_idx in range(NUM_ANCHORS):
        # ── Transit pause before collecting (skip on first stop) ──────────────
        if stop_idx > 0:
            print(f"[MOVE] Travelling to position {stop_idx} — pausing {TRANSIT_TIME}s…")
            with state_lock:
                current_stop = -1
            time.sleep(TRANSIT_TIME)

        with state_lock:
            current_stop = stop_idx
        stop_start = time.monotonic()
        print(f"\n[MOVE] Arrived at position {stop_idx}  {anchor_pos[stop_idx]}")
        print(f"[MOVE] Collecting for {DWELL_TIME}s …")

        if AUTO_ADVANCE:
            deadline = stop_start + DWELL_TIME
            while time.monotonic() < deadline:
                time.sleep(0.1)
        else:
            advance_event.clear()
            advance_event.wait()

        # Mark stop as ready and print sample count summary
        with data_lock:
            n = len(rssi_buffers[stop_idx])
            buf = list(rssi_buffers[stop_idx])
        with state_lock:
            stop_ready[stop_idx] = (n >= MIN_SAMPLES_POS)

        avg = sum(buf) / len(buf) if buf else 0
        print(f"[STOP {stop_idx}] Samples collected: {n}  |  "
              f"Avg RSSI: {avg:.1f} dBm  |  "
              f"Ready: {stop_ready[stop_idx]}")

    with state_lock:
        collecting = False

    # Print final summary and trilateration result
    print("\n=== Collection Complete ===")
    circles = []
    for i in range(NUM_ANCHORS):
        with data_lock:
            buf = list(rssi_buffers[i])
        n = len(buf)
        if n == 0:
            print(f"  Stop {i}: 0 samples — skipped")
            continue
        filt = filter_rssi(buf)
        med  = statistics.median(filt)
        dist = rssi_to_distance(med)
        circles.append((anchor_pos[i], dist))
        print(f"  Stop {i} {anchor_pos[i]}: {n} samples  "
              f"median RSSI={med:.1f} dBm  dist={dist:.2f} units")

    if len(circles) >= 3:
        result = trilaterate(circles)
        if result:
            print(f"\n>>> Estimated TX position: ({result[0]:.2f}, {result[1]:.2f})")
        else:
            print("\n>>> Trilateration failed — degenerate geometry")
    else:
        print(f"\n>>> Not enough stops with data ({len(circles)}/3 minimum)")


# ── RSSI filtering ────────────────────────────────────────────────────────────
def filter_rssi(samples: list) -> list:
    if len(samples) < MIN_SAMPLES:
        return samples
    arr = np.array(samples, dtype=float)
    q1, q3 = np.percentile(arr, [25, 75])
    iqr = q3 - q1
    lo_iqr, hi_iqr = q1 - IQR_FENCE * iqr, q3 + IQR_FENCE * iqr
    mu, sigma = arr.mean(), arr.std()
    if sigma > 0:
        lo_z = mu - ZSCORE_THRESH * sigma
        hi_z = mu + ZSCORE_THRESH * sigma
    else:
        lo_z, hi_z = lo_iqr, hi_iqr
    lo, hi = max(lo_iqr, lo_z), min(hi_iqr, hi_z)
    clean = arr[(arr >= lo) & (arr <= hi)].tolist()
    return clean if clean else samples


def rssi_to_distance(rssi_dbm: float) -> float:
    return 10.0 ** ((TX_POWER_1M - rssi_dbm) / (10.0 * PATH_LOSS))


def smoothed_distance(anchor_id: int, raw_dist: float) -> float:
    prev = ema_dist[anchor_id]
    if prev is None:
        ema_dist[anchor_id] = raw_dist
    else:
        ema_dist[anchor_id] = EWM_ALPHA * raw_dist + (1 - EWM_ALPHA) * prev
    return ema_dist[anchor_id]


# ── Trilateration ─────────────────────────────────────────────────────────────
def trilaterate(circles: list):
    if len(circles) < 3:
        return None
    ref_pos, ref_r = circles[0]
    ATA = [[0.0, 0.0], [0.0, 0.0]]
    ATb = [0.0, 0.0]
    for pos, r in circles[1:]:
        a0 = 2.0 * (pos[0] - ref_pos[0])
        a1 = 2.0 * (pos[1] - ref_pos[1])
        bi = (ref_r**2 - r**2
              - ref_pos[0]**2 + pos[0]**2
              - ref_pos[1]**2 + pos[1]**2)
        ATA[0][0] += a0*a0;  ATA[0][1] += a0*a1
        ATA[1][0] += a1*a0;  ATA[1][1] += a1*a1
        ATb[0]    += a0*bi;  ATb[1]    += a1*bi
    det = ATA[0][0]*ATA[1][1] - ATA[0][1]*ATA[1][0]
    if abs(det) < 1e-12:
        return None
    x = (ATA[1][1]*ATb[0] - ATA[0][1]*ATb[1]) / det
    y = (ATA[0][0]*ATb[1] - ATA[1][0]*ATb[0]) / det
    return (x, y)


# ── Setup dialog ──────────────────────────────────────────────────────────────
def get_anchor_positions() -> dict:
    pos = dict(DEFAULT_ANCHOR_POS)
    confirmed = [False]

    n_fields   = NUM_ANCHORS * 2          # one row per X and one per Y
    fig_h      = max(4, n_fields * 0.7)   # scale height with number of fields
    row_h      = 0.06
    row_gap    = 0.01
    btn_h      = 0.07
    top        = 0.88                     # start below the suptitle
    total_rows = n_fields
    used_h     = total_rows * (row_h + row_gap) + btn_h + 0.05
    scale      = min(1.0, (1.0 - 0.08) / used_h)  # shrink if needed

    fig_cfg, ax_cfg = plt.subplots(figsize=(5, fig_h))
    fig_cfg.canvas.manager.set_window_title("Stop Position Setup")
    fig_cfg.suptitle("Enter sensor stop positions, then click Confirm",
                     fontsize=10, y=0.98)
    ax_cfg.axis("off")

    box_objs = {}
    field_defs = [(i, c, f"Stop {i} {'X' if c=='x' else 'Y'}:")
                  for i in range(NUM_ANCHORS) for c in ("x", "y")]

    for idx, (anchor_id, coord, label) in enumerate(field_defs):
        bottom = top - idx * (row_h + row_gap) * scale
        ax_lbl = fig_cfg.add_axes([0.05, bottom, 0.44, row_h * scale])
        ax_lbl.axis("off")
        ax_lbl.text(1.0, 0.5, label, ha="right", va="center",
                    fontsize=9, transform=ax_lbl.transAxes)
        ax_box = fig_cfg.add_axes([0.51, bottom, 0.42, row_h * scale])
        default_val = str(DEFAULT_ANCHOR_POS[anchor_id][0 if coord == "x" else 1])
        tb = widgets.TextBox(ax_box, "", initial=default_val)
        box_objs[(anchor_id, coord)] = tb

    btn_bottom = top - total_rows * (row_h + row_gap) * scale - 0.02
    ax_btn = fig_cfg.add_axes([0.35, max(0.02, btn_bottom), 0.30, btn_h * scale])
    btn = widgets.Button(ax_btn, "Confirm", color="#c8f0c8", hovercolor="#90d890")

    def on_confirm(_):
        try:
            new_pos = {}
            for i in range(NUM_ANCHORS):
                x = float(box_objs[(i, "x")].text)
                y = float(box_objs[(i, "y")].text)
                new_pos[i] = (x, y)
            pos.update(new_pos)
            confirmed[0] = True
            plt.close(fig_cfg)
        except ValueError:
            ax_cfg.set_title("Invalid input – numbers only", color="red", pad=12)
            fig_cfg.canvas.draw()

    btn.on_clicked(on_confirm)
    plt.show(block=True)
    return pos


# ── Main plot ─────────────────────────────────────────────────────────────────
def run_plot():
    fig = plt.figure(figsize=(15, 10))
    fig.canvas.manager.set_window_title("BLE Trilateration — Mobile Sensor")

    gs = gridspec.GridSpec(3, 2, figure=fig,
                           height_ratios=[0.12, 2, 1],
                           hspace=0.45, wspace=0.3)

    ax_pos   = fig.add_subplot(gs[0, :])
    ax_map   = fig.add_subplot(gs[1, 0])
    ax_rssi  = fig.add_subplot(gs[1, 1])
    ax_table = fig.add_subplot(gs[2, :])

    # ── Position banner ───────────────────────────────────────────────────────
    ax_pos.axis("off")
    pos_banner = ax_pos.text(
        0.5, 0.5, "Move sensor to Stop 0…",
        ha="center", va="center", fontsize=13, fontweight="bold",
        transform=ax_pos.transAxes,
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#ffffcc", edgecolor="#bbbb00")
    )

    # ── Next button (shown only when AUTO_ADVANCE=False) ──────────────────────
    if not AUTO_ADVANCE:
        ax_btn = fig.add_axes([0.44, 0.001, 0.12, 0.04])
        next_btn = widgets.Button(ax_btn, "Next Position ▶",
                                  color="#c8e0ff", hovercolor="#80b0ff")
        def on_next(_):
            advance_event.set()
        next_btn.on_clicked(on_next)

    # ── Map ───────────────────────────────────────────────────────────────────
    ax_map.set_title("TX Target Localization")
    ax_map.set_xlabel("X"); ax_map.set_ylabel("Y")
    ax_map.grid(True, linestyle="--", alpha=0.6)
    ax_map.set_aspect("equal", adjustable="datalim")

    anchor_markers = {}
    anchor_labels_txt = {}
    for idx in range(NUM_ANCHORS):
        x, y = anchor_pos[idx]
        m, = ax_map.plot(x, y, "s", color=TX_COLORS[idx], markersize=10,
                         zorder=3, alpha=0.35)
        t = ax_map.text(x + 2, y + 2, TX_LABELS[idx], fontsize=8,
                        color=TX_COLORS[idx], fontweight="bold", alpha=0.35)
        anchor_markers[idx]    = m
        anchor_labels_txt[idx] = t

    range_circles = {}
    for idx in range(NUM_ANCHORS):
        circle = plt.Circle(anchor_pos[idx], 0, color=TX_COLORS[idx],
                            fill=False, linestyle="--", alpha=0.3)
        ax_map.add_patch(circle)
        range_circles[idx] = circle

    target_dot, = ax_map.plot([], [], "r*", markersize=16,
                               label="TX Position", zorder=5)
    sensor_dot, = ax_map.plot([], [], "kD", markersize=10,
                               label="Sensor (current stop)", zorder=4)
    ax_map.legend(loc="upper right", fontsize=8)

    def _refresh_map_limits():
        xs  = [p[0] for p in anchor_pos.values()]
        ys  = [p[1] for p in anchor_pos.values()]
        pad = max(max(xs) - min(xs), max(ys) - min(ys)) * 0.5 + 20
        ax_map.set_xlim(min(xs) - pad, max(xs) + pad)
        ax_map.set_ylim(min(ys) - pad, max(ys) + pad)

    _refresh_map_limits()

    # ── RSSI plot ─────────────────────────────────────────────────────────────
    ax_rssi.set_ylim(-100, -10)
    ax_rssi.set_xlim(0, WINDOW)
    ax_rssi.set_title("Live RSSI — current stop (filtered)")
    ax_rssi.set_xlabel("Sample"); ax_rssi.set_ylabel("RSSI (dBm)")
    rssi_line, = ax_rssi.plot([], [], color="black", linewidth=1.4)

    # ── Table ─────────────────────────────────────────────────────────────────
    ax_table.axis("off")
    col_labels = ["Stop", "Position", "Samples", "Avg RSSI (dBm)", "Distance (EMA)", "Status"]
    blank = [["—"] * len(col_labels)] * NUM_ANCHORS
    tbl = ax_table.table(cellText=blank, colLabels=col_labels,
                         loc="center", cellLoc="center")
    tbl.scale(1, 1.5)
    for idx in range(NUM_ANCHORS):
        tbl[idx+1, 0].get_text().set_text(f"Stop {idx}")
        tbl[idx+1, 1].get_text().set_text(str(anchor_pos[idx]))

    last_print = [0.0]
    PRINT_INTERVAL = 0.5

    def update(_frame):
        with state_lock:
            stop     = current_stop
            ready    = dict(stop_ready)
            all_done = not collecting

        with data_lock:
            bufs = {i: list(rssi_buffers[i]) for i in range(NUM_ANCHORS)}

        # show sensor position on map — skip during transit (stop = -1)
        if stop >= 0:
            sx, sy = anchor_pos[stop]
            sensor_dot.set_data([sx], [sy])
        else:
            sensor_dot.set_data([], [])

        # update table and range circles
        avg_data = {}
        for i in range(NUM_ANCHORS):
            buf = bufs[i]
            # make stop marker fully opaque once ready
            alpha = 1.0 if ready[i] else (0.8 if i == stop else 0.3)
            anchor_markers[i].set_alpha(alpha)
            anchor_labels_txt[i].set_alpha(alpha)

            if not buf:
                tbl[i+1, 2].get_text().set_text("0")
                tbl[i+1, 3].get_text().set_text("—")
                tbl[i+1, 4].get_text().set_text("—")
                tbl[i+1, 5].get_text().set_text("Waiting" if i == stop else "Not yet")
                continue

            filtered = filter_rssi(buf)
            filt_avg = statistics.median(filtered)
            raw_dist = rssi_to_distance(filt_avg)
            dist     = smoothed_distance(i, raw_dist)

            avg_data[i] = (filt_avg, dist)
            range_circles[i].set_radius(dist if ready[i] else 0)
            range_circles[i].center = anchor_pos[i]

            status = "✓ Ready" if ready[i] else (
                f"Collecting… ({len(buf)}/{MIN_SAMPLES_POS})" if i == stop else "Queued")

            tbl[i+1, 2].get_text().set_text(str(len(buf)))
            tbl[i+1, 3].get_text().set_text(f"{filt_avg:.1f}")
            tbl[i+1, 4].get_text().set_text(f"{dist:.2f} m")
            tbl[i+1, 5].get_text().set_text(status)

        # live RSSI for current stop
        cur_buf = filter_rssi(bufs[stop]) if stop >= 0 and bufs[stop] else []
        rssi_line.set_data(range(len(cur_buf)), cur_buf)

        # progress / instruction banner
        if not all_done:
            elapsed = time.monotonic() - stop_start_time
            if stop < 0:
                pos_banner.set_text(f"Travelling to next stop… ({TRANSIT_TIME}s transit — not collecting)")
            elif AUTO_ADVANCE:
                remaining = max(0.0, DWELL_TIME - elapsed)
                pos_banner.set_text(
                    f"Move sensor to Stop {stop}  {anchor_pos[stop]} "
                    f"— collecting ({remaining:.1f}s left, {len(bufs[stop])} samples)")
            else:
                pos_banner.set_text(
                    f"Sensor at Stop {stop}  {anchor_pos[stop]} "
                    f"— {len(bufs[stop])} samples collected  |  Press 'Next Position' when ready")
        else:
            # all stops done — run trilateration using only ready stops
            ready_ids = [i for i in range(NUM_ANCHORS) if ready[i] and i in avg_data]
            if len(ready_ids) >= 3:
                circles = [(anchor_pos[i], avg_data[i][1]) for i in ready_ids]
                pos = trilaterate(circles)
                if pos:
                    target_dot.set_data([pos[0]], [pos[1]])
                    txt = f"TX Position:  X = {pos[0]:.2f},  Y = {pos[1]:.2f}"
                    pos_banner.set_text(txt)
                    now = time.monotonic()
                    if now - last_print[0] >= PRINT_INTERVAL:
                        print(f"[POS] X={pos[0]:8.2f}  Y={pos[1]:8.2f}")
                        last_print[0] = now
                else:
                    pos_banner.set_text("TX Position: geometry singular — check stop layout")
                    target_dot.set_data([], [])
            else:
                pos_banner.set_text(f"Only {len(ready_ids)} stops ready — need 3")
                target_dot.set_data([], [])

        return ([target_dot, sensor_dot, pos_banner, rssi_line]
                + list(range_circles.values()))

    ani = animation.FuncAnimation(fig, update, interval=150,
                                  blit=False, cache_frame_data=False)
    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("[SETUP] Configure sensor stop positions…")
    new_pos = get_anchor_positions()
    anchor_pos.update(new_pos)
    for sid, (x, y) in anchor_pos.items():
        print(f"  Stop {sid}: ({x}, {y})")

    port = find_arduino_port()
    if not port:
        print("[ERROR] No serial port found. Plug in the RX Arduino and retry.")
        exit(1)

    print(f"[SETUP] Using Arduino on {port}")
    threading.Thread(target=arduino_reader_thread,
                     args=(port,), daemon=True).start()

    threading.Thread(target=stop_manager_thread, daemon=True).start()

    run_plot()

    # Main plot
    run_plot()

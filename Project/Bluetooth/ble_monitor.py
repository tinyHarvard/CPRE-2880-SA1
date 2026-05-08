"""
ble_monitor_mobile.py  –  Virtual multi-anchor BLE trilateration
  using a single mobile sensor that moves between known positions.

Dependencies:
  pip install pyserial matplotlib numpy
"""

import serial
import serial.tools.list_ports
import threading
import time
import statistics
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.widgets as widgets
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
BAUD            = 115200
WINDOW          = 100       # rolling RSSI window (~5 packets/sec)
DWELL_TIME      = 10.0      # seconds to collect at each stop
MIN_SAMPLES_POS = 50       # minimum samples before a stop is "ready"
AUTO_ADVANCE    = False     # True = auto-advance after DWELL_TIME
                            # False = wait for "Next Position" button
TRANSIT_TIME    = 10.0       # seconds between stops for bot to travel

# ── Filtering config ──────────────────────────────────────────────────────────
ZSCORE_THRESH   = 2.5
IQR_FENCE       = 2.0
EWM_ALPHA       = 0.3
MIN_SAMPLES     = 100

# ── Trilateration config ──────────────────────────────────────────────────────
TX_POWER_1M = -75.0 # -68.5 -70.5
PATH_LOSS   = 2.05
NUM_ANCHORS = 6

DEFAULT_ANCHOR_POS = {
    # 0: ( -100,  -100),
    # 1: ( 214,  -100),
    # 2: ( 527,  -100),
    # 3: ( 527,   344),
    # 4: ( 214,   344),
    # 5: (-100,   344),

    0: ( -100,  -100),
    1: ( -100,  214),
    2: ( -100,  527),
    3: ( 344,   527),
    4: ( 344,   214),
    5: ( 344,   -100),
}

TX_COLORS = {0: "blue", 1: "green", 2: "orange", 3: "purple", 4: "red", 5: "brown"}
TX_LABELS = {i: f"Stop {i}" for i in range(NUM_ANCHORS)}

# ── Shared state ──────────────────────────────────────────────────────────────
rssi_buffers    = {i: deque(maxlen=WINDOW) for i in range(NUM_ANCHORS)}
anchor_pos      = dict(DEFAULT_ANCHOR_POS)
ema_dist        = {i: None for i in range(NUM_ANCHORS)}
locked_dist     = {i: None for i in range(NUM_ANCHORS)}
data_lock       = threading.Lock()

current_stop    = 0
transit_to      = -1
stop_start_time = [time.monotonic()]
stop_ready      = {i: False for i in range(NUM_ANCHORS)}
collecting      = True
advance_event   = threading.Event()
start_event     = threading.Event()
state_lock      = threading.Lock()


# ── Port discovery ────────────────────────────────────────────────────────────
def find_arduino_port():
    kws   = ["arduino", "nano", "usb serial", "ch340", "cp210", "ftdi"]
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(kw in (p.description or "").lower() for kw in kws):
            return p.device
    return ports[0].device if ports else None


# ── Serial parser ─────────────────────────────────────────────────────────────
def parse_arduino_line(line: str):
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("tx_id") or line.startswith("ERR"):
        return None
    parts = line.split(",")
    if len(parts) != 6:
        return None
    try:
        return {"rssi": int(parts[4])}
    except ValueError:
        return None


# ── Arduino reader ────────────────────────────────────────────────────────────
def arduino_reader_thread(port: str):
    print(f"[Sensor] Connecting to {port} …")
    while True:
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"[Sensor] Serial open on {port}")
                while True:
                    raw = ser.readline().decode("utf-8", errors="ignore")
                    row = parse_arduino_line(raw)
                    if row is None:
                        continue
                    with state_lock:
                        stop = current_stop
                        done = not collecting
                    if done or stop < 0:
                        continue
                    with data_lock:
                        rssi_buffers[stop].append(row["rssi"])
        except serial.SerialException as e:
            print(f"[Sensor] Serial error ({e}), retrying in 2s…")
            time.sleep(2)
        except Exception as e:
            print(f"[Sensor] Unexpected error ({e}), retrying in 2s…")
            time.sleep(2)


# ── Stop manager ──────────────────────────────────────────────────────────────
def stop_manager_thread():
    global current_stop, collecting, transit_to

    print("[SETUP] Waiting for Start button…")
    start_event.wait()
    print("[SETUP] Started.")

    # reset all state for fresh run
    with data_lock:
        for i in range(NUM_ANCHORS):
            ema_dist[i]    = None
            locked_dist[i] = None
            rssi_buffers[i].clear()

    for stop_idx in range(NUM_ANCHORS):
        if stop_idx > 0:
            print(f"[MOVE] Travelling to position {stop_idx} — pausing {TRANSIT_TIME}s…")
            with state_lock:
                current_stop = -1
                transit_to   = stop_idx
                stop_start_time[0] = time.monotonic()
            time.sleep(TRANSIT_TIME)

        with state_lock:
            current_stop = stop_idx
            stop_start_time[0] = time.monotonic()

        # clear buffer and reset EMA — start completely fresh at this stop
        with data_lock:
            rssi_buffers[stop_idx].clear()
            ema_dist[stop_idx] = None

        print(f"\n[MOVE] Arrived at position {stop_idx}  {anchor_pos[stop_idx]}")
        print(f"[MOVE] Collecting for {DWELL_TIME}s …")

        if AUTO_ADVANCE:
            deadline = stop_start_time[0] + DWELL_TIME
            while time.monotonic() < deadline:
                time.sleep(0.1)
        else:
            advance_event.clear()
            advance_event.wait()

        with data_lock:
            n   = len(rssi_buffers[stop_idx])
            buf = list(rssi_buffers[stop_idx])
        with state_lock:
            stop_ready[stop_idx] = (n >= MIN_SAMPLES_POS)

        # lock distance using mean for sub-dBm precision
        MAX_PLAUSIBLE_DIST = 300.0
        if buf:
            filt = filter_rssi(buf)
            med  = statistics.median(filt)
            d    = rssi_to_distance(med)
            if d <= MAX_PLAUSIBLE_DIST:
                with data_lock:
                    locked_dist[stop_idx] = d
                    print(f"  locked_dist[{stop_idx}] = {d:.2f}  (n={n}  median_rssi={med:.2f})")
            else:
                print(f"  [WARN] Stop {stop_idx} dist={d:.2f} exceeds max — discarded")

        print(f"[STOP {stop_idx}] Samples: {n}  |  Ready: {stop_ready[stop_idx]}")

    with state_lock:
        collecting = False

    # Final summary using locked distances
    print("\n=== Collection Complete ===")
    circles = []
    for i in range(NUM_ANCHORS):
        with data_lock:
            d = locked_dist[i]
            n = len(rssi_buffers[i])
        if d is None:
            print(f"  Stop {i}: no locked distance — skipped")
            continue
        circles.append((anchor_pos[i], d))
        print(f"  Stop {i} {anchor_pos[i]}: {n} samples  dist={d:.2f} units")

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
        lo_z, hi_z = mu - ZSCORE_THRESH*sigma, mu + ZSCORE_THRESH*sigma
    else:
        lo_z, hi_z = lo_iqr, hi_iqr
    lo, hi = max(lo_iqr, lo_z), min(hi_iqr, hi_z)
    clean = arr[(arr >= lo) & (arr <= hi)].tolist()
    return clean if clean else samples

def rssi_to_distance(rssi_dbm: float) -> float:
    if rssi_dbm >= -73.0:       # 0 - 2m:   n = 1.00
        n = 2.56
    elif rssi_dbm >= -78.0:     # 2m - 3m:  n = 3.98
        n = 1.83
    else:                       # > 3m:     n = 2.36 (average)
        n = 0.8
    return 10.0 ** ((TX_POWER_1M - rssi_dbm) / (10.0 * n)) * 100


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
    n_fields = NUM_ANCHORS * 2
    fig_h    = max(5, n_fields * 0.65)
    row_h    = 0.06; row_gap = 0.01; btn_h = 0.07
    top      = 0.88
    scale    = min(1.0, (top - 0.08) / (n_fields * (row_h + row_gap) + btn_h))

    fig_cfg, ax_cfg = plt.subplots(figsize=(5, fig_h))
    fig_cfg.canvas.manager.set_window_title("Stop Position Setup")
    fig_cfg.suptitle("Enter sensor stop positions, then click Confirm",
                     fontsize=10, y=0.98)
    ax_cfg.axis("off")

    box_objs   = {}
    field_defs = [(i, c, f"Stop {i} {'X' if c=='x' else 'Y'}:")
                  for i in range(NUM_ANCHORS) for c in ("x", "y")]

    for idx, (anchor_id, coord, label) in enumerate(field_defs):
        bottom  = top - idx * (row_h + row_gap) * scale
        ax_lbl  = fig_cfg.add_axes([0.05, bottom, 0.44, row_h * scale])
        ax_lbl.axis("off")
        ax_lbl.text(1.0, 0.5, label, ha="right", va="center",
                    fontsize=9, transform=ax_lbl.transAxes)
        ax_box  = fig_cfg.add_axes([0.51, bottom, 0.42, row_h * scale])
        default = str(DEFAULT_ANCHOR_POS[anchor_id][0 if coord == "x" else 1])
        box_objs[(anchor_id, coord)] = widgets.TextBox(ax_box, "", initial=default)

    btn_bottom = top - n_fields * (row_h + row_gap) * scale - 0.02
    ax_btn = fig_cfg.add_axes([0.35, max(0.02, btn_bottom), 0.30, btn_h * scale])
    btn    = widgets.Button(ax_btn, "Confirm", color="#c8f0c8", hovercolor="#90d890")

    def on_confirm(_):
        try:
            new_pos = {i: (float(box_objs[(i,"x")].text),
                           float(box_objs[(i,"y")].text))
                       for i in range(NUM_ANCHORS)}
            pos.update(new_pos)
            plt.close(fig_cfg)
        except ValueError:
            fig_cfg.suptitle("Invalid input – numbers only", color="red", fontsize=10)
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

    # ── Banner ────────────────────────────────────────────────────────────────
    ax_pos.axis("off")
    pos_banner = ax_pos.text(
        0.5, 0.5, "Press Start to begin",
        ha="center", va="center", fontsize=13, fontweight="bold",
        transform=ax_pos.transAxes,
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#ffffcc", edgecolor="#bbbb00")
    )

    # ── Start button ──────────────────────────────────────────────────────────
    ax_start  = fig.add_axes([0.44, 0.046, 0.12, 0.04])
    start_btn = widgets.Button(ax_start, "Start",
                               color="#c8f0c8", hovercolor="#90d890")
    started = [False]
    def on_start(_):
        if not started[0]:
            started[0] = True
            start_btn.label.set_text("Running...")
            start_btn.color      = "#dddddd"
            start_btn.hovercolor = "#dddddd"
            start_event.set()
    start_btn.on_clicked(on_start)

    # ── Next button ───────────────────────────────────────────────────────────
    if not AUTO_ADVANCE:
        ax_btn   = fig.add_axes([0.44, 0.001, 0.12, 0.04])
        next_btn = widgets.Button(ax_btn, "Next Position",
                                  color="#c8e0ff", hovercolor="#80b0ff")
        next_btn.on_clicked(lambda _: advance_event.set())

    # ── Map ───────────────────────────────────────────────────────────────────
    ax_map.set_title("TX Target Localization")
    ax_map.set_xlabel("X"); ax_map.set_ylabel("Y")
    ax_map.grid(True, linestyle="--", alpha=0.6)

    anchor_markers = {}
    anchor_labels_txt = {}
    for idx in range(NUM_ANCHORS):
        x, y = anchor_pos[idx]
        m, = ax_map.plot(x, y, "s", color=TX_COLORS[idx],
                         markersize=10, zorder=3, alpha=0.35)
        t = ax_map.text(x + 2, y + 2, TX_LABELS[idx], fontsize=8,
                        color=TX_COLORS[idx], fontweight="bold", alpha=0.35)
        anchor_markers[idx]    = m
        anchor_labels_txt[idx] = t

    range_circles = {}
    for idx in range(NUM_ANCHORS):
        c = plt.Circle(anchor_pos[idx], 0, color=TX_COLORS[idx],
                       fill=False, linestyle="--", alpha=0.3)
        ax_map.add_patch(c)
        range_circles[idx] = c

    target_dot, = ax_map.plot([], [], "r*", markersize=16, label="TX Position", zorder=5)
    sensor_dot, = ax_map.plot([], [], "kD", markersize=10,
                               label="Sensor (current stop)", zorder=4)
    ax_map.legend(loc="upper right", fontsize=8)

    xs  = [p[0] for p in anchor_pos.values()]
    ys  = [p[1] for p in anchor_pos.values()]
    pad = max(max(xs)-min(xs), max(ys)-min(ys)) * 0.5 + 20
    ax_map.set_xlim(min(xs)-pad, max(xs)+pad)
    ax_map.set_ylim(min(ys)-pad, max(ys)+pad)

    # ── RSSI plot ─────────────────────────────────────────────────────────────
    ax_rssi.set_ylim(-100, -10)
    ax_rssi.set_xlim(0, WINDOW)
    ax_rssi.set_title("Live RSSI — current stop (filtered)")
    ax_rssi.set_xlabel("Sample"); ax_rssi.set_ylabel("RSSI (dBm)")
    rssi_line, = ax_rssi.plot([], [], color="black", linewidth=1.4)

    # ── Table ─────────────────────────────────────────────────────────────────
    ax_table.axis("off")
    col_labels = ["Stop", "Position", "Samples", "Median RSSI (dBm)", "Distance", "Status"]
    blank = [["—"] * len(col_labels) for _ in range(NUM_ANCHORS)]
    tbl   = ax_table.table(cellText=blank, colLabels=col_labels,
                            loc="center", cellLoc="center")
    tbl.scale(1, 1.5)
    for idx in range(NUM_ANCHORS):
        tbl[idx+1, 0].get_text().set_text(f"Stop {idx}")
        tbl[idx+1, 1].get_text().set_text(str(anchor_pos[idx]))

    printed_result = [False]

    def update(_frame):
        with state_lock:
            stop     = current_stop
            t_to     = transit_to
            ready    = dict(stop_ready)
            all_done = not collecting

        with data_lock:
            bufs   = {i: list(rssi_buffers[i]) for i in range(NUM_ANCHORS)}
            locked = dict(locked_dist)

        # sensor dot
        if stop >= 0:
            sx, sy = anchor_pos[stop]
            sensor_dot.set_data([sx], [sy])
        else:
            sensor_dot.set_data([], [])

        # table + range circles
        for i in range(NUM_ANCHORS):
            buf   = bufs[i]
            alpha = 1.0 if ready[i] else (0.8 if i == stop else 0.3)
            anchor_markers[i].set_alpha(alpha)
            anchor_labels_txt[i].set_alpha(alpha)

            if not buf:
                tbl[i+1, 2].get_text().set_text("0")
                tbl[i+1, 3].get_text().set_text("—")
                tbl[i+1, 4].get_text().set_text("—")
                tbl[i+1, 5].get_text().set_text("Waiting" if i == stop else "Not yet")
                range_circles[i].set_radius(0)
                continue

            filtered = filter_rssi(buf)
            med      = statistics.median(filtered)
            raw_dist = rssi_to_distance(med)
            dist     = smoothed_distance(i, raw_dist)

            # use locked distance for circle if ready
            circle_r = locked[i] if ready[i] and locked[i] else 0
            range_circles[i].set_radius(circle_r)
            range_circles[i].center = anchor_pos[i]

            status = "Ready" if ready[i] else (
                f"Collecting ({len(buf)}/{MIN_SAMPLES_POS})" if i == stop else "Queued")

            tbl[i+1, 2].get_text().set_text(str(len(buf)))
            tbl[i+1, 3].get_text().set_text(f"{med:.1f}")
            tbl[i+1, 4].get_text().set_text(f"{dist:.2f}" if not ready[i]
                                             else f"{locked[i]:.2f}" if locked[i] else "—")
            tbl[i+1, 5].get_text().set_text(status)

        # live RSSI line
        cur_buf = filter_rssi(bufs[stop]) if stop >= 0 and bufs[stop] else []
        rssi_line.set_data(range(len(cur_buf)), cur_buf)

        # banner
        if not all_done:
            elapsed = time.monotonic() - stop_start_time[0]
            if not start_event.is_set():
                pos_banner.set_text("Press Start to begin collection")
            elif stop < 0:
                remaining = max(0.0, TRANSIT_TIME - elapsed)
                dest_pos  = anchor_pos[t_to] if t_to >= 0 else "?"
                pos_banner.set_text(
                    f"In transit -> Stop {t_to}  {dest_pos}  ({remaining:.1f}s remaining)")
            elif AUTO_ADVANCE:
                remaining = max(0.0, DWELL_TIME - elapsed)
                pos_banner.set_text(
                    f"Stop {stop} / {NUM_ANCHORS-1}  —  {anchor_pos[stop]}"
                    f"  |  Collecting ({remaining:.1f}s left, {len(bufs[stop])} samples)")
            else:
                pos_banner.set_text(
                    f"Stop {stop} / {NUM_ANCHORS-1}  —  {anchor_pos[stop]}"
                    f"  |  {len(bufs[stop])} samples  |  Press 'Next Position' when ready")
        else:
            ready_ids = [i for i in range(NUM_ANCHORS)
                         if ready[i] and locked[i] is not None]
            if len(ready_ids) >= 3:
                circles = [(anchor_pos[i], locked[i]) for i in ready_ids]
                if not printed_result[0]:
                    print("\n=== Trilateration Input ===")
                    for i in ready_ids:
                        print(f"  Stop {i} pos={anchor_pos[i]}  dist={locked[i]:.2f}")
                pos = trilaterate(circles)
                if pos:
                    target_dot.set_data([pos[0]], [pos[1]])
                    pos_banner.set_text(
                        f"TX Position:  X = {pos[0]:.2f},  Y = {pos[1]:.2f}")
                    if not printed_result[0]:
                        print(f"\n>>> Estimated TX position: ({pos[0]:.2f}, {pos[1]:.2f})")
                        printed_result[0] = True
                else:
                    pos_banner.set_text("Trilateration failed — degenerate geometry")
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

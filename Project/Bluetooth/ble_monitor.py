import serial
import serial.tools.list_ports
import threading
import time
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── Config ────────────────────────────────────────────────────────────────────
PORT   = None    # set to e.g. "COM3" or leave None to auto-detect
BAUD   = 115200
WINDOW = 100     # rolling sample window per TX
# ─────────────────────────────────────────────────────────────────────────────

TX_COLORS = {0: "red", 1: "blue", 2: "green"}
TX_LABELS = {0: "TX_0", 1: "TX_1", 2: "TX_2"}

rssi_buffers = {i: deque(maxlen=WINDOW) for i in range(3)}
latest       = {i: {} for i in range(3)}
data_lock    = threading.Lock()


def find_port() -> str:
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if any(kw in desc for kw in ["arduino", "nano", "usb serial", "ch340", "cp210"]):
            print(f"Auto-detected: {p.device} ({p.description})")
            return p.device
    if ports:
        print(f"Using first available port: {ports[0].device}")
        return ports[0].device
    raise RuntimeError("No serial ports found. Is the RX board plugged in?")


def parse_line(line: str) -> dict | None:
    line = line.strip()
    if not line or line.startswith("#") or line.startswith("tx_id") or line.startswith("ERR"):
        return None
    parts = line.split(",")
    if len(parts) != 6:
        return None
    try:
        tx_id = int(parts[0])
        if tx_id not in (0, 1, 2):
            return None
        return {
            "tx_id":    tx_id,
            "seq":      int(parts[1]),
            "tx_ts":    int(parts[2]),
            "rx_ts":    int(parts[3]),
            "rssi":     int(parts[4]),
            "tx_power": int(parts[5]),
        }
    except ValueError:
        return None


def reader_thread(port: str):
    while True:
        try:
            with serial.Serial(port, BAUD, timeout=1) as ser:
                print(f"Connected to {port} at {BAUD} baud")
                while True:
                    raw = ser.readline().decode("utf-8", errors="ignore")
                    row = parse_line(raw)
                    if row is None:
                        continue

                    tx_id = row["tx_id"]
                    with data_lock:
                        rssi_buffers[tx_id].append(row["rssi"])
                        latest[tx_id] = row

                    print(
                        f"[{TX_LABELS[tx_id]}]  "
                        f"seq={row['seq']:5d}  "
                        f"rssi={row['rssi']:4d} dBm  "
                        f"tx_ts={row['tx_ts']:8d} ms  "
                        f"rx_ts={row['rx_ts']:8d} ms"
                    )

        except serial.SerialException as e:
            print(f"Serial error: {e} — retrying in 2s")
            time.sleep(2)


def run_plot(port: str):
    fig, (ax_main, ax_table) = plt.subplots(
        2, 1,
        figsize=(10, 6),
        gridspec_kw={"height_ratios": [3, 1]}
    )
    fig.suptitle("Live BLE RSSI — 3 TX beacons, 1 RX", fontsize=12)

    # ── Rolling RSSI plot ─────────────────────────────────────────────────────
    ax_main.set_ylim(-100, -30)
    ax_main.set_xlim(0, WINDOW)
    ax_main.set_xlabel("Sample")
    ax_main.set_ylabel("RSSI (dBm)")
    ax_main.grid(True, alpha=0.3)
    ax_main.axhline(-62, color="orange", linestyle="--",
                    linewidth=1, label="TX power ref (-62 dBm)")

    lines = {}
    for tx_id in range(3):
        lines[tx_id], = ax_main.plot(
            [], [],
            color=TX_COLORS[tx_id],
            linewidth=1.4,
            label=TX_LABELS[tx_id]
        )
    ax_main.legend(loc="upper right", fontsize=8)

    # ── Latest values table ───────────────────────────────────────────────────
    ax_table.axis("off")
    col_labels = ["Device", "Seq", "RSSI (dBm)", "TX ts (ms)", "RX ts (ms)"]
    table_data = [["—"] * 5 for _ in range(3)]
    tbl = ax_table.table(
        cellText=table_data,
        colLabels=col_labels,
        loc="center",
        cellLoc="center"
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1, 1.6)

    for tx_id in range(3):
        cell = tbl[tx_id + 1, 0]
        cell.set_facecolor(TX_COLORS[tx_id])
        cell.set_text_props(color="white", fontweight="bold")

    def update(_frame):
        with data_lock:
            buffers_snap = {i: list(rssi_buffers[i]) for i in range(3)}
            latest_snap  = {i: dict(latest[i])        for i in range(3)}

        for tx_id, buf in buffers_snap.items():
            if buf:
                lines[tx_id].set_data(range(len(buf)), buf)

        for tx_id in range(3):
            row = latest_snap[tx_id]
            r   = tx_id + 1
            if row:
                tbl[r, 0].get_text().set_text(TX_LABELS[tx_id])
                tbl[r, 1].get_text().set_text(str(row.get("seq",    "—")))
                tbl[r, 2].get_text().set_text(str(row.get("rssi",   "—")))
                tbl[r, 3].get_text().set_text(str(row.get("tx_ts",  "—")))
                tbl[r, 4].get_text().set_text(str(row.get("rx_ts",  "—")))
            else:
                for col in range(1, 5):
                    tbl[r, col].get_text().set_text("waiting...")

        return list(lines.values())

    t = threading.Thread(target=reader_thread, args=(port,), daemon=True)
    t.start()

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    port = PORT or find_port()
    run_plot(port)
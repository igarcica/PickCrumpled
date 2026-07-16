"""
Kinova Gen3 - F/T Wrench Logger & Plotter
Connects to the robot via BaseCyclic, records the estimated external wrench
at the tool while you run your action from the Kinova Web App, then saves:
  -> cartesian_ft.png   Static 2x3 plot (Fx Fy Fz | Tx Ty Tz)
  -> cartesian_ft.csv   Raw time-series (reuse for any other plot)
  -> cartesian_ft.mp4   Screen-capture video of the live plot (requires ffmpeg)

The wrench is the ESTIMATED external wrench computed by the robot from joint
torques. It is NOT a dedicated hardware F/T sensor reading -- it reflects the
robot's internal model. If you have an external F/T sensor connected to the
tool flange, read it through its own interface instead.

Requirements:
    pip install kortex_api matplotlib numpy
    ffmpeg on PATH for video (sudo apt install ffmpeg / brew install ffmpeg)

Usage:
    1. python kinova_ft_plot.py
    2. Trigger your action from the Kinova Web App.
    3. Ctrl+C when done.
"""

import time
import signal
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

# ─── Configuration ─────────────────────────────────────────────────────────────
ROBOT_IP     = "192.168.1.12"
USERNAME     = "admin"
PASSWORD     = "admin"

POLL_RATE_HZ = 20               # Samples per second (max ~40 Hz over TCP)
MAX_RECORD_S = 300              # Auto-stop safety cap (seconds)

SAVE_PLOT    = True
SAVE_CSV     = True
SAVE_VIDEO   = True

PLOT_PATH    = "cartesian_ft.png"
CSV_PATH     = "cartesian_ft.csv"
VIDEO_PATH   = "cartesian_ft.mp4"
VIDEO_FPS    = 10
# ───────────────────────────────────────────────────────────────────────────────


class WrenchSampler:
    """
    Background thread that polls BaseCyclic.RefreshFeedback() and records
    the estimated external wrench fields from BaseFeedback.
    """

    def __init__(self, cyclic_client: BaseCyclicClient):
        self._client  = cyclic_client
        self._lock    = threading.Lock()
        self._running = False
        maxlen = int(POLL_RATE_HZ * MAX_RECORD_S)
        self.times   = deque(maxlen=maxlen)
        self.fx_vals = deque(maxlen=maxlen)
        self.fy_vals = deque(maxlen=maxlen)
        self.fz_vals = deque(maxlen=maxlen)
        self.tx_vals = deque(maxlen=maxlen)
        self.ty_vals = deque(maxlen=maxlen)
        self.tz_vals = deque(maxlen=maxlen)
        self._t0     = None

    def start(self):
        self._running = True
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[recorder] Sampling F/T at {POLL_RATE_HZ} Hz. Run your action now.")

    def stop(self):
        self._running = False
        self._thread.join(timeout=2)

    def _loop(self):
        interval = 1.0 / POLL_RATE_HZ
        while self._running:
            t_start = time.time()
            try:
                fb      = self._client.RefreshFeedback()
                base    = fb.base          # BaseFeedback message
                elapsed = time.time() - self._t0
                with self._lock:
                    self.times.append(elapsed)
                    self.fx_vals.append(base.tool_external_wrench_force_x)
                    self.fy_vals.append(base.tool_external_wrench_force_y)
                    self.fz_vals.append(base.tool_external_wrench_force_z)
                    self.tx_vals.append(base.tool_external_wrench_torque_x)
                    self.ty_vals.append(base.tool_external_wrench_torque_y)
                    self.tz_vals.append(base.tool_external_wrench_torque_z)
            except Exception as e:
                print(f"[recorder] Warning: {e}")
            sleep_for = interval - (time.time() - t_start)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def snapshot(self):
        """Return thread-safe copies of all buffers."""
        with self._lock:
            return (
                list(self.times),
                list(self.fx_vals),
                list(self.fy_vals),
                list(self.fz_vals),
                list(self.tx_vals),
                list(self.ty_vals),
                list(self.tz_vals),
            )

    @property
    def sample_count(self):
        with self._lock:
            return len(self.times)


# ─── CSV export ────────────────────────────────────────────────────────────────

CSV_HEADER = "time_s,fx_N,fy_N,fz_N,tx_Nm,ty_Nm,tz_Nm"

def save_csv(snapshot, path: str = CSV_PATH):
    t, fx, fy, fz, tx, ty, tz = snapshot
    if not t:
        print("[csv] No data – skipping.")
        return
    data = np.column_stack([t, fx, fy, fz, tx, ty, tz])
    np.savetxt(path, data, delimiter=",", header=CSV_HEADER, comments="", fmt="%.6f")
    print(f"[csv] {len(t)} samples saved to: {path}")


# ─── Plotting ──────────────────────────────────────────────────────────────────

F_LABELS = ["Fx (N)", "Fy (N)", "Fz (N)"]
T_LABELS = ["Tx (N·m)", "Ty (N·m)", "Tz (N·m)"]
LABELS   = F_LABELS + T_LABELS
F_COLORS = ["#1f77b4", "#2ca02c", "#d62728"]   # blue / green / red  (forces)
T_COLORS = ["#9467bd", "#8c564b", "#e377c2"]   # purple / brown / pink (torques)
COLORS   = F_COLORS + T_COLORS


def build_live_figure():
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle(
        "Kinova Gen3 – Estimated External Wrench  (Ctrl+C to stop)",
        fontsize=13, fontweight="bold"
    )
    # Row labels
    fig.text(0.01, 0.72, "Forces", va="center", rotation="vertical",
             fontsize=10, fontweight="bold", color="#333333")
    fig.text(0.01, 0.27, "Torques", va="center", rotation="vertical",
             fontsize=10, fontweight="bold", color="#333333")

    fig.tight_layout(rect=[0.03, 0, 1, 0.95])
    plt.subplots_adjust(hspace=0.45, wspace=0.35)

    lines = []
    for ax, lbl, col in zip(axes.flat, LABELS, COLORS):
        (ln,) = ax.plot([], [], color=col, linewidth=1.5)
        ax.axhline(0, color="gray", linewidth=0.6, linestyle="--")
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel(lbl, fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, linestyle="--", alpha=0.4)
        lines.append(ln)

    return fig, axes, lines


def save_final_plot(snapshot, path: str = PLOT_PATH):
    t, fx, fy, fz, tx, ty, tz = snapshot
    if not t:
        print("[plot] No data – skipping.")
        return

    t    = np.array(t)
    data = [fx, fy, fz, tx, ty, tz]

    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle("Kinova Gen3 – Estimated External Wrench", fontsize=13, fontweight="bold")
    fig.text(0.01, 0.72, "Forces", va="center", rotation="vertical",
             fontsize=10, fontweight="bold", color="#333333")
    fig.text(0.01, 0.27, "Torques", va="center", rotation="vertical",
             fontsize=10, fontweight="bold", color="#333333")
    fig.tight_layout(rect=[0.03, 0, 1, 0.95])
    plt.subplots_adjust(hspace=0.45, wspace=0.35)

    for ax, vals, lbl, col in zip(axes.flat, data, LABELS, COLORS):
        ax.plot(t, vals, color=col, linewidth=1.5)
        ax.axhline(0, color="gray", linewidth=0.6, linestyle="--")
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel(lbl, fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.set_xlim(t[0], t[-1])

    plt.savefig(path, dpi=150, bbox_inches="tight")
    print(f"[plot] Saved to: {path}")
    plt.show()


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    # Connect
    transport = TCPTransport()
    transport.connect(ROBOT_IP, 10000)
    router = RouterClient(transport, lambda exc: print(f"[router] {exc}"))

    session_info = Session_pb2.CreateSessionInfo()
    session_info.username                      = USERNAME
    session_info.password                      = PASSWORD
    session_info.session_inactivity_timeout    = 60_000
    session_info.connection_inactivity_timeout = 2_000

    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)
    print(f"[connection] Connected to {ROBOT_IP}")

    cyclic_client = BaseCyclicClient(router)

    # Start recording
    sampler = WrenchSampler(cyclic_client)
    sampler.start()

    # Build live figure
    fig, axes, lines = build_live_figure()
    stop_event = threading.Event()

    def update_plot(_frame):
        t, fx, fy, fz, tx, ty, tz = sampler.snapshot()
        if not t:
            return lines
        data = [fx, fy, fz, tx, ty, tz]
        for ln, ax, vals in zip(lines, axes.flat, data):
            ln.set_data(t, vals)
            ax.relim()
            ax.autoscale_view()
        return lines

    ani = animation.FuncAnimation(
        fig, update_plot, interval=int(1000 / VIDEO_FPS),
        blit=False, cache_frame_data=False
    )

    # Optional video writer
    video_writer = None
    if SAVE_VIDEO:
        try:
            video_writer = animation.FFMpegWriter(fps=VIDEO_FPS, bitrate=1800)
            video_writer.setup(fig, VIDEO_PATH, dpi=120)
            print(f"[video] Recording to: {VIDEO_PATH}  (fps={VIDEO_FPS})")

            original_update = update_plot
            def update_and_grab(frame):
                result = original_update(frame)
                try:
                    video_writer.grab_frame()
                except Exception as e:
                    print(f"[video] Frame grab warning: {e}")
                return result

            ani.event_source.stop()
            ani = animation.FuncAnimation(
                fig, update_and_grab, interval=int(1000 / VIDEO_FPS),
                blit=False, cache_frame_data=False
            )
        except Exception as e:
            print(f"[video] Could not start writer (ffmpeg installed?): {e}")
            video_writer = None

    # Ctrl+C handler
    def _sigint_handler(sig, frame):
        print(f"\n[recorder] Stopping. Captured {sampler.sample_count} samples.")
        stop_event.set()
        sampler.stop()
        plt.close("all")

    signal.signal(signal.SIGINT, _sigint_handler)

    # Auto-stop after MAX_RECORD_S
    def _auto_stop():
        stop_event.wait(timeout=MAX_RECORD_S)
        if not stop_event.is_set():
            print(f"\n[recorder] Max duration ({MAX_RECORD_S}s) reached, stopping.")
            stop_event.set()
            sampler.stop()
            plt.close("all")

    threading.Thread(target=_auto_stop, daemon=True).start()

    plt.show(block=True)

    # Finalise video
    if video_writer is not None:
        try:
            video_writer.finish()
            print(f"[video] Saved to: {VIDEO_PATH}")
        except Exception as e:
            print(f"[video] Could not finalise video: {e}")

    # Disconnect
    try:
        session_manager.CloseSession()
        transport.disconnect()
        print("[connection] Disconnected.")
    except Exception:
        pass

    # Save outputs
    final = sampler.snapshot()
    if SAVE_CSV:
        save_csv(final)
    if SAVE_PLOT:
        save_final_plot(final)


if __name__ == "__main__":
    main()
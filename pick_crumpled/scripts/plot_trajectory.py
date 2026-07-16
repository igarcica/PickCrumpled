"""
Kinova Gen3 - Cartesian Pose Logger & Plotter
Connects to the robot, records the end-effector pose while you run your
action from the Kinova Web App, then plots the full trajectory.

Requirements:
    pip install kortex_api matplotlib numpy
    ffmpeg must be installed and on PATH for video saving:
        Linux:   sudo apt install ffmpeg
        macOS:   brew install ffmpeg
        Windows: https://ffmpeg.org/download.html
    (kortex_api wheel: https://github.com/Kinovarobotics/kortex/tree/master/api_python)

Usage:
    1. Run this script.
    2. Start your action from the Kinova Web Application.
    3. Press Ctrl+C when the action is finished to stop recording.
       -> Static plot saved as cartesian_pose.png
       -> Raw data saved as cartesian_pose.csv
       -> Live plot video saved as cartesian_pose.mp4  (if SAVE_VIDEO = True)

Replotting from CSV later:
    import pandas as pd, matplotlib.pyplot as plt
    df = pd.read_csv("cartesian_pose.csv")
    df.plot(x="time_s", y=["x_m", "y_m", "z_m"])
    plt.show()
"""

import time
import signal
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

# ─── Configuration ─────────────────────────────────────────────────────────────
ROBOT_IP     = "192.168.1.12"   # Your robot's IP address
USERNAME     = "admin"
PASSWORD     = "admin"

POLL_RATE_HZ = 20               # Pose samples per second
MAX_RECORD_S = 300              # Safety cap: auto-stop after this many seconds

SAVE_PLOT    = True             # Save final static plot as .png
SAVE_CSV     = True             # Save raw samples as .csv
SAVE_VIDEO   = True             # Save the live plot as .mp4 (requires ffmpeg)

PLOT_PATH    = "cartesian_pose.png"
CSV_PATH     = "cartesian_pose.csv"
VIDEO_PATH   = "cartesian_pose.mp4"
VIDEO_FPS    = 10               # Frames per second in the output video
# ───────────────────────────────────────────────────────────────────────────────


class PoseSampler:
    """Background thread that continuously samples the end-effector Cartesian pose."""

    def __init__(self, base_client: BaseClient):
        self._client  = base_client
        self._lock    = threading.Lock()
        self._running = False
        maxlen = int(POLL_RATE_HZ * MAX_RECORD_S)
        self.times   = deque(maxlen=maxlen)
        self.x_vals  = deque(maxlen=maxlen)
        self.y_vals  = deque(maxlen=maxlen)
        self.z_vals  = deque(maxlen=maxlen)
        self.rx_vals = deque(maxlen=maxlen)
        self.ry_vals = deque(maxlen=maxlen)
        self.rz_vals = deque(maxlen=maxlen)
        self._t0     = None

    def start(self):
        self._running = True
        self._t0 = time.time()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[recorder] Sampling at {POLL_RATE_HZ} Hz. Run your action now.")

    def stop(self):
        self._running = False
        self._thread.join(timeout=2)

    def _loop(self):
        interval = 1.0 / POLL_RATE_HZ
        while self._running:
            t_start = time.time()
            try:
                pose    = self._client.GetMeasuredCartesianPose()
                elapsed = time.time() - self._t0
                with self._lock:
                    self.times.append(elapsed)
                    self.x_vals.append(pose.x)
                    self.y_vals.append(pose.y)
                    self.z_vals.append(pose.z)
                    self.rx_vals.append(pose.theta_x)
                    self.ry_vals.append(pose.theta_y)
                    self.rz_vals.append(pose.theta_z)
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
                list(self.x_vals),
                list(self.y_vals),
                list(self.z_vals),
                list(self.rx_vals),
                list(self.ry_vals),
                list(self.rz_vals),
            )

    @property
    def sample_count(self):
        with self._lock:
            return len(self.times)


# ─── CSV export ────────────────────────────────────────────────────────────────

CSV_HEADER = "time_s,x_m,y_m,z_m,theta_x_deg,theta_y_deg,theta_z_deg"

def save_csv(snapshot, path: str = CSV_PATH):
    t, x, y, z, rx, ry, rz = snapshot
    if not t:
        print("[csv] No data collected – nothing to save.")
        return
    data = np.column_stack([t, x, y, z, rx, ry, rz])
    np.savetxt(path, data, delimiter=",", header=CSV_HEADER, comments="", fmt="%.6f")
    print(f"[csv] {len(t)} samples saved to: {path}")


# ─── Plotting ──────────────────────────────────────────────────────────────────

LABELS = ["X (m)", "Y (m)", "Z (m)", "θx (°)", "θy (°)", "θz (°)"]
COLORS = ["#1f77b4", "#2ca02c", "#d62728", "#9467bd", "#8c564b", "#e377c2"]


def build_live_figure():
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle(
        "Kinova Gen3 - End-Effector Cartesian Pose  (Ctrl+C to stop)",
        fontsize=13, fontweight="bold"
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    plt.subplots_adjust(hspace=0.45, wspace=0.35)

    lines = []
    for ax, lbl, col in zip(axes.flat, LABELS, COLORS):
        (ln,) = ax.plot([], [], color=col, linewidth=1.5)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel(lbl, fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, linestyle="--", alpha=0.5)
        lines.append(ln)

    return fig, axes, lines


def save_final_plot(snapshot, path: str = PLOT_PATH):
    t, x, y, z, rx, ry, rz = snapshot
    if not t:
        print("[plot] No data collected – nothing to save.")
        return

    t    = np.array(t)
    data = [x, y, z, rx, ry, rz]

    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle("Kinova Gen3 - Cartesian Pose Trajectory", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    plt.subplots_adjust(hspace=0.45, wspace=0.35)

    for ax, vals, lbl, col in zip(axes.flat, data, LABELS, COLORS):
        ax.plot(t, vals, color=col, linewidth=1.5)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylabel(lbl, fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, linestyle="--", alpha=0.5)
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
    session_info.session_inactivity_timeout    = 60_000   # ms
    session_info.connection_inactivity_timeout = 2_000

    session_manager = SessionManager(router)
    session_manager.CreateSession(session_info)
    print(f"[connection] Connected to {ROBOT_IP}")

    base_client = BaseClient(router)

    # Start recording
    sampler = PoseSampler(base_client)
    sampler.start()

    # Build live figure
    fig, axes, lines = build_live_figure()
    stop_event = threading.Event()

    def update_plot(_frame):
        t, x, y, z, rx, ry, rz = sampler.snapshot()
        if not t:
            return lines
        data = [x, y, z, rx, ry, rz]
        for ln, ax, vals in zip(lines, axes.flat, data):
            ln.set_data(t, vals)
            ax.relim()
            ax.autoscale_view()
        return lines

    ani = animation.FuncAnimation(
        fig, update_plot, interval=int(1000 / VIDEO_FPS),
        blit=False, cache_frame_data=False
    )

    # Start video writer alongside the live animation
    video_writer = None
    if SAVE_VIDEO:
        try:
            writer_cls  = animation.FFMpegWriter
            video_writer = writer_cls(fps=VIDEO_FPS, bitrate=1800)
            video_writer.setup(fig, VIDEO_PATH, dpi=120)
            print(f"[video] Recording to: {VIDEO_PATH}  (fps={VIDEO_FPS})")

            # Grab a frame every animation tick
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
            print(f"[video] Could not start video writer (is ffmpeg installed?): {e}")
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

    plt.show(block=True)   # blocks until Ctrl+C or window closed

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
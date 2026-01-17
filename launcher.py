# Windows launcher that follows EXACT order:
# 1) Run frameProducer.py
# 2) Trigger Unity Play (Unity already open)
# 3) Run streamer.py
# 4) Wait for streamer port to open -> open website once -> ONE-SHOT touch /video_feed
# 5) Run Kinect
# 6) Focus existing Chrome window + F11

import subprocess
import time
import urllib.request
from pathlib import Path
import sys
import webbrowser
import socket

# Repo root
PROJECT_ROOT = Path(__file__).parent.resolve()  # GLAMCAM/

UNITY_PROJECT_ROOT = PROJECT_ROOT / "GlamCam"
WEBSITE_ROOT = PROJECT_ROOT / "website"

PYTHON_EXE = PROJECT_ROOT / "venv" / "Scripts" / "python.exe"
FRAME_PRODUCER_PY = WEBSITE_ROOT / "frameProducer.py"
STREAMER_PY = WEBSITE_ROOT / "streamer.py"

# Website URLs
BASE_URL = "http://127.0.0.1:5001"
VIDEO_FEED_URL = f"{BASE_URL}/video_feed"

# Kinect exe
KINECT_EXE = (
    PROJECT_ROOT
    / "KinectV2"
    / "KinectSender"
    / "bin"
    / "AnyCPU"
    / "Release"
    / "ColorBasics-WPF.exe"
)

# Used only to avoid opening extra browser tabs every run
OPENED_FLAG = PROJECT_ROOT / ".opened_browser.flag"


def focus_chrome_and_f11_only():
    """
    Focus an existing Chrome window and send F11.
    IMPORTANT: does NOT open/navigate any URL (navigation can create a 2nd /video_feed client).
    """
    if sys.platform != "win32":
        return

    ps = r"""
$ws = New-Object -ComObject WScript.Shell
$deadline = (Get-Date).AddSeconds(6)

while ((Get-Date) -lt $deadline) {
  if ($ws.AppActivate('Chrome')) { break }
  Start-Sleep -Milliseconds 200
}

Start-Sleep -Milliseconds 200
$ws.SendKeys('{F11}')
"""
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], check=False)


def run_py(script: Path) -> subprocess.Popen:
    return subprocess.Popen([str(PYTHON_EXE), str(script)], cwd=str(PROJECT_ROOT))


def assert_started(p: subprocess.Popen, name: str):
    time.sleep(0.2)
    if p.poll() is not None:
        raise RuntimeError(f"{name} exited immediately (crashed).")


def trigger_unity_play():
    (UNITY_PROJECT_ROOT / "PLAY.flag").write_text("go", encoding="utf-8")


def wait_port_open(host: str, port: int, timeout_s: int = 10) -> bool:
    """
    Faster than HTTP polling, just check when the TCP port is accepting connections.
    """
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except Exception:
            time.sleep(0.1)
    return False


def touch_video_feed_one_shot(url: str):
    """
    One-shot prime to kick /video_feed so streamer connects to frameProducer and unblocks accept().
    IMPORTANT: do NOT retry-loop here (can steal/flip the single-consumer connection).
    """
    try:
        with urllib.request.urlopen(url, timeout=2) as r:
            r.read(128)
    except Exception:
        pass


def run_kinect_if_windows():
    if KINECT_EXE.exists():
        return subprocess.Popen([str(KINECT_EXE)], cwd=str(KINECT_EXE.parent))
    else:
        print("[INFO] Skipping Kinect (EXE missing)")
        return None


def main():
    procs = []

    fp = st = kinect = None

    try:
        # 1) Run frameProducer.py
        print("Starting frameProducer.py")
        fp = run_py(FRAME_PRODUCER_PY)
        procs.append(fp)
        assert_started(fp, "frameProducer.py")

        # 2) Trigger Unity Play (Unity already open)
        print("Triggering Unity Play")
        trigger_unity_play()
        time.sleep(0.5)

        # 3) Run streamer.py
        print("Starting streamer.py")
        st = run_py(STREAMER_PY)
        procs.append(st)
        assert_started(st, "streamer.py")

        # 4) Fast wait for streamer port to open, then open site once, then one-shot touch /video_feed
        print("Waiting for streamer port 5001 to open...")
        if wait_port_open("127.0.0.1", 5001, timeout_s=10):
            if not OPENED_FLAG.exists():
                webbrowser.open(BASE_URL, new=0)
                OPENED_FLAG.write_text("opened", encoding="utf-8")

            touch_video_feed_one_shot(VIDEO_FEED_URL)
        else:
            print("[WARN] Streamer port 5001 not open after 10s; skipping browser open/touch.")

        # 5) Run Kinect
        print("Starting Kinect")
        kinect = run_kinect_if_windows()
        if kinect is not None:
            procs.append(kinect)

        # 6) Focus Chrome + fullscreen WITHOUT navigating/reloading
        # print("Focusing Chrome + fullscreen (F11)")
        focus_chrome_and_f11_only()

        print("Done. Process is running, keep this terminal open.")
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Shutting down...")

    finally:
        try:
            (UNITY_PROJECT_ROOT / "STOP.flag").write_text("stop", encoding="utf-8")
        except Exception:
            pass

        # terminate in reverse order (dependents first)
        for p in reversed(procs):
            try:
                if p.poll() is None:
                    p.terminate()
            except Exception:
                pass

        # wait a bit, then force kill if needed
        for p in reversed(procs):
            try:
                if p.poll() is None:
                    p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass

        print("All child processes are stopped!.")


if __name__ == "__main__":
    main()
# Windows launcher that follows EXACT order:
# 1) Run frameProducer.py
# 2) Trigger Unity Play (Unity already open)
# 3) Run streamer.py
# 4) Refresh website (127.0.0.1:5001) + FORCE /video_feed once so frameProducer unblocks
# 5) Run Kinect (recommended: run the built Release .exe)

import subprocess
import time
import urllib.request
from pathlib import Path
import sys
import webbrowser

# Repo root
PROJECT_ROOT = Path(__file__).parent.resolve()  # GLAMCAM/

UNITY_PROJECT_ROOT = PROJECT_ROOT / "GlamCam"
WEBSITE_ROOT = PROJECT_ROOT / "website"

# if sys.platform == "win32":
# PYTHON_EXE = PROJECT_ROOT / "venv" / "Scripts" / "python.exe"
# else:
#     PYTHON_EXE = PROJECT_ROOT / "venv" / "bin" / "python3"

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

OPENED_FLAG = PROJECT_ROOT / ".opened_browser.flag"


def run_py(script: Path) -> subprocess.Popen:
    return subprocess.Popen([str(PYTHON_EXE), str(script)], cwd=str(PROJECT_ROOT))


def assert_started(p: subprocess.Popen, name: str):
    time.sleep(1)
    if p.poll() is not None:
        raise RuntimeError(f"{name} exited immediately (crashed).")


def trigger_unity_play():
    (UNITY_PROJECT_ROOT / "PLAY.flag").write_text("go", encoding="utf-8")


def wait_http(url: str, timeout_s: int = 30) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        try:
            with urllib.request.urlopen(url, timeout=2):
                return True
        except Exception:
            time.sleep(0.5)
    return False


def touch_video_feed(url: str):
    # This forces streamer -> frameProducer socket connect, unblocking frameProducer.accept()
    try:
        with urllib.request.urlopen(url, timeout=3) as r:
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
        time.sleep(1)

        # 3) Run streamer.py
        print("Starting streamer.py")
        st = run_py(STREAMER_PY)
        procs.append(st)
        assert_started(st, "streamer.py")

        # 4) Wait for website, then open it once, then force /video_feed
        print("Waiting for website then opening it")
        if wait_http(BASE_URL, timeout_s=45):
            if not OPENED_FLAG.exists():
                webbrowser.open(BASE_URL, new=0)
                OPENED_FLAG.write_text("opened", encoding="utf-8")

            # Always touch video feed to unblock frameProducer each run
            touch_video_feed(VIDEO_FEED_URL)
        else:
            print(f"[WARN] Website not responding at {BASE_URL}")

        # 5) Run Kinect
        print("Starting Kinect")
        kinect = run_kinect_if_windows()
        if kinect is not None:
            procs.append(kinect)

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

        try:
            OPENED_FLAG.unlink(missing_ok=True)
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

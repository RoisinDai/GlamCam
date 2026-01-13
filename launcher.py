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
PROJECT_ROOT = Path(__file__).parent.parent.resolve()   # GLAMCAM/

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


def run_py(script: Path) -> subprocess.Popen:
    return subprocess.Popen([str(PYTHON_EXE), str(script)], cwd=str(script.parent))


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
        subprocess.Popen([str(KINECT_EXE)], cwd=str(KINECT_EXE.parent))
    else:
        print("[INFO] Skipping Kinect (EXE missing)")


def main():
    # 1) Run frameProducer.py
    print("Starting frameProducer.py")
    fp = run_py(FRAME_PRODUCER_PY)
    assert_started(fp, "frameProducer.py")

    # 2) Trigger Unity Play (Unity already open)
    print("Triggering Unity Play")
    trigger_unity_play()
    time.sleep(1)

    # 3) Run streamer.py
    print("Starting streamer.py")
    st = run_py(STREAMER_PY)
    assert_started(st, "streamer.py")

    # 4) Wait for website, then open it, then force /video_feed once
    print("Waiting for website then opening it")
    if wait_http(BASE_URL, timeout_s=45):
        webbrowser.open(BASE_URL)
        touch_video_feed(VIDEO_FEED_URL)
    else:
        print(f"[WARN] Website not responding at {BASE_URL}")

    # 5) Run Kinect
    print("Starting Kinect")
    run_kinect_if_windows()

    print("Done. Process is running, keep this terminal open.")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()

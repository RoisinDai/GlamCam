#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Full Application Launcher

This script runs the complete application flow:
1. Run InitializationSequence to measure body and generate MakeHuman model
2. After model is created, run launcher.py to start the main application

Usage:
    python run_full_app.py
"""

import subprocess
import sys
import time
from pathlib import Path

# Repo root
PROJECT_ROOT = Path(__file__).parent.resolve()  # GLAMCAM/

# InitializationSequence executable paths (try Release first, then Debug)
INIT_SEQ_RELEASE = (
    PROJECT_ROOT
    / "KinectV2"
    / "InitializationSequence"
    / "bin"
    / "AnyCPU"
    / "Release"
    / "SilhouetteBasics-WPF.exe"
)

INIT_SEQ_DEBUG = (
    PROJECT_ROOT
    / "KinectV2"
    / "InitializationSequence"
    / "bin"
    / "AnyCPU"
    / "Debug"
    / "SilhouetteBasics-WPF.exe"
)

# Expected output model file
MODEL_FILE = PROJECT_ROOT / "GlamCam" / "Assets" / "Base Models" / "clothed_avatar.fbx"

# Launcher script
LAUNCHER_PY = PROJECT_ROOT / "launcher.py"

# Python executable (use same venv as launcher.py)
if sys.platform == "win32":
    PYTHON_EXE = PROJECT_ROOT / "venv" / "Scripts" / "python.exe"
else:
    PYTHON_EXE = PROJECT_ROOT / "venv" / "bin" / "python3"


def find_init_sequence_exe():
    """Find the InitializationSequence executable."""
    if INIT_SEQ_RELEASE.exists():
        return INIT_SEQ_RELEASE
    elif INIT_SEQ_DEBUG.exists():
        print(f"[INFO] Using Debug build (Release not found)")
        return INIT_SEQ_DEBUG
    else:
        raise FileNotFoundError(
            f"InitializationSequence executable not found. Tried:\n"
            f"  - {INIT_SEQ_RELEASE}\n"
            f"  - {INIT_SEQ_DEBUG}\n"
            f"Please build the InitializationSequence project first."
        )


def check_model_exists():
    """Check if the generated model file exists."""
    return MODEL_FILE.exists()


def run_initialization_sequence():
    """Run the InitializationSequence application and wait for completion."""
    print("=" * 60)
    print("Step 1: Running InitializationSequence")
    print("=" * 60)

    init_exe = find_init_sequence_exe()
    print(f"Executable: {init_exe}")

    print("\nStarting InitializationSequence...")
    print("Please measure your body and generate the model in the application window.")
    print("The application will close automatically when the model is generated.\n")

    try:
        # Run the executable and wait for it to complete
        process = subprocess.Popen(
            [str(init_exe)],
            cwd=str(init_exe.parent),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Wait for the process to complete
        stdout, stderr = process.communicate()

        exit_code = process.returncode

        # Decode output
        stdout_text = stdout.decode('utf-8', errors='ignore').strip()
        stderr_text = stderr.decode('utf-8', errors='ignore').strip()

        # Print stdout if present
        if stdout_text:
            print("\n--- Standard Output ---")
            print(stdout_text)
            print("-" * 50)

        # Print stderr if present
        if stderr_text:
            print("\n--- Standard Error ---")
            print(stderr_text)
            print("-" * 50)

        if exit_code != 0:
            print(f"\n[WARN] InitializationSequence exited with code {exit_code}")

            # Save full output to a log file for easier viewing
            log_file = PROJECT_ROOT / "initialization_sequence_output.log"
            with open(log_file, 'w', encoding='utf-8') as f:
                f.write("=== InitializationSequence Output Log ===\n\n")
                f.write(f"Exit Code: {exit_code}\n\n")
                if stdout_text:
                    f.write("--- STDOUT ---\n")
                    f.write(stdout_text)
                    f.write("\n\n")
                if stderr_text:
                    f.write("--- STDERR ---\n")
                    f.write(stderr_text)
                    f.write("\n")

            print(f"\nFull output saved to: {log_file}")
        else:
            print("\nInitializationSequence completed successfully.")

        return exit_code == 0

    except Exception as e:
        print(f"\n[ERROR] Failed to run InitializationSequence: {e}")
        return False


def verify_model_created():
    """Verify that the model file was created."""
    print("\n" + "=" * 60)
    print("Step 2: Verifying Model Creation")
    print("=" * 60)

    if check_model_exists():
        print(f"✓ Model file found: {MODEL_FILE}")
        print(f"  File size: {MODEL_FILE.stat().st_size / 1024:.2f} KB")
        return True
    else:
        print(f"✗ Model file not found: {MODEL_FILE}")
        print("\nThe model may not have been generated successfully.")
        print("Please check the InitializationSequence application for errors.")
        return False


def run_launcher():
    """Run the main launcher.py script."""
    print("\n" + "=" * 60)
    print("Step 3: Starting Main Application")
    print("=" * 60)

    if not LAUNCHER_PY.exists():
        raise FileNotFoundError(f"Launcher script not found: {LAUNCHER_PY}")

    if not PYTHON_EXE.exists():
        raise FileNotFoundError(
            f"Python executable not found: {PYTHON_EXE}\n"
            f"Please ensure the virtual environment is set up."
        )

    print(f"Python: {PYTHON_EXE}")
    print(f"Launcher: {LAUNCHER_PY}\n")

    # Import and run launcher.main() directly
    # This allows us to reuse the launcher's logic without spawning a new process
    sys.path.insert(0, str(PROJECT_ROOT))
    import launcher

    print("Starting launcher.py...\n")
    launcher.main()


def main():
    """Main execution function."""
    print("\n" + "=" * 60)
    print("GlamCam Full Application Launcher")
    print("=" * 60)
    print("\nThis script will:")
    print("  1. Run InitializationSequence to measure body and generate model")
    print("  2. Verify model was created")
    print("  3. Start the main application (launcher.py)")
    print()

    try:
        # Step 1: Run InitializationSequence
        init_success = run_initialization_sequence()

        if not init_success:
            print("\n[ERROR] InitializationSequence failed.")
            response = input(
                "\nDo you want to continue anyway? (y/n): "
            ).strip().lower()
            if response != "y":
                print("Aborting.")
                return 1

        # Step 2: Verify model was created
        model_exists = verify_model_created()

        if not model_exists:
            print("\n[WARN] Model file not found.")
            response = input(
                "\nDo you want to continue anyway? (y/n): "
            ).strip().lower()
            if response != "y":
                print("Aborting.")
                return 1

        # Step 3: Run launcher.py
        print("\n" + "=" * 60)
        print("Proceeding to main application...")
        print("=" * 60)
        time.sleep(1)

        run_launcher()

    except KeyboardInterrupt:
        print("\n\nCtrl+C detected. Exiting...")
        return 130

    except FileNotFoundError as e:
        print(f"\n[ERROR] {e}")
        return 1

    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

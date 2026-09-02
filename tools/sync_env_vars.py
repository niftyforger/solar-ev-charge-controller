#!/usr/bin/env python3
"""One-time setup so the PlatformIO IDE's plain "Upload" button (VSCode
toolbar, `ota` environment selected) works without going through
tools/ota_upload.py or exporting anything by hand first - persists
PLATFORMIO_UPLOAD_PORT/PLATFORMIO_UPLOAD_FLAGS as Windows user environment
variables (via `setx`) so they already exist before VSCode itself launches.
See the [env:ota] comment in platformio.ini for why a build-time mechanism
can't set these in time instead.

Run this after every edit to .env's OTA_PASSWORD/UPLOAD_PORT, then RESTART
VSCode - `setx` writes to the registry for FUTURE processes; it cannot
change the environment of the already-running VSCode window or any terminal
already open under it.
"""
import os
import subprocess
import sys

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ENV_FILE = os.path.join(PROJECT_DIR, ".env")


def load_env_file(path):
    values = {}
    if not os.path.isfile(path):
        return values
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, _, value = line.partition("=")
            values[key.strip()] = value.strip()
    return values


def main():
    if os.name != "nt":
        sys.stderr.write(
            "sync_env_vars.py: Windows-only (uses `setx`). On macOS/Linux, "
            "export PLATFORMIO_UPLOAD_PORT/PLATFORMIO_UPLOAD_FLAGS in your "
            "shell profile instead.\n"
        )
        return 1

    values = load_env_file(ENV_FILE)
    missing = [k for k in ("OTA_PASSWORD", "UPLOAD_PORT") if not values.get(k)]
    if missing:
        sys.stderr.write(
            f"sync_env_vars.py: {', '.join(missing)} not set in .env. Copy "
            ".env.example to .env and fill them in first.\n"
        )
        return 1

    to_set = {
        "PLATFORMIO_UPLOAD_PORT": values["UPLOAD_PORT"],
        "PLATFORMIO_UPLOAD_FLAGS": "--auth=" + values["OTA_PASSWORD"],
    }
    for key, value in to_set.items():
        subprocess.check_call(["setx", key, value])

    print(
        "\nDone. Restart VSCode (setx only affects NEW processes) for the "
        "PlatformIO IDE's Upload button to pick these up."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

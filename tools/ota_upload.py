#!/usr/bin/env python3
"""Runs `pio run -e ota -t upload` with OTA_PASSWORD/UPLOAD_PORT loaded from
a gitignored ".env" file (template: .env.example), translated into the
PLATFORMIO_UPLOAD_PORT/PLATFORMIO_UPLOAD_FLAGS env vars `pio` itself reads -
see the [env:ota] comment in platformio.ini for why that translation can't
happen in a pre-build script instead. For the PlatformIO IDE's own "Upload"
button (no wrapper involved), see tools/sync_env_vars.py instead.

Any real, already-exported PLATFORMIO_UPLOAD_PORT/PLATFORMIO_UPLOAD_FLAGS
still wins over .env, same precedence PlatformIO's env vars already imply.
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
    env = os.environ.copy()
    dotenv = load_env_file(ENV_FILE)
    for key, value in dotenv.items():
        env.setdefault(key, value)

    if not env.get("OTA_PASSWORD") or not env.get("UPLOAD_PORT"):
        print(
            "OTA_PASSWORD and/or UPLOAD_PORT not set. Copy .env.example to "
            ".env and fill them in, or export them directly.",
            file=sys.stderr,
        )
        return 1

    env.setdefault("PLATFORMIO_UPLOAD_PORT", env["UPLOAD_PORT"])
    env.setdefault("PLATFORMIO_UPLOAD_FLAGS", "--auth=" + env["OTA_PASSWORD"])

    cmd = ["pio", "run", "-e", "ota", "-t", "upload"] + sys.argv[1:]
    return subprocess.call(cmd, cwd=PROJECT_DIR, env=env)


if __name__ == "__main__":
    sys.exit(main())

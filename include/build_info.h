#pragma once

// Identity of the running firmware, as a human-readable build stamp
// ("2026-09-06 14:32:05 UTC (4f2a1c9)"). Generated fresh on every build by
// tools/sync_build_info.py into the gitignored include/build_info_stamp.h,
// which only src/build_info.cpp includes - so callers get a stable header
// and the per-build churn stays in one small translation unit.
//
// Serves two purposes at once, which is why it's a build-time string rather
// than a runtime hash of the image: it's unique per build (so the control
// page can compare it across polls and reload itself when the board has been
// reprogrammed) and it's readable (so it can be shown in the page footer).
//
// The obvious runtime alternatives don't work here:
//   - esp_ota_get_app_description()->app_elf_sha256 is all zeros, because
//     PlatformIO's Arduino-framework ElfToBin builder doesn't pass
//     --elf-sha256-offset to esptool (only its ESP-IDF path does), and that
//     struct's date/time fields come from arduino-esp32's *precompiled*
//     esp_app_desc.c, so they're constant across our builds.
//   - ESP.getSketchMD5() does hash the real flashed image, but costs a
//     ~1.2MB flash read and yields only opaque hex.
const char *firmware_build_id();

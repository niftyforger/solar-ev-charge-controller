#include "build_info.h"

// The only translation unit that includes the generated stamp header, so a
// rebuild costs this file plus a relink rather than recompiling everything
// that wants to know the firmware's identity. See include/build_info.h.
#include "build_info_stamp.h"

const char *firmware_build_id() {
    return FIRMWARE_BUILD_STAMP;
}

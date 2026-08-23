#pragma once

// Core 0 task: 20x4 I2C status LCD, refreshed at ~1Hz. Purely informational
// - never touched from Core 1, and a stuck/disconnected LCD cannot affect
// the control loop since this task only ever reads shared status snapshots.
void status_display_task(void *pvParameters);

#pragma once

// Core 1 task, pinned + high priority: real-time CP interception.
//
// Captures the EVSE's native PWM edges via a GPIO interrupt (both edges,
// microsecond timestamps), and generates the clamp assert/release waveform
// via the RMT peripheral so the actual pulse timing is hardware-scheduled
// rather than software-toggled - only the trigger (rising-edge ISR ->
// woken task -> rmt_write_items) carries software latency, not the
// assert/release edges themselves.
void cp_interceptor_task(void *pvParameters);

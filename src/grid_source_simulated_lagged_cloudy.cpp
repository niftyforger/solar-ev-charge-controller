#include "grid_data_source.h"
#include "config.h"
#include <math.h>

// Lagged/cloudy simulated grid source - the only source that can reproduce the control
// loop's measurement-lag oscillation, and therefore the regression test for the damping
// added alongside it (see CLAUDE.md "Control-loop damping").
//
// Every other simulated source computes outWatts from the INSTANTANEOUS currentDrawW:
// the commanded draw is reflected back perfectly, with no lag and no noise. That is
// exactly the case the deadbeat control law was designed for, and it is unconditionally
// stable at gain 1 - which is precisely why the oscillation was never caught on the
// bench despite the reactive family being otherwise faithful. This source adds the two
// things reality has and they don't:
//
//   1. The plant doesn't follow instantly. A real vehicle ramps its current over seconds
//      after a CP duty change, and the DTSU666 -> RS485 -> inverter register chain adds
//      its own transport delay on top. Feed a deadbeat integrator a reading that hasn't
//      absorbed its last correction and it double-counts: at exactly one decision-period
//      of lag the error poles sit on the unit circle (z^2 - z + 1 = 0), an undamped ring
//      with a period of six decisions. Anything beyond that lag rings harder.
//   2. The surplus itself moves. Cloud transients are what turn a marginal loop into a
//      visibly oscillating one.
//
// Both models are deterministic - no random() anywhere - because the whole point is that
// a run before the fix and a run after it are comparable.
static const float SIMULATED_HOUSE_LOAD_W = 800.0f;

// PV sweeps 2000-5600W, so surplus after house load sweeps 1200-4800W = 5.0-20.0A at
// 240V. Deliberately spans the 6A floor at the bottom and sits well inside MAX_CURRENT_A
// at the top, so entry/exit and steady-state tracking are all exercised. It also covers
// 6-8.6A, a band no existing simulated source lands in - the band where damping the gain
// at target 0 would silently stop charging from ever starting.
static const float SIMULATED_PV_BASE_W = 3800.0f;

// Two incommensurate periods, so the sum never repeats on a short cycle and the loop
// can't accidentally lock to it. ~17s is roughly a decision period (the frequency a
// marginal loop rings at); ~6.3s is faster than the poll interval, modelling the
// high-frequency component that single-sample decisions alias.
static const float SIMULATED_CLOUD_SLOW_AMP_W = 1200.0f;
static const float SIMULATED_CLOUD_SLOW_PERIOD_S = 17.0f;
static const float SIMULATED_CLOUD_FAST_AMP_W = 600.0f;
static const float SIMULATED_CLOUD_FAST_PERIOD_S = 6.3f;

// How fast the modelled vehicle ramps toward the commanded draw. 480W/s = 2A/s at 240V,
// so a 10A step takes ~5s to land - a rate limit, not an exponential, because that's
// what a real vehicle's current ramp actually is.
static const float SIMULATED_EV_RAMP_W_PER_S = 480.0f;

// Meter/register transport delay, in polls. The value returned is the one computed
// SIMULATED_LAG_POLLS polls ago, so at POLL_INTERVAL_MS this is ~10s of pure delay on
// top of the vehicle ramp above.
#define SIMULATED_LAG_POLLS 2

// Battery data goes briefly untrustworthy on a slow, fixed cycle: 2 polls out of every
// 24 (~10s in every ~2min). No other simulated source ever reports false, so without
// this the rule that an averaged increase must not sneak past an unverified battery
// reading has no test at all. Kept rare on purpose - frequent enough to be observed in a
// few minutes of bench time, too rare to distort the oscillation being reproduced.
#define SIMULATED_BATTERY_INVALID_CYCLE_POLLS 24
#define SIMULATED_BATTERY_INVALID_POLLS 2

// Modelled vehicle draw, ramped toward currentDrawW rather than snapping to it.
static float s_evDrawW = 0.0f;
static uint32_t s_lastCallMs = 0;
static bool s_everCalled = false;

// Own accumulated phase rather than sinf() on millis() directly - millis() rolls over at
// ~49 days and would step the cloud waveform discontinuously at the wrap.
static float s_phaseS = 0.0f;

// Transport-delay FIFO. Seeded on the first call (below) rather than zero-initialized, so
// the first few polls report a plausible reading instead of a fictitious 0W.
static float s_lagBuf[SIMULATED_LAG_POLLS];
static size_t s_lagIdx = 0;

static uint32_t s_pollCount = 0;

static bool simulated_lagged_cloudy_read_power_w(IPAddress /*host*/, float currentDrawW,
                                                    float &outWatts, float &outVoltageV,
                                                    float &outBatteryW, bool &outBatteryDataValid) {
    uint32_t nowMs = millis();
    // Advance on elapsed wall time, not per call, so the model stays honest if
    // POLL_INTERVAL_MS ever changes.
    float dtS = s_everCalled ? (float)(nowMs - s_lastCallMs) / 1000.0f : 0.0f;
    s_lastCallMs = nowMs;
    s_phaseS += dtS;

    // Vehicle ramp: move toward the commanded draw at a bounded rate.
    float maxStepW = SIMULATED_EV_RAMP_W_PER_S * dtS;
    float errW = currentDrawW - s_evDrawW;
    if (errW > maxStepW) {
        s_evDrawW += maxStepW;
    } else if (errW < -maxStepW) {
        s_evDrawW -= maxStepW;
    } else {
        s_evDrawW = currentDrawW;
    }

    float pvW = SIMULATED_PV_BASE_W
        + SIMULATED_CLOUD_SLOW_AMP_W * sinf(2.0f * (float)M_PI * s_phaseS / SIMULATED_CLOUD_SLOW_PERIOD_S)
        + SIMULATED_CLOUD_FAST_AMP_W * sinf(2.0f * (float)M_PI * s_phaseS / SIMULATED_CLOUD_FAST_PERIOD_S);

    float instantW = s_evDrawW + SIMULATED_HOUSE_LOAD_W - pvW; // negative = exporting

    if (!s_everCalled) {
        // Prime the delay line with the first reading so early polls aren't answered from
        // a zero-filled buffer.
        for (size_t i = 0; i < SIMULATED_LAG_POLLS; i++) {
            s_lagBuf[i] = instantW;
        }
        s_everCalled = true;
    }

    // Emit the oldest entry, then overwrite it with the newest - a plain ring used as a
    // fixed-length delay line.
    outWatts = s_lagBuf[s_lagIdx];
    s_lagBuf[s_lagIdx] = instantW;
    s_lagIdx = (s_lagIdx + 1) % SIMULATED_LAG_POLLS;

    outVoltageV = MAINS_VOLTAGE_FIXED_V;
    outBatteryW = 0.0f; // no battery modeled
    outBatteryDataValid =
        (s_pollCount % SIMULATED_BATTERY_INVALID_CYCLE_POLLS) >= SIMULATED_BATTERY_INVALID_POLLS;
    s_pollCount++;
    return true;
}

const GridDataSource GRID_SOURCE_SIMULATED_LAGGED_CLOUDY = {
    "simulated_lagged_cloudy",
    // No '&' or '<' in the name: the control page interpolates it into innerHTML.
    "Simulated - Lagged and Cloudy (oscillation reproduction)",
    simulated_lagged_cloudy_read_power_w,
};

/*
 * A 30-second city-traffic scenario, deterministic so every run sees exactly the same input:
 *   0–8 s   cruise at 50 km/h, 40 m behind a car going the same speed           -> OK
 *   8–10 s  the car ahead brakes hard (to 10 km/h); we keep going at 50 km/h   -> WARN at ~10.1 s, BRAKE at ~11.4 s
 *   11.7 s  the driver reacts to the alarm and brakes hard (8 m/s^2)           -> OK at ~12.8 s, stops ~2.7 m short
 *   18–20 s the radar cable is "unplugged" (no frames for 0.5 s)               -> SENSOR_STALE
 *   20–30 s calm driving again                                                   -> OK
 */
#include "scenario.h"

#define HZ 100.0
#define DURATION_S 30.0

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

const char *scenario_name(void) { return "city_hard_brake_30s"; }

int scenario_dropout(uint32_t i)
{
    double t = i / HZ;
    return t >= 18.0 && t < 18.5;
}

int scenario_frame(uint32_t i, frame_t *out)
{
    /* integrate both vehicles step by step from 0 so the state is consistent */
    static uint32_t last_i = 0;
    static double own_v = 50.0 / 3.6, lead_v = 50.0 / 3.6, gap = 40.0;
    if (i == 0) { last_i = 0; own_v = 50.0 / 3.6; lead_v = 50.0 / 3.6; gap = 40.0; }

    double t = i / HZ;
    if (t > DURATION_S) return 0;

    for (; last_i < i; last_i++) {
        double tt = last_i / HZ, dt = 1.0 / HZ;
        /* lead car: brakes hard between 8 and 10 s, then holds 10 km/h, speeds up after 20 s */
        if (tt >= 8.0 && tt < 10.0)  lead_v -= 5.5 * dt;
        if (tt >= 20.0)              lead_v += 1.0 * dt;
        lead_v = clampd(lead_v, 10.0 / 3.6, 50.0 / 3.6);
        /* own car: keeps speed until the driver reacts to the alarm at 11.7 s */
        if (tt >= 11.7 && tt < 15.7) own_v -= 8.0 * dt;
        if (tt >= 20.0)              own_v += 0.8 * dt;
        own_v = clampd(own_v, lead_v * 0.9, 50.0 / 3.6);
        gap += (lead_v - own_v) * dt;
        gap = clampd(gap, 0.5, 200.0);
    }

    out->t_s = t;
    out->speed_kmh = own_v * 3.6;
    out->gap_m = gap;
    out->closing_mps = own_v - lead_v;
    out->brake_pedal = (t >= 11.7 && t < 15.7);
    return 1;
}

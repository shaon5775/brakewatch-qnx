#ifndef BRAKEWATCH_SCENARIO_H
#define BRAKEWATCH_SCENARIO_H
#include <stdint.h>

/* One CAN-like frame (what a radar + wheel-speed ECU would publish). */
typedef struct {
    double t_s;          /* scenario time, seconds */
    double speed_kmh;    /* own speed */
    double gap_m;        /* distance to the vehicle ahead */
    double closing_mps;  /* how fast the gap shrinks (>0 = getting closer) */
    int    brake_pedal;  /* 1 if the driver presses the brake */
} frame_t;

/* Fills frame i (100 Hz). Returns 0 when the scenario is over. */
int scenario_frame(uint32_t i, frame_t *out);
/* 1 if frame i is lost (sensor dropout test). */
int scenario_dropout(uint32_t i);
const char *scenario_name(void);

#endif

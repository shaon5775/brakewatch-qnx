/*
 * BrakeWatch — a hard-deadline forward-collision watchdog for QNX 8.0.
 *
 * Three kinds of threads, all in one process:
 *   1. sensor   (prio 40) : replays a CAN-like stream at 100 Hz (speed, gap to the car ahead, closing speed,
 *                           brake pedal). Each frame is sent to the monitor with QNX native message passing
 *                           (MsgSend -> MsgReceive -> MsgReply), so the sender blocks until the monitor replies.
 *   2. monitor  (prio 60) : receives each frame, computes time-to-collision (TTC) and decides
 *                           OK / WARN / BRAKE. A periodic QNX timer delivers a pulse every 10 ms; if no
 *                           fresh frame arrived in the last 30 ms the monitor raises SENSOR_STALE (fail-safe).
 *                           Every decision is time-stamped with ClockCycles() to measure reaction latency
 *                           (frame sent -> decision made).
 *   3. load     (prio 10) : N busy threads that eat the CPU ("infotainment / logging"). With a real-time
 *                           scheduler their load must NOT change the monitor's worst-case latency.
 *
 * Output: one line per state change + a latency summary (min / mean / p99 / max in microseconds) and a CSV
 * (latency_<tag>.csv) so the demo can plot "no load" vs "full load".
 *
 * Build (QNX SDP 8.0):  qcc -Vgcc_ntox86_64 -O2 -Wall -o brakewatch brakewatch.c scenario.c
 * Run on target/QEMU:   ./brakewatch -l 4 -t full_load      (4 load threads)
 *                       ./brakewatch -l 0 -t no_load
 */
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>

#include "scenario.h"

#define PRIO_MONITOR 60
#define PRIO_SENSOR  40
#define PRIO_LOAD    10
#define FRAME_PERIOD_NS   10000000L   /* 100 Hz CAN-like stream */
#define WATCHDOG_PERIOD_NS 10000000L  /* monitor pulse every 10 ms */
#define STALE_LIMIT_NS    30000000LL  /* no frame for 30 ms -> fail-safe */
#define TTC_WARN_S  2.5
#define TTC_BRAKE_S 1.2
#define MAX_SAMPLES 20000

#define PULSE_CODE_WATCHDOG (_PULSE_CODE_MINAVAIL + 1)

typedef enum { ST_OK = 0, ST_WARN, ST_BRAKE, ST_STALE } state_t;
static const char *STATE_NAME[] = { "OK", "WARN", "BRAKE", "SENSOR_STALE" };

typedef struct {
    uint16_t type;          /* _IO_MAX + 1 so it never collides with resource-manager messages */
    uint16_t pad;
    uint32_t seq;
    uint64_t sent_cycles;   /* ClockCycles() when the sensor sent the frame */
    frame_t  f;
} frame_msg_t;

typedef struct {
    uint32_t seq;
    int32_t  state;
} frame_reply_t;

typedef union {
    struct _pulse pulse;
    frame_msg_t   frame;
} rx_msg_t;

static int g_chid = -1;
static volatile int g_running = 1;
static uint64_t g_cps;                 /* cycles per second */
static double   g_lat_us[MAX_SAMPLES];
static int      g_nlat = 0;

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void set_self_prio(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) fprintf(stderr, "warning: could not set priority %d: %s\n", prio, strerror(rc));
}

static state_t decide(const frame_t *f, double *ttc_out)
{
    double ttc = 99.0;
    if (f->closing_mps > 0.1) ttc = f->gap_m / f->closing_mps;
    *ttc_out = ttc;
    if (f->brake_pedal && ttc > TTC_BRAKE_S) return ST_OK;   /* driver already braking: no nagging */
    if (ttc <= TTC_BRAKE_S) return ST_BRAKE;
    if (ttc <= TTC_WARN_S)  return ST_WARN;
    return ST_OK;
}

/* ---------------- monitor: highest priority, owns the channel ---------------- */
static void *monitor_thread(void *arg)
{
    (void)arg;
    set_self_prio(PRIO_MONITOR);

    /* periodic watchdog pulse delivered to our own channel */
    int coid = ConnectAttach(0, 0, g_chid, _NTO_SIDE_CHANNEL, 0);
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, PRIO_MONITOR, PULSE_CODE_WATCHDOG, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("timer_create"); return NULL; }
    struct itimerspec its = { { 0, WATCHDOG_PERIOD_NS }, { 0, WATCHDOG_PERIOD_NS } };
    timer_settime(tid, 0, &its, NULL);

    state_t state = ST_OK;
    uint64_t last_frame_ns = now_ns();
    rx_msg_t msg;

    while (g_running) {
        int rcvid = MsgReceive(g_chid, &msg, sizeof msg, NULL);
        if (rcvid == -1) { if (errno == EINTR) continue; perror("MsgReceive"); break; }

        if (rcvid == 0) {                              /* pulse */
            if (msg.pulse.code == PULSE_CODE_WATCHDOG) {
                if (now_ns() - last_frame_ns > STALE_LIMIT_NS && state != ST_STALE) {
                    state = ST_STALE;
                    printf("[%8.3f s] SENSOR_STALE  no CAN frame for >30 ms -> fail-safe warning\n",
                           now_ns() / 1e9);
                }
            }
            continue;
        }

        /* a frame from the sensor thread */
        uint64_t got = ClockCycles();
        double ttc;
        state_t next = decide(&msg.frame.f, &ttc);
        uint64_t done = ClockCycles();
        last_frame_ns = now_ns();

        frame_reply_t rep = { msg.frame.seq, next };
        MsgReply(rcvid, EOK, &rep, sizeof rep);

        double lat_us = (double)(done - msg.frame.sent_cycles) * 1e6 / (double)g_cps;
        (void)got;
        if (g_nlat < MAX_SAMPLES) g_lat_us[g_nlat++] = lat_us;

        if (next != state) {
            printf("[t=%6.2f s] %-12s speed %5.1f km/h  gap %5.1f m  closing %5.1f m/s  TTC %5.2f s  (reaction %.1f us)\n",
                   msg.frame.f.t_s, STATE_NAME[next], msg.frame.f.speed_kmh, msg.frame.f.gap_m,
                   msg.frame.f.closing_mps, ttc, lat_us);
            state = next;
        }
    }
    timer_delete(tid);
    ConnectDetach(coid);
    return NULL;
}

/* ---------------- sensor: replays the scenario at 100 Hz ---------------- */
static void *sensor_thread(void *arg)
{
    (void)arg;
    set_self_prio(PRIO_SENSOR);
    int coid = ConnectAttach(0, 0, g_chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("ConnectAttach"); g_running = 0; return NULL; }

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    frame_msg_t m;
    memset(&m, 0, sizeof m);
    m.type = _IO_MAX + 1;

    for (uint32_t i = 0; g_running && scenario_frame(i, &m.f); i++) {
        if (scenario_dropout(i)) goto sleep;          /* simulate a broken sensor cable */
        m.seq = i;
        m.sent_cycles = ClockCycles();
        frame_reply_t rep;
        if (MsgSend(coid, &m, sizeof m, &rep, sizeof rep) == -1) { perror("MsgSend"); break; }
sleep:
        next.tv_nsec += FRAME_PERIOD_NS;
        while (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    g_running = 0;
    ConnectDetach(coid);
    return NULL;
}

/* ---------------- load: low-priority CPU hogs ---------------- */
static void *load_thread(void *arg)
{
    (void)arg;
    set_self_prio(PRIO_LOAD);
    volatile double x = 1.0;
    while (g_running) { for (int i = 0; i < 100000; i++) x = x * 1.0000001 + 0.5; }
    return NULL;
}

static int cmp_double(const void *a, const void *b)
{
    double d = *(const double *)a - *(const double *)b;
    return (d > 0) - (d < 0);
}

int main(int argc, char **argv)
{
    int nload = 0;
    const char *tag = "run";
    int opt;
    while ((opt = getopt(argc, argv, "l:t:")) != -1) {
        if (opt == 'l') nload = atoi(optarg);
        else if (opt == 't') tag = optarg;
        else { fprintf(stderr, "usage: %s [-l load_threads] [-t tag]\n", argv[0]); return 1; }
    }

    g_cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    g_chid = ChannelCreate(_NTO_CHF_PRIVATE);
    if (g_chid == -1) { perror("ChannelCreate"); return 1; }

    printf("BrakeWatch on QNX — %d load thread(s), scenario: %s\n", nload, scenario_name());

    pthread_t mon, sen, load[16];
    if (nload > 16) nload = 16;
    pthread_create(&mon, NULL, monitor_thread, NULL);
    for (int i = 0; i < nload; i++) pthread_create(&load[i], NULL, load_thread, NULL);
    usleep(50000);
    pthread_create(&sen, NULL, sensor_thread, NULL);

    pthread_join(sen, NULL);
    g_running = 0;
    for (int i = 0; i < nload; i++) pthread_join(load[i], NULL);
    pthread_cancel(mon);

    /* latency summary */
    if (g_nlat > 0) {
        char path[128];
        snprintf(path, sizeof path, "latency_%s.csv", tag);
        FILE *fp = fopen(path, "w");
        if (fp) { fprintf(fp, "seq,latency_us\n"); for (int i = 0; i < g_nlat; i++) fprintf(fp, "%d,%.2f\n", i, g_lat_us[i]); fclose(fp); }
        double sum = 0;
        for (int i = 0; i < g_nlat; i++) sum += g_lat_us[i];
        qsort(g_lat_us, (size_t)g_nlat, sizeof(double), cmp_double);
        printf("\nreaction latency over %d frames (us): min %.1f  mean %.1f  p99 %.1f  max %.1f   -> %s\n",
               g_nlat, g_lat_us[0], sum / g_nlat, g_lat_us[(int)(g_nlat * 0.99)], g_lat_us[g_nlat - 1], path);
    }
    ChannelDestroy(g_chid);
    return 0;
}

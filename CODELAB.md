# Codelab: measure a hard real-time deadline on QNX 8.0 — in the cloud, for free

**What you'll build:** a tiny safety monitor that receives sensor frames through QNX native message passing, makes a
decision for each one, raises a fail-safe alarm if the sensor goes silent, and proves with numbers that heavy CPU load
does not delay it.

**What you'll learn:** QNX channels and `MsgSend`/`MsgReceive`/`MsgReply`, timer pulses as a watchdog, `SCHED_FIFO`
priorities, `ClockCycles()` timing, and how to run QNX 8.0 on QEMU without owning any hardware.

**Time:** about 60 minutes. **Cost:** free (QNX Everywhere licence + a GitHub Codespace).

---

## 1. Get QNX 8.0 (10 min)

1. Create a myQNX account and request the free **QNX Everywhere** licence (non-commercial) on qnx.com.
2. In the myQNX License Manager, **Accept** the licence and **Deploy** it to your own email.
3. Download **QNX Software Center 2.0.4 – Linux hosts** (`qnx-setup-*-linux.run`).

> Tip: you don't need a Linux PC. A free GitHub Codespace (2 cores, 8 GB) is enough and even has `/dev/kvm`.

## 2. Install the SDP from the command line (15 min)

QNX Software Center has a command-line tool, so no desktop is needed:

```bash
./qnx-setup-2.0.4-*-linux.run --noexec --nox11 --target ~/qsc          # unpack only
read -s -p "myQNX password: " QPW; echo                                   # never type it on the command line
~/qsc/qnxsoftwarecenter/qnxsoftwarecenter_clt \
    -myqnx.user you@example.com -myqnx.password "$QPW" \
    -syncLicenseKeys -activateAll -mirror \
    -installBaseline com.qnx.qnx800 -destination ~/qnx800 -cleanInstall
unset QPW
source ~/qnx800/qnxsdp-env.sh
```

Common errors and fixes:
- *"No registered license keys"* → add `-syncLicenseKeys -activateAll`.
- *"run with -mirrorBaseline first"* / *"requires com.qnx.qnx800.target … could not be found"* → add `-mirror`.
- *"Connect timed out"* → just run it again; downloads resume.

## 3. Build and boot a QNX image on QEMU (5 min)

```bash
sudo apt-get install -y qemu-system-x86
sudo chmod 666 /dev/kvm                       # Codespaces: let your user use KVM
mkdir -p ~/vm && cd ~/vm
mkqnximage --type=qemu --arch=x86_64 --build
```

`mkqnximage --run` wants a network bridge (`brctl`), which containers such as Codespaces cannot create. For a
console-only VM, make a private copy of the tool and skip the networking block:

```bash
mkdir ~/mkqbin && cp $QNX_HOST/../../common/bin/mkqnximage ~/mkqbin/
cp -r $QNX_HOST/../../common/mkqnximage ~/mkqbin/tool_dir
sed -i '35s/.*/if true; then/' ~/mkqbin/tool_dir/qemu/runimage   # "Networking not supported" branch
~/mkqbin/mkqnximage --run=-h                                      # console goes to output/qemu.out
```

## 4. The real-time core (read `src/brakewatch.c`)

**A channel and a client connection**

```c
int chid = ChannelCreate(_NTO_CHF_PRIVATE);
int coid = ConnectAttach(0, 0, chid, _NTO_SIDE_CHANNEL, 0);
```

**The sensor sends and blocks until the monitor answers** — this is what makes the latency measurement honest:

```c
m.sent_cycles = ClockCycles();
MsgSend(coid, &m, sizeof m, &rep, sizeof rep);
```

**The monitor receives frames and pulses on the same channel**

```c
int rcvid = MsgReceive(chid, &msg, sizeof msg, NULL);
if (rcvid == 0)        { /* a pulse: the 10 ms watchdog tick */ }
else { decide(...); MsgReply(rcvid, EOK, &rep, sizeof rep); }
```

**A timer that delivers a pulse every 10 ms (the watchdog)**

```c
struct sigevent ev;
SIGEV_PULSE_INIT(&ev, coid, PRIO_MONITOR, PULSE_CODE_WATCHDOG, 0);
timer_create(CLOCK_MONOTONIC, &ev, &tid);
```

**Priorities**: monitor 60, main 50, sensor 40, CPU hogs 10 — all `SCHED_FIFO` via `pthread_setschedparam`.

## 5. Put the program into the image and run it at boot

```bash
make                                              # qcc -Vgcc_ntox86_64 ...
cd ~/vm
echo '[uid=0 gid=0 perms=755] var/brakewatch=/path/to/brakewatch' >> local/snippets/data_files.custom
cat >> local/snippets/post_start.custom <<'EOF'
cd /tmp
/data/var/brakewatch -l 0 -t no_load
/data/var/brakewatch -l 4 -t full_load
EOF
mkqnximage --build --noprompt && ~/mkqbin/mkqnximage --run=-h
grep -a -E 'WARN|BRAKE|STALE|latency over' output/qemu.out
```

You should see WARN at 10.14 s, BRAKE at 11.44 s and SENSOR_STALE at 18.03 s in **both** runs, and a latency summary
like `mean 12.7  p99 30.5  max 960.3` (µs).

## 6. The lesson we learned the hard way

Our first loaded run produced no frames at all. `main` was left at the default priority (10), the same as the four
`SCHED_FIFO` busy threads — so it never ran again to create the sensor thread. On a real-time OS **start-up order and
priorities are part of the design**: give the thread that wires the system together a priority above the load, start
the safety path first, and make the watchdog arm only after the first real frame.

## 7. Next steps

- Try more load threads, or lower the monitor's priority below the load and watch the deadline break.
- Add a second channel for a real CAN driver, or run on a Raspberry Pi 5 with QNX 8.0.

# EtherCAT Stability Test

This tool is for repeated boot stability experiments. The default output is one
daily summary, so many reboot tests remain easy to compare.

## Files

- `tools/ecat_stability_test.sh`: main test script.
- `tools/ecat-stability-test.service`: optional 30 minute boot test service.
- `tools/ecat_main.service`: optional service template for automatic `ecat_main`.

## Daily Output

Default log root:

```bash
/home/user/ecat_stability_logs/
```

Each day uses one directory:

```bash
/home/user/ecat_stability_logs/YYYY-MM-DD/
```

Important files:

- `daily_summary.csv`: one row per experiment.
- `daily_totals.txt`: accumulated totals for that day.
- `daily_events.log`: compact OP enter/loss events.
- `esc_counters.log`: register snapshots before and after each experiment.

The snapshots contain AL status, ESC receive/lost-link counters, process data
watchdog count, and DC system-time difference for every configured slave. They
are taken while `ecat_main` is stopped so they do not add traffic during RUN.

Per-run detailed logs are disabled by default. Enable them only when diagnosing:

```bash
KEEP_DETAIL_LOGS=1
```

## Dry Run Without Shutdown

```bash
sudo SHUTDOWN_WHEN_DONE=0 DURATION_SEC=120 ./tools/ecat_stability_test.sh
```

## Finish Actions

The script uses `FINISH_ACTION`:

- `none`: write summary and keep running.
- `poweroff`: write summary and power off.
- `reboot`: write summary and reboot for the next round.

`MAX_RUNS_PER_DAY=48` means about one day of 30 minute rounds. After the 48th
run summary is written, the machine powers off instead of rebooting.

## Install 30 Minute Reboot-Loop Test

Use only when you are ready for automatic reboot testing. Disable `ecat_main.service`
first so this test script is the only thing that starts `ecat_main`.

```bash
sudo systemctl disable --now ecat_main.service
chmod +x tools/ecat_stability_test.sh
sudo install -m 0644 tools/ecat-stability-test.service /etc/systemd/system/ecat-stability-test.service
sudo systemctl daemon-reload
sudo systemctl enable ecat-stability-test.service
```

The installed service defaults to:

```bash
DURATION_SEC=1800
FINISH_ACTION=reboot
MAX_RUNS_PER_DAY=48
START_ECAT_MAIN=1
```

So each boot runs one 30 minute test, records the daily summary, then reboots.
After 48 runs in the same date directory, it powers off.

Disable it:

```bash
sudo systemctl disable --now ecat-stability-test.service
```

Stopping the service sends `SIGTERM`; the script writes its partial summary,
stops its child processes, and exits immediately without waiting for the full
test duration.

## Install ecat_main Service

This starts `ecat_main` at boot without opening a terminal window.

```bash
sudo install -m 0644 tools/ecat_main.service /etc/systemd/system/ecat_main.service
sudo systemctl daemon-reload
sudo systemctl enable ecat_main.service
```

Start/stop manually through systemd:

```bash
sudo systemctl start ecat_main.service
sudo systemctl stop ecat_main.service
sudo systemctl restart ecat_main.service
```

View output:

```bash
journalctl -u ecat_main.service -f
```

If output is still delayed, add flushing in `ecat_main.cpp`, for example:

```cpp
std::cout.setf(std::ios::unitbuf);
```

Systemd journal only captures output from the service-started process. If you run
`sudo ./ecat_main` manually, use that terminal to see its output.

## Stop ecat_main Before Updating

If service-managed:

```bash
sudo systemctl stop ecat_main.service
```

If manually started:

```bash
pgrep -af ecat_main
sudo pkill -TERM -f ecat_main
sleep 2
pgrep -af ecat_main
```

If it still exists:

```bash
sudo pkill -KILL -f ecat_main
```

For normal `ecat_main` binary updates, stopping `ecat_main` is enough. Do not stop
`ethercat.service` unless the lower-level EtherCAT master must be reloaded.

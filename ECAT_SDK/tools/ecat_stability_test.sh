#!/usr/bin/env bash
set -u

# EtherCAT stability boot test.
# Default output is analysis-friendly:
#   /home/user/ecat_stability_logs/YYYY-MM-DD/daily_summary.csv
#   /home/user/ecat_stability_logs/YYYY-MM-DD/daily_totals.txt
#
# Optional detailed per-run logs can be enabled with KEEP_DETAIL_LOGS=1.

DURATION_SEC="${DURATION_SEC:-1800}"
SAMPLE_INTERVAL_SEC="${SAMPLE_INTERVAL_SEC:-1}"
LOG_DIR="${LOG_DIR:-/home/user/ecat_stability_logs}"
ECAT_SERVICE="${ECAT_SERVICE:-ethercat.service}"
ECAT_MAIN_CMD="${ECAT_MAIN_CMD:-/home/user/ECAT_SDK/ecat_main}"
START_ECAT_MAIN="${START_ECAT_MAIN:-1}"
START_SCUT="${START_SCUT:-0}"
SCUT_WORKDIR="${SCUT_WORKDIR:-/home/user/scut1111230t/WCUTMMI_NET}"
SCUT_CMD="${SCUT_CMD:-./bin/SCUT}"
SCUT_START_DELAY_AFTER_OP_SEC="${SCUT_START_DELAY_AFTER_OP_SEC:-0}"
SHUTDOWN_WHEN_DONE="${SHUTDOWN_WHEN_DONE:-1}"
FINISH_ACTION="${FINISH_ACTION:-}"
MAX_RUNS_PER_DAY="${MAX_RUNS_PER_DAY:-0}"
KEEP_DETAIL_LOGS="${KEEP_DETAIL_LOGS:-0}"
DMESG_PATTERN="${DMESG_PATTERN:-ethercat|sync|synchronization|0x001a|0x002d|working counter|wkc|al status|did not sync}"

if [ -z "$FINISH_ACTION" ]; then
    if [ "$SHUTDOWN_WHEN_DONE" = "1" ]; then
        FINISH_ACTION="poweroff"
    else
        FINISH_ACTION="none"
    fi
fi

case "$FINISH_ACTION" in
    none|poweroff|reboot) ;;
    *)
        echo "Invalid FINISH_ACTION=$FINISH_ACTION; use none, poweroff, or reboot." >&2
        exit 2
        ;;
esac

RUN_ID="$(date '+%Y%m%d_%H%M%S')"
DAY_ID="$(date '+%F')"
DAY_DIR="$LOG_DIR/$DAY_ID"
RUN_DIR="$DAY_DIR/runs/$RUN_ID"
mkdir -p "$DAY_DIR"

DAILY_SUMMARY="$DAY_DIR/daily_summary.csv"
DAILY_TOTALS="$DAY_DIR/daily_totals.txt"
DAILY_EVENTS="$DAY_DIR/daily_events.log"
ESC_COUNTER_LOG="$DAY_DIR/esc_counters.log"

if [ "$KEEP_DETAIL_LOGS" = "1" ]; then
    mkdir -p "$RUN_DIR"
else
    RUN_DIR="$(mktemp -d /tmp/ecat_stability_${RUN_ID}.XXXXXX)"
fi

MAIN_LOG="$RUN_DIR/ecat_main.log"
SCUT_LOG="$RUN_DIR/scut.log"
SAMPLE_LOG="$RUN_DIR/samples.csv"
EVENT_LOG="$RUN_DIR/events.log"
DMESG_LOG="$RUN_DIR/dmesg_ethercat.log"
MASTER_LOG="$RUN_DIR/master_snapshots.log"
SLAVES_LOG="$RUN_DIR/slaves_snapshots.log"
SYSTEM_LOG="$RUN_DIR/system.log"

ecat_main_pid=""
scut_pid=""
scut_started=0
start_epoch="$(date +%s)"
last_state="UNKNOWN"
last_transition_epoch="$start_epoch"
first_all_op_epoch=""
op_enter_count=0
op_loss_count=0
sum_time_to_op=0
sum_op_duration=0
max_time_to_op=0
max_op_duration=0
first_op_loss_sec=""
last_op_loss_sec=""
op_loss_0_5min=0
op_loss_5_10min=0
op_loss_10_30min=0
scut_started_epoch=""
scut_started_sec=""
first_op_loss_after_scut_sec=""
not_op_samples=0
total_samples=0
last_not_op=""
last_dmesg_summary=""
summary_written=0

log_event() {
    local msg="$*"
    printf '[%s] %s\n' "$(date '+%F %T')" "$msg" | tee -a "$EVENT_LOG" >> "$DAILY_EVENTS"
}

snapshot_system() {
    {
        echo "===== $(date '+%F %T') ====="
        uptime || true
        free -m || true
        ps -eLo pid,tid,cls,rtprio,pri,psr,comm | egrep 'ecat_main|EtherCAT|irq/.*ens|irq/.*eth' || true
        grep -iE 'ens|eth|EtherCAT' /proc/interrupts || true
    } >> "$SYSTEM_LOG" 2>&1
}

snapshot_ethercat() {
    {
        echo "===== $(date '+%F %T') ====="
        ethercat master || true
    } >> "$MASTER_LOG" 2>&1

    {
        echo "===== $(date '+%F %T') ====="
        ethercat slaves || true
    } >> "$SLAVES_LOG" 2>&1
}

snapshot_esc_counters() {
    local label="$1"
    local slave_lines

    if pgrep -x ecat_main >/dev/null 2>&1; then
        log_event "ESC counter snapshot skipped ($label): ecat_main is running"
        return
    fi

    slave_lines="$(get_slave_state_line)"

    {
        printf '[%s] run_id=%s label=%s\n' "$(date '+%F %T')" "$RUN_ID" "$label"
        printf '%s\n' "$slave_lines" | awk 'NF > 0 {print $1, $2, $3}' |
        while read -r pos alias state; do
            [ -n "$pos" ] || continue
            printf 'slave=%s alias=%s state=%s ' "$pos" "$alias" "$state"
            printf 'al0130='; ethercat reg_read -p "$pos" 0x0130 6 2>/dev/null | sed 's/0x//g' | tr -d '[:space:]'
            printf ' esc0300='; ethercat reg_read -p "$pos" 0x0300 20 2>/dev/null | sed 's/0x//g' | tr -d '[:space:]'
            printf ' wd0442='; ethercat reg_read -p "$pos" 0x0442 1 2>/dev/null | sed 's/0x//g' | tr -d '[:space:]'
            printf ' dc092c='; ethercat reg_read -p "$pos" 0x092c 4 2>/dev/null | sed 's/0x//g' | tr -d '[:space:]'
            printf '\n'
        done
        printf '\n'
    } >> "$ESC_COUNTER_LOG" 2>&1
}

start_scut_once() {
    if [ "$START_SCUT" != "1" ] || [ "$scut_started" = "1" ]; then
        return
    fi

    if pgrep -f '(^|/)SCUT($| )|(^|/)bin/SCUT($| )' >/dev/null 2>&1; then
        scut_started=1
        log_event "SCUT already running; monitor only"
        return
    fi

    if [ ! -d "$SCUT_WORKDIR" ]; then
        log_event "SCUT workdir missing: $SCUT_WORKDIR"
        scut_started=1
        return
    fi

    log_event "starting SCUT after OP: cd $SCUT_WORKDIR && $SCUT_CMD"
    if [ "$SCUT_START_DELAY_AFTER_OP_SEC" -gt 0 ]; then
        sleep "$SCUT_START_DELAY_AFTER_OP_SEC"
    fi

    (
        cd "$SCUT_WORKDIR" || exit 1
        if command -v stdbuf >/dev/null 2>&1; then
            stdbuf -oL -eL $SCUT_CMD
        else
            $SCUT_CMD
        fi
    ) >> "$SCUT_LOG" 2>&1 &
    scut_pid="$!"
    scut_started=1
    scut_started_epoch="$(date +%s)"
    scut_started_sec=$((scut_started_epoch - start_epoch))
    log_event "SCUT pid=$scut_pid"
}

stop_scut_if_started() {
    if [ -n "$scut_pid" ] && kill -0 "$scut_pid" 2>/dev/null; then
        log_event "stopping SCUT pid=$scut_pid"
        kill -TERM "$scut_pid" 2>/dev/null || true
        sleep 3
        kill -KILL "$scut_pid" 2>/dev/null || true
    elif [ "$START_SCUT" = "1" ]; then
        if pgrep -f '(^|/)SCUT($| )|(^|/)bin/SCUT($| )' >/dev/null 2>&1; then
            log_event "stopping existing SCUT process"
            pkill -TERM -f '(^|/)SCUT($| )|(^|/)bin/SCUT($| )' || true
            sleep 3
            pkill -KILL -f '(^|/)SCUT($| )|(^|/)bin/SCUT($| )' || true
        fi
    fi
}

get_slave_state_line() {
    ethercat slaves 2>/dev/null || true
}

all_slaves_op() {
    local lines="$1"
    local count
    count="$(printf '%s\n' "$lines" | awk 'NF > 0 {n++} END {print n+0}')"
    if [ "$count" -eq 0 ]; then
        return 1
    fi

    printf '%s\n' "$lines" | awk '
        NF > 0 {
            if ($3 != "OP") bad=1
        }
        END {
            exit bad ? 1 : 0
        }'
}

not_op_summary() {
    local lines="$1"
    printf '%s\n' "$lines" | awk 'NF > 0 && $3 != "OP" {printf "%s:%s:%s ", $1, $2, $3}'
}

refresh_dmesg_summary() {
    sudo dmesg -T 2>/dev/null | grep -iE "$DMESG_PATTERN" > "$DMESG_LOG" || true
    local sync_001a sync_002d did_not_sync working_counter
    sync_001a="$(grep -ic '0x001a' "$DMESG_LOG" 2>/dev/null || true)"
    sync_002d="$(grep -ic '0x002d' "$DMESG_LOG" 2>/dev/null || true)"
    did_not_sync="$(grep -ic 'did not sync' "$DMESG_LOG" 2>/dev/null || true)"
    working_counter="$(grep -ic 'working counter' "$DMESG_LOG" 2>/dev/null || true)"
    last_dmesg_summary="0x001A=${sync_001a};0x002D=${sync_002d};did_not_sync=${did_not_sync};working_counter=${working_counter};"
}

ensure_daily_header() {
    if [ ! -f "$DAILY_SUMMARY" ]; then
        echo "run_id,date,start_time,end_time,elapsed_sec,final_state,first_all_op_sec,op_enter_count,op_loss_count,avg_time_to_op_sec,max_time_to_op_sec,avg_op_duration_before_loss_sec,max_op_duration_before_loss_sec,first_op_loss_sec,last_op_loss_sec,op_loss_0_5min,op_loss_5_10min,op_loss_10_30min,scut_started_sec,first_op_loss_after_scut_sec,total_samples,not_op_samples,last_not_op,dmesg_summary" > "$DAILY_SUMMARY"
    fi
}

csv_escape() {
    printf '%s' "$1" | sed 's/"/""/g'
}

append_run_summary() {
    local end_epoch elapsed avg_time_to_op avg_op_duration first_all_op_sec final_state
    end_epoch="$(date +%s)"
    elapsed=$((end_epoch - start_epoch))
    final_state="$last_state"

    if [ "$op_enter_count" -gt 0 ]; then
        avg_time_to_op=$((sum_time_to_op / op_enter_count))
    else
        avg_time_to_op=0
    fi

    if [ "$op_loss_count" -gt 0 ]; then
        avg_op_duration=$((sum_op_duration / op_loss_count))
    else
        avg_op_duration=0
    fi

    if [ -n "$first_all_op_epoch" ]; then
        first_all_op_sec=$((first_all_op_epoch - start_epoch))
    else
        first_all_op_sec="never"
    fi

    ensure_daily_header
    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,"%s","%s"\n' \
        "$RUN_ID" \
        "$DAY_ID" \
        "$(date -d "@$start_epoch" '+%F %T' 2>/dev/null || date '+%F %T')" \
        "$(date '+%F %T')" \
        "$elapsed" \
        "$final_state" \
        "$first_all_op_sec" \
        "$op_enter_count" \
        "$op_loss_count" \
        "$avg_time_to_op" \
        "$max_time_to_op" \
        "$avg_op_duration" \
        "$max_op_duration" \
        "${first_op_loss_sec:-none}" \
        "${last_op_loss_sec:-none}" \
        "$op_loss_0_5min" \
        "$op_loss_5_10min" \
        "$op_loss_10_30min" \
        "${scut_started_sec:-none}" \
        "${first_op_loss_after_scut_sec:-none}" \
        "$total_samples" \
        "$not_op_samples" \
        "$(csv_escape "$last_not_op")" \
        "$(csv_escape "$last_dmesg_summary")" >> "$DAILY_SUMMARY"
}

write_daily_totals() {
    if [ ! -f "$DAILY_SUMMARY" ]; then
        return
    fi

    awk -F',' '
        NR == 1 {next}
        {
            runs++
            op_enter += $8
            op_loss += $9
            loss_0_5 += $16
            loss_5_10 += $17
            loss_10_30 += $18
            total_samples += $21
            not_op_samples += $22
            if ($7 != "never") {
                first_op_runs++
                first_op_sum += $7
                if ($7 > first_op_max) first_op_max = $7
            }
            if ($14 != "none") {
                first_loss_runs++
                first_loss_sum += $14
                if ($14 > first_loss_max) first_loss_max = $14
            }
            if ($9 > 0) {
                loss_runs++
                op_duration_sum += $12
                if ($13 > op_duration_max) op_duration_max = $13
            }
        }
        END {
            print "date='"$DAY_ID"'"
            print "experiment_count=" runs+0
            print "op_enter_total=" op_enter+0
            print "op_loss_total=" op_loss+0
            print "runs_with_op_loss=" loss_runs+0
            print "op_loss_0_5min_total=" loss_0_5+0
            print "op_loss_5_10min_total=" loss_5_10+0
            print "op_loss_10_30min_total=" loss_10_30+0
            print "total_samples=" total_samples+0
            print "not_op_samples=" not_op_samples+0
            if (first_op_runs > 0) {
                print "avg_first_all_op_sec=" int(first_op_sum / first_op_runs)
                print "max_first_all_op_sec=" first_op_max+0
            } else {
                print "avg_first_all_op_sec=never"
                print "max_first_all_op_sec=never"
            }
            if (first_loss_runs > 0) {
                print "avg_first_op_loss_sec=" int(first_loss_sum / first_loss_runs)
                print "max_first_op_loss_sec=" first_loss_max+0
            } else {
                print "avg_first_op_loss_sec=none"
                print "max_first_op_loss_sec=none"
            }
            if (loss_runs > 0) {
                print "avg_op_duration_before_loss_sec=" int(op_duration_sum / loss_runs)
                print "max_op_duration_before_loss_sec=" op_duration_max+0
            } else {
                print "avg_op_duration_before_loss_sec=none"
                print "max_op_duration_before_loss_sec=none"
            }
        }
    ' "$DAILY_SUMMARY" > "$DAILY_TOTALS"
}

daily_experiment_count() {
    if [ ! -f "$DAILY_SUMMARY" ]; then
        echo 0
        return
    fi
    awk 'NR > 1 {n++} END {print n+0}' "$DAILY_SUMMARY"
}

finish_summary_once() {
    if [ "$summary_written" = "1" ]; then
        return
    fi
    refresh_dmesg_summary
    append_run_summary
    write_daily_totals
    summary_written=1
}

cleanup() {
    log_event "cleanup"
    stop_scut_if_started
    snapshot_system
    snapshot_ethercat
    finish_summary_once

    if [ -n "$ecat_main_pid" ] && kill -0 "$ecat_main_pid" 2>/dev/null; then
        log_event "stopping ecat_main pid=$ecat_main_pid"
        kill -TERM "$ecat_main_pid" 2>/dev/null || true
        sleep 2
        kill -KILL "$ecat_main_pid" 2>/dev/null || true
        wait "$ecat_main_pid" 2>/dev/null || true
    fi
    snapshot_esc_counters "end"

    if [ "$KEEP_DETAIL_LOGS" != "1" ]; then
        rm -rf "$RUN_DIR"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 0' TERM

log_event "test start run_id=$RUN_ID keep_detail=$KEEP_DETAIL_LOGS"
log_event "config duration=${DURATION_SEC}s finish_action=$FINISH_ACTION max_runs_per_day=$MAX_RUNS_PER_DAY start_scut=$START_SCUT"
snapshot_system

log_event "starting $ECAT_SERVICE"
systemctl start "$ECAT_SERVICE" >> "$SYSTEM_LOG" 2>&1 || log_event "failed to start $ECAT_SERVICE"
sleep 2
snapshot_esc_counters "start"

if [ "$START_ECAT_MAIN" = "1" ]; then
    if pgrep -f "$ECAT_MAIN_CMD" >/dev/null 2>&1 || pgrep -f 'ecat_main' >/dev/null 2>&1; then
        log_event "ecat_main already running; monitor only"
    else
        log_event "starting ecat_main: $ECAT_MAIN_CMD"
        if command -v stdbuf >/dev/null 2>&1; then
            stdbuf -oL -eL "$ECAT_MAIN_CMD" >> "$MAIN_LOG" 2>&1 &
        else
            "$ECAT_MAIN_CMD" >> "$MAIN_LOG" 2>&1 &
        fi
        ecat_main_pid="$!"
        log_event "ecat_main pid=$ecat_main_pid"
    fi
fi

echo "time,elapsed_sec,state,op_count,total_count,not_op" > "$SAMPLE_LOG"
snapshot_ethercat

while true; do
    now_epoch="$(date +%s)"
    elapsed=$((now_epoch - start_epoch))
    if [ "$elapsed" -ge "$DURATION_SEC" ]; then
        log_event "duration reached: ${elapsed}s"
        break
    fi

    slaves="$(get_slave_state_line)"
    total_count="$(printf '%s\n' "$slaves" | awk 'NF > 0 {n++} END {print n+0}')"
    op_count="$(printf '%s\n' "$slaves" | awk 'NF > 0 && $3 == "OP" {n++} END {print n+0}')"
    total_samples=$((total_samples + 1))

    if all_slaves_op "$slaves"; then
        state="ALL_OP"
    else
        state="NOT_OP"
        not_op_samples=$((not_op_samples + 1))
    fi

    not_op="$(not_op_summary "$slaves")"
    if [ -n "$not_op" ]; then
        last_not_op="$not_op"
    fi
    printf '%s,%s,%s,%s,%s,"%s"\n' "$(date '+%F %T')" "$elapsed" "$state" "$op_count" "$total_count" "$not_op" >> "$SAMPLE_LOG"

    if [ "$state" != "$last_state" ]; then
        if [ "$state" = "ALL_OP" ]; then
            delay=$((now_epoch - last_transition_epoch))
            op_enter_count=$((op_enter_count + 1))
            sum_time_to_op=$((sum_time_to_op + delay))
            [ "$delay" -gt "$max_time_to_op" ] && max_time_to_op="$delay"
            [ -z "$first_all_op_epoch" ] && first_all_op_epoch="$now_epoch"
            log_event "ENTER_OP delay_sec=$delay op_count=$op_count/$total_count"
            start_scut_once
        elif [ "$last_state" = "ALL_OP" ]; then
            duration=$((now_epoch - last_transition_epoch))
            loss_sec=$((now_epoch - start_epoch))
            op_loss_count=$((op_loss_count + 1))
            sum_op_duration=$((sum_op_duration + duration))
            [ "$duration" -gt "$max_op_duration" ] && max_op_duration="$duration"
            [ -z "$first_op_loss_sec" ] && first_op_loss_sec="$loss_sec"
            last_op_loss_sec="$loss_sec"
            if [ "$loss_sec" -lt 300 ]; then
                op_loss_0_5min=$((op_loss_0_5min + 1))
            elif [ "$loss_sec" -lt 600 ]; then
                op_loss_5_10min=$((op_loss_5_10min + 1))
            elif [ "$loss_sec" -lt 1800 ]; then
                op_loss_10_30min=$((op_loss_10_30min + 1))
            fi
            if [ -n "$scut_started_epoch" ] && [ -z "$first_op_loss_after_scut_sec" ]; then
                first_op_loss_after_scut_sec=$((now_epoch - scut_started_epoch))
            fi
            log_event "LEAVE_OP op_duration_sec=$duration op_count=$op_count/$total_count not_op=$not_op"
            snapshot_ethercat
            snapshot_system
        else
            log_event "STATE $last_state -> $state op_count=$op_count/$total_count not_op=$not_op"
        fi
        last_state="$state"
        last_transition_epoch="$now_epoch"
    fi

    if [ $((elapsed % 60)) -eq 0 ]; then
        snapshot_system
    fi

    sleep "$SAMPLE_INTERVAL_SEC"
done

log_event "test finished"

finish_summary_once

# Finish diagnostics and stop child processes before reboot/poweroff starts.
# The EXIT trap remains a fallback for interrupted or failed runs.
cleanup
trap - EXIT

runs_today="$(daily_experiment_count)"
if [ "$MAX_RUNS_PER_DAY" -gt 0 ] && [ "$runs_today" -ge "$MAX_RUNS_PER_DAY" ]; then
    log_event "max runs reached: $runs_today/$MAX_RUNS_PER_DAY; poweroff requested"
    sync
    /sbin/poweroff
fi

case "$FINISH_ACTION" in
    none)
        log_event "finish action: none"
        ;;
    poweroff)
        log_event "finish action: poweroff"
        sync
        /sbin/poweroff
        ;;
    reboot)
        log_event "finish action: reboot"
        sync
        /sbin/reboot
        ;;
esac

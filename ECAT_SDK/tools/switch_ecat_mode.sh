#!/usr/bin/env bash
set -euo pipefail

# Quick mode switch for the ECAT test machine.
#
# Usage:
#   sudo ./switch_ecat_mode.sh status
#   sudo ./switch_ecat_mode.sh standard
#   sudo ./switch_ecat_mode.sh loop-test
#   sudo ./switch_ecat_mode.sh stop-all

ECAT_DIR="${ECAT_DIR:-/home/user/ECAT_SDK}"
ECAT_MAIN_SERVICE_SRC="$ECAT_DIR/tools/ecat_main.service"
STABILITY_SERVICE_SRC="$ECAT_DIR/tools/ecat-stability-test.service"
STABILITY_SCRIPT="$ECAT_DIR/tools/ecat_stability_test.sh"

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "Please run with sudo." >&2
        exit 1
    fi
}

service_value() {
    local kind="$1"
    local service="$2"
    local value
    value="$(systemctl "$kind" "$service" 2>/dev/null || true)"
    if [ -z "$value" ]; then
        value="not-installed"
    fi
    printf '%s' "$value"
}

install_services() {
    if [ ! -f "$ECAT_MAIN_SERVICE_SRC" ]; then
        echo "Missing $ECAT_MAIN_SERVICE_SRC" >&2
        exit 1
    fi
    if [ ! -f "$STABILITY_SERVICE_SRC" ]; then
        echo "Missing $STABILITY_SERVICE_SRC" >&2
        exit 1
    fi
    if [ ! -f "$STABILITY_SCRIPT" ]; then
        echo "Missing $STABILITY_SCRIPT" >&2
        exit 1
    fi

    chmod +x "$STABILITY_SCRIPT"
    install -m 0644 "$ECAT_MAIN_SERVICE_SRC" /etc/systemd/system/ecat_main.service
    install -m 0644 "$STABILITY_SERVICE_SRC" /etc/systemd/system/ecat-stability-test.service
    systemctl daemon-reload
}

kill_manual_ecat_main() {
    if pgrep -f '(^|/)ecat_main($| )' >/dev/null 2>&1; then
        pkill -TERM -f '(^|/)ecat_main($| )' || true
        sleep 2
        pkill -KILL -f '(^|/)ecat_main($| )' || true
    fi
}

show_status() {
    echo "===== mode services ====="
    echo "ecat_main enabled: $(service_value is-enabled ecat_main.service)"
    echo "ecat_main active:  $(service_value is-active ecat_main.service)"
    echo "test enabled:      $(service_value is-enabled ecat-stability-test.service)"
    echo "test active:       $(service_value is-active ecat-stability-test.service)"

    echo
    echo "===== processes ====="
    pgrep -af '(^|/)ecat_main($| )' || true

    echo
    echo "===== network ====="
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev status 2>/dev/null || true

    echo
    echo "===== latest stability totals ====="
    today="$(date +%F)"
    totals="/home/user/ecat_stability_logs/$today/daily_totals.txt"
    if [ -f "$totals" ]; then
        cat "$totals"
    else
        echo "No totals for $today"
    fi
}

mode_standard() {
    require_root
    install_services

    systemctl disable --now ecat-stability-test.service >/dev/null 2>&1 || true
    kill_manual_ecat_main
    systemctl enable ecat_main.service
    systemctl restart ecat_main.service

    echo "Switched to STANDARD mode."
    echo "ecat_main.service is enabled and running."
    echo "View log: journalctl -u ecat_main.service -f"
}

mode_loop_test() {
    require_root
    install_services

    systemctl disable --now ecat_main.service >/dev/null 2>&1 || true
    kill_manual_ecat_main
    systemctl enable ecat-stability-test.service

    echo "Switched to LOOP-TEST mode."
    echo "ecat_main.service is disabled."
    echo "ecat-stability-test.service is enabled."
    echo
    echo "Next boot will run one 30 minute test, write daily summary, then reboot."
    echo "After MAX_RUNS_PER_DAY is reached, the machine will power off."
    echo
    echo "Start now with: sudo systemctl start ecat-stability-test.service"
    echo "Or reboot now with: sudo reboot"
}

mode_stop_all() {
    require_root
    systemctl disable --now ecat-stability-test.service >/dev/null 2>&1 || true
    systemctl disable --now ecat_main.service >/dev/null 2>&1 || true
    kill_manual_ecat_main
    echo "Stopped and disabled ecat_main.service and ecat-stability-test.service."
}

case "${1:-}" in
    status)
        show_status
        ;;
    standard)
        mode_standard
        show_status
        ;;
    loop-test|loop)
        mode_loop_test
        show_status
        ;;
    stop-all|stop)
        mode_stop_all
        show_status
        ;;
    *)
        echo "Usage: sudo $0 {status|standard|loop-test|stop-all}" >&2
        exit 2
        ;;
esac

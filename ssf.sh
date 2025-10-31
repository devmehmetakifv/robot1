#!/usr/bin/env bash

set -euo pipefail



USER_NAME="$(whoami)"

HOST_NAME="$(hostname)"

MACHINE_ID="$(cat /etc/machine-id 2>/dev/null || echo N/A)"

PRODUCT_UUID="$(cat /sys/class/dmi/id/product_uuid 2>/dev/null || echo N/A)"





MACS="$(

 for p in /sys/class/net/*; do

  IF="$(basename "$p")"

  case "$IF" in lo|docker*|veth*|br*|virbr*|zt* ) continue ;; esac

  [ -f "$p/address" ] && cat "$p/address"

 done | awk 'NF' | paste -sd, -

)"



NOW_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"



read -r -p "Ogrenci Numaraniz: " STUDENT_ID



RAW="${STUDENT_ID}|${USER_NAME}|${HOST_NAME}|${MACHINE_ID}|${PRODUCT_UUID}|${MACS}|${NOW_UTC}"





HASH="$(printf "%s" "$RAW" | sha256sum | awk '{print $1}')"



printf "\n===== SSF RAW =====\n%s\n" "$RAW"

printf "\n===== SSF SHA256 =====\n%s\n" "$HASH"



printf "%s\n" "$HASH" > "$HOME/SSF_HASH.txt"

echo "Kaydedildi: $HOME/SSF_HASH.txt"

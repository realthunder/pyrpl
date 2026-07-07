#!/usr/bin/env bash
# POWER_OPT fit/timing sweep for fft125ssr4n11.
#
# power_opt_design's clock gating adds control sets (2794) and overflows slices
# by ~2% under the profile's DEFAULT placer DET=7 (AltSpreadLogic_medium — a
# SPREADING directive that intentionally uses more slices). This sweep tries
# DENSE place directives (the fit lever; phys_opt runs after place and can't fix
# a place-stage overflow) to find a combo that both FITS and closes timing with
# POWER_OPT=1. Each run is isolated in .sweep_pow/<tag>; HLS is shared via .hls.
#
#   DETS='8 1 4 6' PHYSES='AggressiveExplore' PARALLEL=2 ./sweep_powopt.sh
#
# DET index -> place_design directive (red_pitaya_vivado.tcl):
#   1 Explore  2 ExtraNetDelay_high  3 AltSpreadLogic_high  4 WLDrivenBlockPlacement
#   5 ExtraPostPlacementOpt  6 EarlyBlockPlacement  7 AltSpreadLogic_medium
#   8 Default  9 ExtraTimingOpt  10 ExtraNetDelay_low  11 AltSpreadLogic_low
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; cd "$ROOT"

PARALLEL=${PARALLEL:-2}
MIN_FREE_GB=${MIN_FREE_GB:-6}
read -ra DETS   <<< "${DETS:-8 1 4 6}"          # dense placers first
read -ra PHYSES <<< "${PHYSES:-AggressiveExplore}"
SUMMARY="$ROOT/sweep_powopt_summary.txt"

log(){ echo "[$(date '+%T')] $*" | tee -a "$SUMMARY"; }
avail_gb(){ awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
running(){ jobs -rp | wc -l; }
wait_slot(){ while (( $(running) >= PARALLEL )) || (( $(avail_gb) < MIN_FREE_GB )); do sleep 15; done; }

run_one(){  # $1=det  $2=phys
    local det=$1 phys=$2 tag="det${1}-${2}" wd="$ROOT/.sweep_pow/det${1}-${2}"
    local blog="$ROOT/build_pow_${tag}.log"
    rm -rf "$wd"; mkdir -p "$wd"
    BUILD_DIR="$wd" PROFILE=fft125ssr4n11 POWER_OPT=1 \
        DETERMINISTIC="$det" PHYS_OPT="$phys" ./make.sh > "$blog" 2>&1
    local ts="$wd/out/post_route_timing_summary.rpt"
    if grep -qiE "could not place all instances|place_design failed|packing of instances" "$blog"; then
        local over=$(grep -oE "require [0-9]+ slices" "$blog" | tail -1)
        log "RESULT $tag: FIT-FAIL ($over)"
    elif [[ -f "$ts" ]]; then
        # first numeric row under the Design Timing Summary = WNS TNS ... WHS ...
        local row=$(awk '/Design Timing Summary/{g=1} g&&/^ *-?[0-9]/{print;exit}' "$ts")
        local wns=$(awk '{print $1}' <<<"$row"); local whs=$(awk '{print $5}' <<<"$row")
        local bit="no-bit"; ls "$wd"/out/*.bit >/dev/null 2>&1 && bit="BIT"
        log "RESULT $tag: WNS=$wns WHS=$whs $bit"
    else
        log "RESULT $tag: INCOMPLETE (see $blog)"
    fi
}

log "== POWER_OPT sweep fft125ssr4n11: DET={${DETS[*]}} x PHYS={${PHYSES[*]}} parallel=$PARALLEL =="
for det in "${DETS[@]}"; do
    for phys in "${PHYSES[@]}"; do
        wait_slot
        log "launch det${det} x ${phys}  (avail $(avail_gb)GB, running $(running))"
        run_one "$det" "$phys" &
        sleep 5
    done
done
wait
log "== sweep done =="

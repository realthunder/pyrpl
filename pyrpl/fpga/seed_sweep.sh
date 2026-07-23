#!/usr/bin/env bash
# Parallel placement-directive sweep to close a given FFT/clock configuration.
# Each build runs single-threaded (reproducible) in its own BUILD_DIR=.sweep/detN
# so several run concurrently. Concurrency = min(PARALLEL_SWEEP, free-memory budget).
#
#   SWEEP_FFT       FFT/clock env settings to sweep   (default: SSR=2/N13/200MHz)
#   SWEEP_LABEL     label for result/archive dirs     (default: ssr2n13-200)
#   PARALLEL_SWEEP  max concurrent builds             (default 2)
#   MIN_FREE_GB     don't launch below this free RAM  (default 8)
set -u
cd "$(dirname "$0")"
ROOT=$(pwd)

PARALLEL_SWEEP=${PARALLEL_SWEEP:-2}
MIN_FREE_GB=${MIN_FREE_GB:-8}
FFTENV="${SWEEP_FFT:-FFT_SSR=2 FFT_NFFT=13 FFT_CLK_200=1 FFT_CLK_SEL=1}"
LABEL="${SWEEP_LABEL:-ssr2n13-200}"
# must match red_pitaya_vivado.tcl det_place_dirs (1-indexed by DETERMINISTIC)
DIRS=(Explore ExtraNetDelay_high AltSpreadLogic_high WLDrivenBlockPlacement ExtraPostPlacementOpt EarlyBlockPlacement AltSpreadLogic_medium Default ExtraTimingOpt ExtraNetDelay_low AltSpreadLogic_low)

RESULTS=sweep_results.txt
: > "$RESULTS"
log() { echo "$@" | tee -a "$RESULTS"; }
log "parallel directive sweep start $(date '+%F %T')  PARALLEL_SWEEP=$PARALLEL_SWEEP MIN_FREE_GB=$MIN_FREE_GB"
log "label: $LABEL    config: $FFTENV"

# Pre-warm the shared HLS cache once (workers symlink $ROOT/.hls and skip HLS,
# so they never race on a cold cache).
log "== pre-warming HLS =="
if ! env $FFTENV ./make.sh hls > build_hls_prewarm.log 2>&1; then
    log "HLS PREWARM FAILED — see build_hls_prewarm.log"; exit 1
fi
log "HLS ready ($(grep -c 'CSim done with 0 errors' build_hls_prewarm.log) csim-pass this run; cached if 0)"

avail_gb() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
running()  { jobs -rp | wc -l; }
wait_slot() {
    while (( $(running) >= PARALLEL_SWEEP )) || (( $(avail_gb) < MIN_FREE_GB )); do
        sleep 15
    done
}

run_one() {  # $1=n
    local n=$1 wd="$ROOT/.sweep/det${n}"
    rm -rf "$wd"; mkdir -p "$wd"
    BUILD_DIR="$wd" DETERMINISTIC="$n" env $FFTENV ./make.sh > "build_det${n}.log" 2>&1
}

# SWEEP_SEEDS: space-separated DETERMINISTIC values to sweep (default: all 11).
SEEDS=(${SWEEP_SEEDS:-$(seq 1 ${#DIRS[@]})})

for n in "${SEEDS[@]}"; do
    wait_slot
    log "== launch DET=$n (${DIRS[$((n-1))]}) at $(date '+%T')  avail=$(avail_gb)GB running=$(running) =="
    run_one "$n" &
    sleep 5   # small stagger so concurrent Vivado launches don't hit licensing/setup at once
done
wait
log "==== all builds done $(date '+%T') — collecting =="

extract() { awk -v c="$1" '$1==c && $2 ~ /^-?[0-9]+\.[0-9]+$/{print $2; exit}' "$2" 2>/dev/null; }
for n in "${SEEDS[@]}"; do
    dir=${DIRS[$((n-1))]}; rpt="$ROOT/.sweep/det${n}/out/post_route_timing_summary.rpt"
    if [[ -f "$rpt" ]]; then
        adc=$(extract pll_adc_clk "$rpt"); ser=$(extract pll_ser_clk "$rpt")
        worst=$(awk -v a="$adc" -v s="$ser" 'BEGIN{print (a<s)?a:s}')
        rm -rf "$ROOT/out.d/sweep-${LABEL}-det${n}-${dir}"
        cp -r "$ROOT/.sweep/det${n}/out" "$ROOT/out.d/sweep-${LABEL}-det${n}-${dir}" 2>/dev/null
        log "DET=$n ($dir) : adc=$adc ser=$ser worst=$worst"
    else
        log "DET=$n ($dir) : NO TIMING — see build_det${n}.log"
    fi
done
log "ranked (best worst-slack last):"
grep -E "worst=-?[0-9]" "$RESULTS" | sed -E 's/.*worst=([-0-9.]+).*/\1 &/' | sort -g | sed -E 's/^[-0-9.]+ //' | tee -a "$RESULTS"

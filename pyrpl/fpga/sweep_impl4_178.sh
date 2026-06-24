#!/usr/bin/env bash
# 11x3 place-directive x phys_opt sweep to close IMPL=4 SSR=4 N11 @178.57 MHz.
# Only the shared ADC recurrence (i_dsp/sum2 -> adc_scht, pll_adc_clk) is short
# (-0.102); the FFT (pll_ser_clk @178) already closes (+0.100). Each build is
# single-threaded/reproducible in its own BUILD_DIR=.sweep/<tag> so many run
# concurrently. HLS is pre-warmed once (workers symlink $ROOT/.hls, skip HLS).
set -u
cd "$(dirname "$0")"
ROOT=$(pwd)

PARALLEL_SWEEP=${PARALLEL_SWEEP:-8}
MIN_FREE_GB=${MIN_FREE_GB:-10}
FFTENV="FFT_IMPL=4 FFT_SSR=4 FFT_NFFT=11 FFT_WIDTH=24 FFT_USE_APPROX=1 FFT_CLK_SEL=0 SUM1_REPLICATE=0"
LABEL="impl4-ssr4n11-125-dsz24"

# 11 place directives — must match red_pitaya_vivado.tcl / make.sh det_dirs (1-indexed).
DIRS=(Explore ExtraNetDelay_high AltSpreadLogic_high WLDrivenBlockPlacement \
      ExtraPostPlacementOpt EarlyBlockPlacement AltSpreadLogic_medium Default \
      ExtraTimingOpt ExtraNetDelay_low AltSpreadLogic_low)
# 3 phys_opt directives (same set as the prior ssr2n13 sweep).
PHYS=(Explore AggressiveExplore AggressiveFanoutOpt)

RESULTS=sweep_impl4_178_results.txt
: > "$RESULTS"
log() { echo "$@" | tee -a "$RESULTS"; }
log "11x3 sweep start $(date '+%F %T')  PARALLEL=$PARALLEL_SWEEP MIN_FREE_GB=$MIN_FREE_GB"
log "label: $LABEL    config: $FFTENV"

# Wipe any prior .sweep build dirs so a partially-run sweep can never leave stale
# results from a different config (e.g. an earlier DSZ) to be misread mid-run.
log "== cleaning .sweep =="
rm -rf "$ROOT/.sweep"

log "== pre-warming HLS =="
if ! env $FFTENV ./make.sh hls > build_hls_prewarm.log 2>&1; then
    log "HLS PREWARM FAILED — see build_hls_prewarm.log"; exit 1
fi
log "HLS ready"

avail_gb() { awk '/MemAvailable/{print int($2/1048576)}' /proc/meminfo; }
running()  { jobs -rp | wc -l; }
wait_slot() {
    while (( $(running) >= PARALLEL_SWEEP )) || (( $(avail_gb) < MIN_FREE_GB )); do
        sleep 15
    done
}

run_one() {  # $1=det index   $2=phys directive
    local n=$1 phys=$2 tag="det${1}-${2}" wd
    wd="$ROOT/.sweep/${tag}"
    rm -rf "$wd"; mkdir -p "$wd"
    BUILD_DIR="$wd" DETERMINISTIC="$n" PHYS_OPT="$phys" env $FFTENV ./make.sh \
        > "build_${tag}.log" 2>&1
}

for n in $(seq 1 ${#DIRS[@]}); do
    for phys in "${PHYS[@]}"; do
        wait_slot
        log "== launch det${n}(${DIRS[$((n-1))]}) x ${phys} at $(date '+%T') avail=$(avail_gb)GB running=$(running) =="
        run_one "$n" "$phys" &
        sleep 5
    done
done
wait
log "==== all builds done $(date '+%T') — collecting =="

extract() { awk -v c="$1" '$1==c && $2 ~ /^-?[0-9]+\.[0-9]+$/{print $2; exit}' "$2" 2>/dev/null; }
for n in $(seq 1 ${#DIRS[@]}); do
    dir=${DIRS[$((n-1))]}
    for phys in "${PHYS[@]}"; do
        tag="det${n}-${phys}"
        rpt="$ROOT/.sweep/${tag}/out/post_route_timing_summary.rpt"
        if [[ -f "$rpt" ]]; then
            adc=$(extract pll_adc_clk "$rpt"); ser=$(extract pll_ser_clk "$rpt")
            worst=$(awk -v a="$adc" -v s="$ser" 'BEGIN{print (a<s)?a:s}')
            rm -rf "$ROOT/out.d/sweep-${LABEL}-${tag}"
            cp -r "$ROOT/.sweep/${tag}/out" "$ROOT/out.d/sweep-${LABEL}-${tag}" 2>/dev/null
            log "$tag ($dir x $phys) : adc=$adc ser=$ser worst=$worst"
        else
            log "$tag ($dir x $phys) : NO TIMING — see build_${tag}.log"
        fi
    done
done
log "ranked (best worst-slack last):"
grep -E "worst=-?[0-9]" "$RESULTS" | sed -E 's/.*worst=([-0-9.]+).*/\1 &/' | sort -g | sed -E 's/^[-0-9.]+ //' | tee -a "$RESULTS"
log "==== sweep complete $(date '+%F %T') ===="

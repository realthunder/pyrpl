rm -rf out .Xil .srcs sdk

TA_PATH=/opt/Xilinx
export XILINX_VITIS=${TA_PATH}/Vitis/2020.1
export XILINX_VIVADO=${TA_PATH}/Vivado/2020.1
source ${XILINX_VIVADO}/settings64.sh

HLS=${XILINX_VIVADO}/bin/vivado_hls
VIVADO=${XILINX_VIVADO}/bin/vivado

# FFT_IMPL selects the FFT back-end:
#   1 = plain LogiCORE  (IMPL==1, legacy)
#   2 = HLS SSR via Vitis xf::dsp (IMPL==2, default when unset)
#   3 = IP SSR: HLS pre/post + LogiCORE sub-FFTs in BD (IMPL==3)
: "${FFT_IMPL:=2}"

if [ "$1" = "hls" ]; then
    if [ "$FFT_IMPL" = "3" ]; then
        # Build pre then post.  pre generates the twiddle LUT header first.
        $HLS -f hls/fft_ip_ssr_pre.tcl
        $HLS -f hls/fft_ip_ssr_post.tcl
    else
        $HLS -f hls/fft_ssr.tcl
    fi
elif [ "$1" = "bd" ]; then
    # Build BD wiring only (HLS IPs must already be built).
    if [ "$FFT_IMPL" = "3" ]; then
        $VIVADO -nolog -nojournal -mode tcl \
            -source ip/fft_ip_ssr_bd.tcl -tclargs $@
    else
        echo "bd target only applicable for FFT_IMPL=3"
        exit 1
    fi
else
    script=$1
    if [ -z "$script" ]; then
        script=red_pitaya_vivado.tcl
    else
        shift
    fi

    $VIVADO -nolog -nojournal -mode tcl -source "$script" -tclargs $@

    echo compilation finished
fi

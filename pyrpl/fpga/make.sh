rm -rf out .Xil .srcs sdk

TA_PATH=/opt/Xilinx
export XILINX_VITIS=${TA_PATH}/Vitis/2020.1
export XILINX_VIVADO=${TA_PATH}/Vivado/2020.1
source ${XILINX_VIVADO}/settings64.sh

if [ "$1" = "hls" ]; then
    ${XILINX_VIVADO}/bin/vivado_hls -f hls/fft_ssr.tcl
else
    script=$1
    if [ -z $script ]; then
    script=red_pitaya_vivado.tcl
    else
    shift
    fi

    ${XILINX_VIVADO}/bin/vivado -nolog -nojournal -mode tcl -source "$script" -tclargs $@

    echo compilation finished
fi

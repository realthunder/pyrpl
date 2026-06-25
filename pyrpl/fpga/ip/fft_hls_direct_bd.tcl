# Block design for FFT_IMPL==5: a single HLS IP (fft_hls_direct) that instantiates
# the LogiCORE FFT internally via hls::fft. No xfft cell or post IP in the BD — the
# SSR butterfly/twiddle, the sub-FFTs (xfft subcores generated inside the HLS IP),
# and the magnitude are all inside fft_hls_direct.
#
# External interface matches the other FFT BDs so fft_proc.sv is unchanged:
#   aclk, aresetn, s_axis (slave), m_axis (master), event_frame_started (output)
#
# Standalone:  vivado -mode tcl -source ip/fft_hls_direct_bd.tcl

proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    if {[info exists ::env($name)]}   { return $::env($name) }
    return $default
}

set part        [getparam part        xc7z020clg400-1]
set fft_ssr     [getparam fft_ssr     2]
set fft_scaled  [getparam fft_scaled  2]
# Must match the HLS build: the nfft pin only exists on the IP when built with
# FFT_RUNTIME_NFFT, so add the BD port under the same condition.
set fft_runtime [getparam fft_runtime_nfft 0]
set fft_width   [getparam fft_width   [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]

# s_axis: fft_ssr ADC samples (ASZ=14), byte-rounded
set s_bytes [expr {($fft_ssr * 14 + 7) / 8}]

if {[get_projects -quiet] eq ""} {
    create_project -in_memory -part $part
}

set ip_dir ".hls/fft_hls_direct/solution1/impl/ip"
if {[file isdirectory $ip_dir]} {
    set existing [get_property ip_repo_paths [current_project]]
    set_property ip_repo_paths [concat $existing $ip_dir] [current_project]
    update_ip_catalog
} else {
    puts "WARNING: HLS IP not found at $ip_dir — run 'make.sh hls' first"
}

if {[get_ipdefs -all xilinx.com:hls:fft_hls_direct:1.0] eq ""} {
    error "IP not found in catalog: xilinx.com:hls:fft_hls_direct:1.0 — check HLS build"
}

create_bd_design "fft_hls_direct_bd"

create_bd_port -dir I -type clk -freq_hz 250000000 aclk
create_bd_port -dir I -type rst                     aresetn
create_bd_port -dir O -type intr                    event_frame_started
if {$fft_runtime} {
    # Runtime sub-FFT length exponent (log2 of the per-lane transform size),
    # driven from fft_proc.sv's fft_nfft. Stable per frame; selects the run-time
    # configurable transform length inside the hls::fft core.
    create_bd_port -dir I -from 4 -to 0             nfft
}

create_bd_intf_port -mode Slave  -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis
set_property -dict [list \
    CONFIG.TDATA_NUM_BYTES  $s_bytes \
    CONFIG.HAS_TKEEP        0 \
    CONFIG.HAS_TLAST        1 \
    CONFIG.HAS_TREADY       1 \
    CONFIG.HAS_TSTRB        0 \
] [get_bd_intf_ports s_axis]

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis

create_bd_cell -type ip -vlnv xilinx.com:hls:fft_hls_direct:1.0 fft_0

connect_bd_net [get_bd_ports aclk]    [get_bd_pins fft_0/ap_clk]
connect_bd_net [get_bd_ports aresetn] [get_bd_pins fft_0/ap_rst_n]
if {$fft_runtime} {
    connect_bd_net [get_bd_ports nfft] [get_bd_pins fft_0/nfft]
}

connect_bd_intf_net [get_bd_intf_ports s_axis]       [get_bd_intf_pins fft_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins fft_0/m_axis]  [get_bd_intf_ports m_axis]
# Wire the ap_vld pulse, not the data pin. With the HLS port declared ap_vld, the
# core exposes event_frame_started (constant-1 data, left unconnected) plus
# event_frame_started_ap_vld — a clean 1-cycle pulse at each frame entry, which is
# what fft_proc.sv expects on its event_frame_started input.
connect_bd_net      [get_bd_pins fft_0/event_frame_started_ap_vld] [get_bd_ports event_frame_started]

save_bd_design
validate_bd_design

# Block design for FFT_IMPL==4: fft_native_pre -> single SSR xfft -> fft_native_mag
#
# Uses the Vivado 2025.2 xfft 9.1 native super-sample-rate support
# (CONFIG.super_sample_rates), so ONE xfft instance does the full N-point transform
# at FFT_SSR samples/clock. This removes the manual Cooley-Tukey pre/post
# decomposition of fft_ip_ssr_bd.tcl (and its fft_ip_ssr_post twiddle adder, which
# was the pll_ser_clk timing wall).
#
# External interface matches fft_ip_ssr_bd / fft_ssr_bd so fft_proc.sv is unchanged:
#   aclk, aresetn, s_axis (slave), m_axis (master), event_frame_started (output)
#
# Standalone:  vivado -mode tcl -source ip/fft_ssr_native_bd.tcl
# or sourced from red_pitaya_vivado.tcl with globals already set.

proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    if {[info exists ::env($name)]}   { return $::env($name) }
    return $default
}

set part        [getparam part        xc7z020clg400-1]
set fft_ssr     [getparam fft_ssr     2]
set fft_nfft    [getparam fft_nfft    12]
set fft_scaled  [getparam fft_scaled  2]
set fft_width   [getparam fft_width   [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]

set fft_size    [expr {1 << $fft_nfft}]
set scaling_opt [expr {$fft_scaled == 1 ? "scaled" : "unscaled"}]

# AXIS widths (bytes, rounded up)
set in_bytes  [expr {($fft_ssr * 14 + 7) / 8}]            ;# ASZ=14 real samples
set out_bytes [expr {($fft_ssr * $fft_width + 7) / 8}]    ;# DSZ=$fft_width magnitudes

# ---- Ensure a project exists (standalone run) ----------------------------
if {[get_projects -quiet] eq ""} {
    create_project -in_memory -part $part
}

# ---- Register HLS IP repos -----------------------------------------------
set repo_paths {}
foreach hls_ip {fft_native_pre fft_native_mag} {
    set ip_dir ".hls/${hls_ip}/solution1/impl/ip"
    if {[file isdirectory $ip_dir]} {
        lappend repo_paths $ip_dir
    } else {
        puts "WARNING: HLS IP not found at $ip_dir — run 'make.sh hls' first"
    }
}
if {$repo_paths ne {}} {
    set existing [get_property ip_repo_paths [current_project]]
    set_property ip_repo_paths [concat $existing $repo_paths] [current_project]
    update_ip_catalog
}

foreach ip_vlnv {
    xilinx.com:ip:xfft:9.1
    xilinx.com:hls:fft_native_pre:1.0
    xilinx.com:hls:fft_native_mag:1.0
} {
    if {[get_ipdefs -all $ip_vlnv] eq ""} {
        error "IP not found in catalog: $ip_vlnv — check HLS builds and IP repos"
    }
}

create_bd_design "fft_ssr_native_bd"

# ---- External ports -------------------------------------------------------
create_bd_port -dir I -type clk -freq_hz 250000000 aclk
create_bd_port -dir I -type rst                     aresetn
create_bd_port -dir O -type intr                    event_frame_started

create_bd_intf_port -mode Slave  -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis
set_property -dict [list \
    CONFIG.TDATA_NUM_BYTES  $in_bytes \
    CONFIG.HAS_TKEEP        0 \
    CONFIG.HAS_TLAST        1 \
    CONFIG.HAS_TREADY       1 \
    CONFIG.HAS_TSTRB        0 \
] [get_bd_intf_ports s_axis]

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis
# Master width is driven by the connected IP (mag/m_axis); no set_property needed.

# ---- Instances -----------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_native_pre:1.0 pre_0
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_native_mag:1.0 mag_0

create_bd_cell -type ip -vlnv xilinx.com:ip:xfft:9.1 xfft_0
set_property -dict [list \
    CONFIG.aresetn                                 {true} \
    CONFIG.super_sample_rates                      $fft_ssr \
    CONFIG.transform_length                        $fft_size \
    CONFIG.data_format                             {fixed_point} \
    CONFIG.implementation_options                  {pipelined_streaming_io} \
    CONFIG.input_width                             {16} \
    CONFIG.phase_factor_width                      {18} \
    CONFIG.rounding_modes                          {convergent_rounding} \
    CONFIG.run_time_configurable_transform_length  {false} \
    CONFIG.scaling_options                         $scaling_opt \
    CONFIG.output_ordering                         {natural_order} \
    CONFIG.target_clock_frequency                  {250} \
    CONFIG.target_data_throughput                  {250} \
    CONFIG.throttle_scheme                         {nonrealtime} \
] [get_bd_cells xfft_0]

# ---- Clock / reset --------------------------------------------------------
foreach pin [list pre_0/ap_clk mag_0/ap_clk xfft_0/aclk] {
    connect_bd_net [get_bd_ports aclk] [get_bd_pins $pin]
}
foreach pin [list pre_0/ap_rst_n mag_0/ap_rst_n xfft_0/aresetn] {
    connect_bd_net [get_bd_ports aresetn] [get_bd_pins $pin]
}

# ---- AXIS datapath --------------------------------------------------------
connect_bd_intf_net [get_bd_intf_ports s_axis]            [get_bd_intf_pins pre_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_data]  [get_bd_intf_pins xfft_0/S_AXIS_DATA]
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg]   [get_bd_intf_pins xfft_0/S_AXIS_CONFIG]
connect_bd_intf_net [get_bd_intf_pins xfft_0/M_AXIS_DATA] [get_bd_intf_pins mag_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins mag_0/m_axis]       [get_bd_intf_ports m_axis]

connect_bd_net [get_bd_pins pre_0/event_frame_started] [get_bd_ports event_frame_started]

save_bd_design
validate_bd_design

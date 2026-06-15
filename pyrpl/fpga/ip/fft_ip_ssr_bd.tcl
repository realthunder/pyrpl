# Block design for FFT_IMPL==3: fft_ip_ssr_pre → SSR×xfft → fft_ip_ssr_post
#
# External interface matches fft_ssr_bd so fft_proc.sv needs no changes:
#   aclk, aresetn
#   s_axis  (slave,  byte-rounded FSSR*ASZ bits, tlast)
#   m_axis  (master, byte-rounded FSSR*DSZ bits, tlast)
#   event_frame_started (output)
#
# Can be sourced standalone via:
#   vivado -mode tcl -source ip/fft_ip_ssr_bd.tcl
# or sourced from red_pitaya_vivado.tcl with globals already set.

proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    if {[info exists ::env($name)]}   { return $::env($name) }
    return $default
}

set part           [getparam part           xc7z020clg400-1]
set fft_ssr        [getparam fft_ssr        2]
set fft_nfft       [getparam fft_nfft       12]

set ssr_bits  [expr {int(log($fft_ssr) / log(2) + 0.5)}]
set sub_nfft  [expr {$fft_nfft - $ssr_bits}]
set sub_size  [expr {1 << $sub_nfft}]
set stages_bram [expr {$sub_nfft > 9 ? $sub_nfft - 9 : 0}]

# AXIS widths (bytes, rounded up)
set in_bytes  [expr {($fft_ssr * 14 + 7) / 8}]   ;# ASZ=14
set out_bytes [expr {($fft_ssr * 28 + 7) / 8}]   ;# DSZ=28
set cmpx_bytes 4                                  ;# 2*INT_W/8 = 32/8 for INT_W=16

# ---- Ensure a project exists (create temporary one when run standalone) --
if {[get_projects -quiet] eq ""} {
    create_project -in_memory -part $part
}

# ---- Register HLS IP repos ------------------------------------------------
set repo_paths {}
foreach hls_ip {fft_ip_ssr_pre fft_ip_ssr_post} {
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

# ---- Sanity: require xfft 9.1 and the two HLS IPs -----------------------
foreach ip_vlnv {
    xilinx.com:ip:xfft:9.1
    xilinx.com:hls:fft_ip_ssr_pre:1.0
    xilinx.com:hls:fft_ip_ssr_post:1.0
} {
    if {[get_ipdefs -all $ip_vlnv] eq ""} {
        error "IP not found in catalog: $ip_vlnv — check HLS builds and IP repos"
    }
}

create_bd_design "fft_ip_ssr"

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
set_property -dict [list \
    CONFIG.TDATA_NUM_BYTES  $out_bytes \
    CONFIG.HAS_TKEEP        0 \
    CONFIG.HAS_TLAST        1 \
    CONFIG.HAS_TREADY       1 \
    CONFIG.HAS_TSTRB        0 \
] [get_bd_intf_ports m_axis]

# ---- Instances -----------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_ip_ssr_pre:1.0  pre_0
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_ip_ssr_post:1.0 post_0

for {set ch 0} {$ch < $fft_ssr} {incr ch} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xfft:9.1 xfft_$ch
    set_property -dict [list \
        CONFIG.aresetn                                              {true} \
        CONFIG.data_format                                          {fixed_point} \
        CONFIG.implementation_options                               {pipelined_streaming_io} \
        CONFIG.input_width                                          {16} \
        CONFIG.output_ordering                                      {bit_reversed_order} \
        CONFIG.phase_factor_width                                   {18} \
        CONFIG.rounding_modes                                       {convergent_rounding} \
        CONFIG.run_time_configurable_transform_length               {false} \
        CONFIG.scaling_options                                      {scaled} \
        CONFIG.target_clock_frequency                               {250} \
        CONFIG.target_data_throughput                               {250} \
        CONFIG.throttle_scheme                                      {nonrealtime} \
        CONFIG.transform_length                                     $sub_size \
        CONFIG.number_of_stages_using_block_ram_for_data_and_phase_factors $stages_bram \
    ] [get_bd_cells xfft_$ch]
}

# ---- Clock / reset --------------------------------------------------------
foreach pin [list \
    pre_0/ap_clk  post_0/ap_clk \
    xfft_0/aclk   xfft_1/aclk \
] {
    connect_bd_net [get_bd_ports aclk] [get_bd_pins $pin]
}
foreach pin [list \
    pre_0/ap_rst_n  post_0/ap_rst_n \
    xfft_0/aresetn  xfft_1/aresetn \
] {
    connect_bd_net [get_bd_ports aresetn] [get_bd_pins $pin]
}
if {$fft_ssr >= 4} {
    connect_bd_net [get_bd_ports aclk]    [get_bd_pins xfft_2/aclk]
    connect_bd_net [get_bd_ports aclk]    [get_bd_pins xfft_3/aclk]
    connect_bd_net [get_bd_ports aresetn] [get_bd_pins xfft_2/aresetn]
    connect_bd_net [get_bd_ports aresetn] [get_bd_pins xfft_3/aresetn]
}

# ---- External AXIS --------------------------------------------------------
connect_bd_intf_net [get_bd_intf_ports s_axis] [get_bd_intf_pins pre_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins post_0/m_axis] [get_bd_intf_ports m_axis]
connect_bd_net [get_bd_pins pre_0/event_frame_started] [get_bd_ports event_frame_started]

# ---- Internal: pre → xfft data+config, xfft → post ----------------------
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_data0] \
                    [get_bd_intf_pins xfft_0/S_AXIS_DATA]
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg0] \
                    [get_bd_intf_pins xfft_0/S_AXIS_CONFIG]
connect_bd_intf_net [get_bd_intf_pins xfft_0/M_AXIS_DATA] \
                    [get_bd_intf_pins post_0/s_axis_data0]

connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_data1] \
                    [get_bd_intf_pins xfft_1/S_AXIS_DATA]
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg1] \
                    [get_bd_intf_pins xfft_1/S_AXIS_CONFIG]
connect_bd_intf_net [get_bd_intf_pins xfft_1/M_AXIS_DATA] \
                    [get_bd_intf_pins post_0/s_axis_data1]

if {$fft_ssr >= 4} {
    connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_data2] \
                        [get_bd_intf_pins xfft_2/S_AXIS_DATA]
    connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg2] \
                        [get_bd_intf_pins xfft_2/S_AXIS_CONFIG]
    connect_bd_intf_net [get_bd_intf_pins xfft_2/M_AXIS_DATA] \
                        [get_bd_intf_pins post_0/s_axis_data2]

    connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_data3] \
                        [get_bd_intf_pins xfft_3/S_AXIS_DATA]
    connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg3] \
                        [get_bd_intf_pins xfft_3/S_AXIS_CONFIG]
    connect_bd_intf_net [get_bd_intf_pins xfft_3/M_AXIS_DATA] \
                        [get_bd_intf_pins post_0/s_axis_data3]
}

save_bd_design
validate_bd_design

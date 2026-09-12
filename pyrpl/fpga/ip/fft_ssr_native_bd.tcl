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
    # Env vars are UPPERCASE (FFT_THROTTLE); the tcl var name is lowercase.
    set envname [string toupper $name]
    if {[info exists ::env($envname)]} { return $::env($envname) }
    return $default
}

set part        [getparam part        xc7z020clg400-1]
set fft_ssr     [getparam fft_ssr     2]
set fft_nfft    [getparam fft_nfft    12]
set fft_scaled  [getparam fft_scaled  2]
set fft_width   [getparam fft_width   [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]
# xfft AXIS throttle scheme (FFT_THROTTLE env / fft_throttle global):
#   realtime    (default) — no input/output flow control inside the core. Drops the
#                 non-realtime data_in/data_out throttle FIFOs (~-844 LUT / -315 FF per
#                 core at SSR4/2048, measured OOC) and their two top-fanout CE nets.
#                 REQUIRES the fft_proc feed to keep tvalid high for every beat of a
#                 frame (full-half input buffering) and a never-stalling mag/peak sink;
#                 a violation raises event_data_in/out_channel_halt (exported below,
#                 latched sticky in fft_proc status_o) and corrupts that frame.
#   nonrealtime — the previous behaviour (core back-pressures / tolerates gaps).
set fft_throttle [getparam fft_throttle realtime]
if {$fft_throttle ne "realtime" && $fft_throttle ne "nonrealtime"} {
    error "fft_throttle must be realtime or nonrealtime, got '$fft_throttle'"
}

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
# Core event pulses, exported so fft_proc can latch them (sticky) into status_o.
# Not every pin exists in every throttle mode (e.g. data_out_channel_halt is
# realtime-only): a missing one is tied 0 so the wrapper port set is constant.
foreach ev {event_data_in_channel_halt event_data_out_channel_halt \
            event_tlast_missing event_tlast_unexpected} {
    create_bd_port -dir O -type intr $ev
}

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
    CONFIG.throttle_scheme                         $fft_throttle \
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
# pre_0 -> xfft data: pin-level, with TLAST gated by TVALID. The HLS master's
# output register slice HOLDS TLAST at the last transferred value, so between
# frames the net sits high (TVALID low) for the whole inter-frame gap. That is
# legal AXI4-Stream, but the realtime-throttle xfft samples TLAST around a frame
# start without qualifying it and reports event_tlast_unexpected twice per frame
# (seen in sim/tb_fft_chain.sv). AND-ing with TVALID makes the net a clean pulse.
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:2.0 tlast_gate
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells tlast_gate]
connect_bd_net [get_bd_pins pre_0/m_axis_data_TVALID] [get_bd_pins tlast_gate/Op1]
connect_bd_net [get_bd_pins pre_0/m_axis_data_TLAST]  [get_bd_pins tlast_gate/Op2]
connect_bd_net [get_bd_pins tlast_gate/Res]           [get_bd_pins xfft_0/s_axis_data_tlast]
connect_bd_net [get_bd_pins pre_0/m_axis_data_TVALID] [get_bd_pins xfft_0/s_axis_data_tvalid]
connect_bd_net [get_bd_pins pre_0/m_axis_data_TREADY] [get_bd_pins xfft_0/s_axis_data_tready]
connect_bd_net [get_bd_pins pre_0/m_axis_data_TDATA]  [get_bd_pins xfft_0/s_axis_data_tdata]
connect_bd_intf_net [get_bd_intf_pins pre_0/m_axis_cfg]   [get_bd_intf_pins xfft_0/S_AXIS_CONFIG]
connect_bd_intf_net [get_bd_intf_pins xfft_0/M_AXIS_DATA] [get_bd_intf_pins mag_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins mag_0/m_axis]       [get_bd_intf_ports m_axis]

# event_frame_started comes from the real xfft core (like FFT_IMPL==1). The HLS
# fft_native_pre wrapper also exposes an event_frame_started pin, but it is hard-
# wired to 0 (assign event_frame_started = 1'd0;), which left frame_cnt/scan_frame_cnt
# in fft_proc.sv frozen. Tap xfft_0 so the pulse actually fires once per frame.
connect_bd_net [get_bd_pins xfft_0/event_frame_started] [get_bd_ports event_frame_started]

set const0 ""
foreach ev {event_data_in_channel_halt event_data_out_channel_halt \
            event_tlast_missing event_tlast_unexpected} {
    if {[get_bd_pins -quiet xfft_0/$ev] ne ""} {
        connect_bd_net [get_bd_pins xfft_0/$ev] [get_bd_ports $ev]
        puts "fft_ssr_native_bd: $ev exported from xfft_0 (throttle=$fft_throttle)"
    } else {
        if {$const0 eq ""} {
            set const0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 ev_const0]
            set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] $const0
        }
        connect_bd_net [get_bd_pins ev_const0/dout] [get_bd_ports $ev]
        puts "fft_ssr_native_bd: $ev not present on xfft_0 (throttle=$fft_throttle), tied 0"
    }
}

save_bd_design
validate_bd_design

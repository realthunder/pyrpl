create_bd_design "fft_ssr_bd"

set_property ip_repo_paths "./.hls/fft_ssr/solution1/impl/ip" [current_project]
update_ip_catalog

# Add IP
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_ssr:1.0 fft_ssr_0

# Create clock
create_bd_port -dir I -type clk -freq_hz 125000000 aclk 
create_bd_port -dir I -type rst aresetn

# Connect clock/reset
connect_bd_net [get_bd_ports aclk] [get_bd_pins fft_ssr_0/ap_clk]
connect_bd_net [get_bd_ports aresetn] [get_bd_pins fft_ssr_0/ap_rst_n]

# Create AXIS ports — widths derived from build parameters
# s_axis: fft_ssr lanes × cint16 (2×16-bit complex) = fft_ssr × 4 bytes
# m_axis: fft_ssr lanes × fft_width bits
set s_bytes [expr {$fft_ssr * 4}]
set m_bytes [expr {$fft_ssr * $fft_width / 8}]
create_bd_intf_port -mode Slave  -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis
set_property CONFIG.TDATA_NUM_BYTES $s_bytes [get_bd_intf_ports s_axis]
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis

# Connect AXIS ports
connect_bd_intf_net [get_bd_intf_ports s_axis] [get_bd_intf_pins fft_ssr_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins fft_ssr_0/m_axis] [get_bd_intf_ports m_axis]

# EVENT SIGNAL
create_bd_port -dir O event_frame_started

connect_bd_net [get_bd_ports event_frame_started] \
               [get_bd_pins fft_ssr_0/event_frame_started]

save_bd_design


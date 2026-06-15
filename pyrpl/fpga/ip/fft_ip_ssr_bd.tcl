create_bd_design "fft_ip_ssr_bd"

set_property ip_repo_paths "./.hls/fft_ip_ssr/solution1/impl/ip" [current_project]
update_ip_catalog

# Add IP
create_bd_cell -type ip -vlnv xilinx.com:hls:fft_ip_ssr:1.0 fft_ip_ssr_0

# Create clock / reset ports
create_bd_port -dir I -type clk -freq_hz 125000000 aclk
create_bd_port -dir I -type rst aresetn

connect_bd_net [get_bd_ports aclk]    [get_bd_pins fft_ip_ssr_0/ap_clk]
connect_bd_net [get_bd_ports aresetn] [get_bd_pins fft_ip_ssr_0/ap_rst_n]

# Input port: fft_ssr ADC samples packed, each ASZ=14 bits, byte-rounded
# s_bytes = ceil(fft_ssr * 14 / 8)
set s_bytes [expr {($fft_ssr * 14 + 7) / 8}]
create_bd_intf_port -mode Slave  -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis
set_property CONFIG.TDATA_NUM_BYTES $s_bytes [get_bd_intf_ports s_axis]

# Output port: fft_ssr magnitude lanes, each fft_width bits — width inferred from IP
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis

connect_bd_intf_net [get_bd_intf_ports s_axis] [get_bd_intf_pins fft_ip_ssr_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins fft_ip_ssr_0/m_axis] [get_bd_intf_ports m_axis]

# Event signal
create_bd_port -dir O event_frame_started
connect_bd_net [get_bd_ports event_frame_started] \
               [get_bd_pins fft_ip_ssr_0/event_frame_started]

save_bd_design

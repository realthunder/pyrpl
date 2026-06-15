create_bd_design "peak_detector_bd"

set_property ip_repo_paths "./.hls/peak_detector/solution1/impl/ip" [current_project]
update_ip_catalog

# Add HLS IP
create_bd_cell -type ip -vlnv xilinx.com:hls:peak_detector:1.0 peak_detector_0

# Clock / reset
create_bd_port -dir I -type clk -freq_hz 250000000 aclk
create_bd_port -dir I -type rst aresetn

connect_bd_net [get_bd_ports aclk]    [get_bd_pins peak_detector_0/ap_clk]
connect_bd_net [get_bd_ports aresetn] [get_bd_pins peak_detector_0/ap_rst_n]

# AXI-Stream ports — widths derived from build parameters
# s_axis: fft_ssr lanes × fft_width bits (consumes fft_ssr m_axis output)
# m_axis: fixed 64-bit result packet
set s_bytes [expr {$fft_ssr * $fft_width / 8}]
set m_bytes 8
create_bd_intf_port -mode Slave  -vlnv xilinx.com:interface:axis_rtl:1.0 s_axis
set_property CONFIG.TDATA_NUM_BYTES $s_bytes [get_bd_intf_ports s_axis]
create_bd_intf_port -mode Master -vlnv xilinx.com:interface:axis_rtl:1.0 m_axis

connect_bd_intf_net [get_bd_intf_ports s_axis] [get_bd_intf_pins peak_detector_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins peak_detector_0/m_axis] [get_bd_intf_ports m_axis]

# Auto-start: tie ap_start = 1 so the block restarts immediately after each frame
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_start
set_property CONFIG.CONST_VAL {1} [get_bd_cells const_start]
connect_bd_net [get_bd_pins const_start/dout] [get_bd_pins peak_detector_0/ap_start]

# ap_done: one pulse per completed frame (useful for status / debug)
create_bd_port -dir O ap_done
connect_bd_net [get_bd_ports ap_done] [get_bd_pins peak_detector_0/ap_done]

# Control ports (ap_none → direct wires; widths match HLS defaults DSZ=28, SSZ=14)
create_bd_port -dir I -from 15 -to 0 threshold_k_sq
create_bd_port -dir I -from 13 -to 0 start_index
create_bd_port -dir I -from 13 -to 0 end_index
create_bd_port -dir I -from 27 -to 0 data_min
create_bd_port -dir I -from  3 -to 0 nfft

connect_bd_net [get_bd_ports threshold_k_sq] [get_bd_pins peak_detector_0/threshold_k_sq]
connect_bd_net [get_bd_ports start_index]    [get_bd_pins peak_detector_0/start_index]
connect_bd_net [get_bd_ports end_index]      [get_bd_pins peak_detector_0/end_index]
connect_bd_net [get_bd_ports data_min]       [get_bd_pins peak_detector_0/data_min]
connect_bd_net [get_bd_ports nfft]           [get_bd_pins peak_detector_0/nfft]

save_bd_design

################################################################################
# RF Data Converter for the ZCU216 (XCZU49DR), module rfdc. Sourced by
# zcu216_vivado.tcl after create_project.
#
# Sample clocks come straight from the CLK104 LMX2594s at 2.5 GHz (tile PLLs off;
# zcu216/clk104 programs them): ADC_REFCLK reaches ADC tile 226 and DAC_REFCLK DAC
# tile 230 (as in AMD's RF DC evaluation design), and each forwards its clock to
# its neighbours. Clocks may not hop a tile, so 225/229 relay to 224/228.
#
#   ADC tile 224, channels 0-3: real, no decimation, 8 samples x 16 bit per
#                               312.5 MHz AXI-Stream beat (m0x_axis)
#   DAC tile 228, channels 0-3: real, no interpolation, 8 samples per beat (s0x_axis)
#   ADC 225/226, DAC 229/230:   one channel each, only so the tiles are on to
#                               carry the clock; their streams are unused
#
# The IP validates each value against the current state, so the order matters:
# slices first (they enable the tiles), then rates, then the distributing tiles,
# then the clock sources, then the per-tile widths together.
################################################################################

proc rfdc_zcu216_create {dir} {
    file mkdir $dir
    create_ip -name usp_rf_data_converter -vendor xilinx.com -library ip \
              -module_name rfdc -dir $dir -force
    set rf [get_ips rfdc]

    proc rf_set {rf args} {
        foreach {k v} $args {
            set_property CONFIG.$k $v $rf
            if {[get_property CONFIG.$k $rf] ne $v} {
                error "rfdc: $k = $v not taken (is [get_property CONFIG.$k $rf])"
            }
        }
    }

    foreach s {00 01 02 03 10 20} {
        rf_set $rf ADC_Slice${s}_Enable true DAC_Slice${s}_Enable true
    }
    foreach t {0 1 2} {
        rf_set $rf ADC${t}_Sampling_Rate 2.5 ADC${t}_Refclk_Freq 2500.000
        rf_set $rf DAC${t}_Sampling_Rate 2.5 DAC${t}_Refclk_Freq 2500.000
    }
    rf_set $rf ADC2_Clock_Dist 1 DAC2_Clock_Dist 1
    rf_set $rf ADC1_Clock_Source 2 DAC1_Clock_Source 6
    rf_set $rf ADC0_Clock_Source 2 DAC0_Clock_Source 6

    set d {}
    foreach s {00 01 02 03 10 20} {
        lappend d CONFIG.ADC_Data_Type$s 0 CONFIG.ADC_Decimation_Mode$s 1 \
                  CONFIG.ADC_Data_Width$s 8 CONFIG.ADC_Nyquist$s 0 \
                  CONFIG.DAC_Data_Type$s 0 CONFIG.DAC_Interpolation_Mode$s 1 \
                  CONFIG.DAC_Data_Width$s 8 CONFIG.DAC_Nyquist$s 0
    }
    set_property -dict $d $rf
    foreach t {0 1 2} {
        rf_set $rf ADC${t}_Fabric_Freq 312.500 DAC${t}_Fabric_Freq 312.500
    }
    foreach p {ADC0_Clock_Source ADC0_Fabric_Freq ADC_Data_Width00 ADC_Mixer_Mode00
               DAC0_Clock_Source DAC0_Fabric_Freq DAC_Data_Width00 DAC_Mixer_Mode00} {
        puts "INFO: rfdc $p = [get_property CONFIG.$p $rf]"
    }
    return $rf
}

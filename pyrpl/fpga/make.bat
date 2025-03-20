@RD /S /Q out
@RD /S /Q .Xil
@RD /S /Q .srcs
@RD /S /Q sdk

REM  d:/Xilinx/Vivado/2020.1/bin/vivado.bat -nolog -nojournal -mode tcl -source red_pitaya_vivado_project.tcl
d:/Xilinx/Vivado/2020.1/bin/vivado.bat -nolog -nojournal -mode tcl -source red_pitaya_vivado.tcl

echo compilation finished

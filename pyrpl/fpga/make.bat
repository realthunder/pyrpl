@RD /S /Q out
@RD /S /Q .Xil
@RD /S /Q .srcs
@RD /S /Q sdk

IF "%~1" == "" (
  set script=red_pitaya_vivado.tcl
) ELSE (
  set script="%1"
  shift
)

REM  d:/Xilinx/Vivado/2020.1/bin/vivado.bat -nolog -nojournal -mode tcl -source red_pitaya_vivado_project.tcl
d:/Xilinx/Vivado/2020.1/bin/vivado.bat -nolog -nojournal -mode tcl -source %script% -tclargs %*

echo compilation finished

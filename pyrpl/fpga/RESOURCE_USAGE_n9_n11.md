# FPGA resource usage - latest n9 vs n11 builds

Device: **xc7z020clg400-1** (Vivado 2025.2). Both builds are `FFT_IMPL=4`, `FFT_SSR=4`,
`FFT_WIDTH=24`, `FFT_CLK_SEL=0`, ramp / SO-CFAR peak detector.

## Source reports

| Build | Snapshot directory                  | Utilization report                                     | Build config / timing                             |
| ----- | ----------------------------------- | ------------------------------------------------------ | ------------------------------------------------- |
| n9    | `out.d/n9-ramp-so-adc+0.151/`       | `out.d/n9-ramp-so-adc+0.151/post_route_util.rpt`       | `out.d/n9-ramp-so-adc+0.151/BUILD_INFO.txt`       |
| n11   | `out.d/n11-ramp-so-slim-adc+0.023/` | `out.d/n11-ramp-so-slim-adc+0.023/post_route_util.rpt` | `out.d/n11-ramp-so-slim-adc+0.023/BUILD_INFO.txt` |

Paths are relative to `pyrpl/fpga/`. Reports were produced post-route by
`report_utilization -hierarchical -hierarchical_depth 3` (see `red_pitaya_vivado.tcl:554`),
so the numbers below are the `red_pitaya_top` row of each report; percentages are computed
against the xc7z020 device capacities and are not printed in the report itself.

Build identity:

| Item        | n9                         | n11                                         |
| ----------- | -------------------------- | ------------------------------------------- |
| FFT_NFFT    | 9 (512-pt)                 | 11 (2048-pt)                                |
| Build date  | 2026-07-20                 | 2026-07-22                                  |
| Git commit  | 3fcd1032 (dirty)           | 1bf0c7b0 (dirty)                            |
| WNS adc clk | +0.151 ns                  | +0.023 ns (after steping multicycle on dcp) |
| WHS         | not recorded in BUILD_INFO | +0.020 ns                                   |

## Resource usage

| Resource             | Available | n9 used | n9 %   | n11 used | n11 %  |
| -------------------- | --------- | ------- | ------ | -------- | ------ |
| Slice LUTs (total)   | 53200     | 41245   | 77.5 % | 48300    | 90.8 % |
| - LUTs as logic      | 53200     | 35323   | 66.4 % | 36396    | 68.4 % |
| - LUTs as memory     | 17400     | 5922    | 34.0 % | 11904    | 68.4 % |
| - - LUTRAM           | 17400     | 2670    | 15.3 % | 6812     | 39.1 % |
| - - SRL              | 17400     | 3252    | 18.7 % | 5092     | 29.3 % |
| Slice registers (FF) | 106400    | 53017   | 49.8 % | 55461    | 52.1 % |
| Block RAM tiles      | 140       | 92.5    | 66.1 % | 87.5     | 62.5 % |
| - RAMB36             | 140       | 84      | 60.0 % | 68       | 48.6 % |
| - RAMB18             | 280       | 17      | 6.1 %  | 39       | 13.9 % |
| DSP48E1              | 220       | 167     | 75.9 % | 185      | 84.1 % |

Notes:

- "LUTs as memory" is the sum of LUTRAM and SRL; both draw from the same 17400-site
  pool (only SLICEM LUTs), which is why n11 sits at 68 % there while logic LUTs are at 68 %
  of the much larger total pool.
- A RAMB18 counts as half a block RAM tile, hence tiles = RAMB36 + RAMB18 / 2.
- n11 is LUT-limited (90.8 %), which is what made this configuration hard to place and route;
  n9 has substantially more headroom on every resource except block RAM.
- The hierarchical reports contain no `IO`, `BUFG`, `MMCM`, or `PLL` section - regenerate a
  flat `report_utilization` from the checkpoint (`post_route.dcp`, or `post_route_mcp.dcp`
  for the timing-closed n11) if those are needed.

#!/bin/bash
# condensed critical path from pnr.log: cumulative time + shortened net/cell names
awk '/Critical path report for clock/{f=1} /Critical path report for cross-domain/{f=0} f' pnr.log | \
 grep -E "Info: +(clk-to-q|logic|routing|setup) " | awk '{n=$NF; gsub(/_(LUT4|PFUMX|L6MUX21|TRELLIS_FF|CCU2C|DPRAM)[A-Za-z0-9_$.]*/,"",n); print $2, $4, substr(n,1,90)}' | grep -v "^routing"
grep -E "ns logic, .* ns routing" pnr.log | head -1

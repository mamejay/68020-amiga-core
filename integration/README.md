# Base64 integration files

Made against jbilander/base64_gateware commit dac50c8 (fx68k build, "reset: a RESET instruction must reset the autoconfig boards").

* `base64_top.patch` - swaps the fx68k instance for m68020_fx68k (CE_DIV = 3). Apply from the
  gateware root: `patch -p1 < base64_top.patch`
* `base64_m68020.fdc` - multicycle constraints for the core, to merge into
  fx68k/constraints/base64.fdc
* Project file: remove the six rtl/fx68k/* sources from prj/base64_fx68k/base64_fx68k.ldf and add
  rtl/m68020_core.sv + rtl/m68020_fx68k.sv (see ../doc/integration.md).

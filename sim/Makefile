# Lockstep harness: m68020_core (Verilator) vs Musashi 68020.
#   make                  build ./lockstep (uses ../rtl/m68020_core.sv if present, else stub_core.sv)
#   make TRACE=1          also enable --vcd
#   make tests            assemble tests/*.S -> tests/*.bin
#   make check            run directed tests + a random batch
export PATH := $(HOME)/.local/opt/oss-cad-suite/bin:$(PATH)

RTL_DIR   ?= ../rtl
CORE_SRC  ?= $(if $(wildcard $(RTL_DIR)/m68020_core.sv),$(RTL_DIR)/m68020_core.sv $(filter-out $(RTL_DIR)/m68020_core.sv,$(wildcard $(RTL_DIR)/*.sv)),stub_core.sv)
TRACE     ?= 0
SEED      ?= 1
COUNT     ?= 2000

MUS       = musashi
MUS_SRC   = $(MUS)/m68kcpu.c $(MUS)/m68kops.c $(MUS)/m68kdasm.c $(MUS)/softfloat/softfloat.c
MUS_OBJ   = $(patsubst %.c,obj_mus/%.o,$(notdir $(MUS_SRC)))

VFLAGS    = --cc --exe --build -j 0 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -O3 --top-module tb \
            -CFLAGS "-O2 -I$(CURDIR)/$(MUS)" -LDFLAGS "$(addprefix $(CURDIR)/,$(MUS_OBJ)) -lm"
ifeq ($(TRACE),1)
VFLAGS   += --trace
endif

AS        = m68k-linux-gnu-as
LD        = m68k-linux-gnu-ld
OBJCOPY   = m68k-linux-gnu-objcopy
TESTS_S   = $(wildcard tests/*.S)
TESTS_BIN = $(TESTS_S:.S=.bin)

all: lockstep

$(MUS)/m68kops.c $(MUS)/m68kops.h: $(MUS)/m68k_in.c $(MUS)/m68kmake.c
	cd $(MUS) && gcc -O2 -o m68kmake m68kmake.c && ./m68kmake . m68k_in.c

obj_mus/%.o: $(MUS)/%.c $(MUS)/m68kops.h $(MUS)/m68kconf.h $(MUS)/m68kcpu.h
	@mkdir -p obj_mus
	gcc -O2 -w -I$(MUS) -c $< -o $@
obj_mus/softfloat.o: $(MUS)/softfloat/softfloat.c
	@mkdir -p obj_mus
	gcc -O2 -w -I$(MUS) -c $< -o $@

# rebuild when the selected core sources change (stub <-> real RTL)
.core_src: FORCE
	@echo "$(CORE_SRC) TRACE=$(TRACE)" | cmp -s - $@ || echo "$(CORE_SRC) TRACE=$(TRACE)" > $@
FORCE:

lockstep: lockstep.cpp tb.sv $(CORE_SRC) $(MUS_OBJ) .core_src
	verilator $(VFLAGS) tb.sv $(CORE_SRC) lockstep.cpp -o Vtb
	cp obj_dir/Vtb lockstep
	@echo "built lockstep with CORE_SRC=$(CORE_SRC)"

tests: $(TESTS_BIN)
tests/%.bin: tests/%.S
	$(AS) -m68020 --register-prefix-optional -o tests/$*.o $<
	$(LD) -Ttext=0 -e 0 --no-warn-rwx-segments -o tests/$*.elf tests/$*.o
	$(OBJCOPY) -O binary -j .text tests/$*.elf $@

check: lockstep tests
	@fail=0; for t in $(TESTS_BIN); do ./lockstep prog $$t --max 200000 || fail=1; done; \
	./lockstep random --seed $(SEED) --count $(COUNT) --show 10 || fail=1; exit $$fail

musashi-check: lockstep
	./lockstep --musashi-only -v

clean:
	rm -rf obj_dir obj_mus lockstep .core_src tests/*.o tests/*.elf tests/*.bin

.PHONY: all tests check clean musashi-check FORCE

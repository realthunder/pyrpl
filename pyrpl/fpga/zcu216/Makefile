# Board-side tools for the ZCU216, static against musl (any aarch64 kernel), built
# with zig: python3 -m venv ~/.venvs/zig && ~/.venvs/zig/bin/pip install ziglang
ZIG_CC := $(HOME)/.venvs/zig/bin/python -m ziglang cc -target aarch64-linux-musl -static -O2 -Wall

clk104: clk104.c clk104_tables.h
	$(ZIG_CC) -o $@ clk104.c

clk104_tables.h: gen_clk104_tables.py
	python3 -I gen_clk104_tables.py

clean:
	rm -f clk104

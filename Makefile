CXXFLAGS=-O3 -std=c++11

all: rtl_zwave zwave_mod

rtl_zwave: rtl_zwave.o
	$(CXX) -o $@  $^ $(LDFLAGS)

zwave_mod: zwave_mod.o
	$(CXX) -o $@  $^ $(LDFLAGS)

clean:
	rm -f zwave_mod.o zwave_mod rtl_zwave.o rtl_zwave

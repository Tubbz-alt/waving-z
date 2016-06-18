CXXFLAGS=-O3 -std=c++11
rtl_zwave: rtl_zwave.o
	$(CXX) -o $@  $^ $(LDFLAGS)


clean:
	rm -f rtl_zwave.o rtl_zwave

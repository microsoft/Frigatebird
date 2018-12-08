# top level makefile to build SITL for primary vehicle targets. 
# Useful for static analysis tools

all: sitl

sitl: TARGET=sitl
sitl: plane

linux: TARGET=linux
linux: plane

clean: TARGET=clean
clean: plane

.PHONY: plane

plane:
	$(MAKE) -C ArduPlane $(TARGET)


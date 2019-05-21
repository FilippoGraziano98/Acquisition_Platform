all:
	+$(MAKE) -C host_build
	+$(MAKE) -C firmware_build

clean:
	+$(MAKE) clean -C host_build
	+$(MAKE) clean -C firmware_build

upload:
	+$(MAKE) upload -C firmware_build

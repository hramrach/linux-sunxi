subdir		:= standby

obj-y		+= pm.o standby.o \
	     standby-kernel.o standby/standby_delay.o

$(obj)/standby.o: $(obj)/standby/standby.bin

$(obj)/standby/standby.bin: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/standby

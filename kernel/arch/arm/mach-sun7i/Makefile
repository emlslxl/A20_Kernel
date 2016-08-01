#
# Makefile for the linux kernel.
#

obj-y = clock/
obj-y += dma/
obj-$(CONFIG_CPU_FREQ)			+= cpu-freq/
obj-$(CONFIG_PM) += pm/
obj-$(CONFIG_AW_TIME_DELAY)	+= delay.o
obj-$(CONFIG_BOARD_NAS1000) += board-nas1000.o
obj-$(CONFIG_BOARD_EM6000) += board-em6000.o
obj-$(CONFIG_BOARD_EM6057E) += board-em6057e.o
obj-$(CONFIG_BOARD_EP1000M) += board-ep1000m.o
ifeq ($(CONFIG_SMP),y)
obj-y += platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o


ifeq ($(CONFIG_LOCAL_TIMERS),y)
obj-y += localtimer.o
endif


endif



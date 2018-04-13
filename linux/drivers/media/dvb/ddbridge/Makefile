#
# Makefile for the ddbridge device driver
#

ddbridge-objs := ddbridge-core.o

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o

EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core/
EXTRA_CFLAGS += -Idrivers/media/dvb/frontends/
EXTRA_CFLAGS += -Idrivers/media/common/tuners/

# For the staging CI driver cxd2099
EXTRA_CFLAGS += -Idrivers/media/staging/cxd2099/

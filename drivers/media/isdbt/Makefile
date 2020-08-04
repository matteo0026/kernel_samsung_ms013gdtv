##########################################
# Makefile for ISDBT Device Drivers.
##########################################

###########
# Raontech
###########

ccflags-$(CONFIG_ISDBT)  += -Idrivers/media/isdbt
#ccflags-y += -v

obj-$(CONFIG_ISDBT_VENDOR_RAONTECH) := isdbt.o

obj-$(CONFIG_ISDBT_FC8300) += fc8300/
# MTV230
ccflags-$(CONFIG_ISDBT_MTV23x) += -Idrivers/media/isdbt/mtv23x
obj-$(CONFIG_ISDBT_MTV23x) += mtv23x/

# SPI
obj-$(CONFIG_ISDBT_VENDOR_RAONTECH) += isdbt_spi.o
obj-$(CONFIG_ISDBT_VENDOR_RAONTECH) += isdbt_isr.o

# SPI(Control) + TSIF(Data)
#obj-y += isdbt_spi.o

# TSIF
#obj-y += isdbt_i2c.o

# CAMIF
#obj-y += isdbt_i2c.o


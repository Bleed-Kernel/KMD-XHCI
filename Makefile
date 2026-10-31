KMD_NAME    = XHCI
KMD_DEPENDS = USB

BLEED_SDK ?= $(firstword $(wildcard ../bleed-sdk ../Bleed-Kernel/bin/bleed-sdk))
ifeq ($(wildcard $(BLEED_SDK)/kmd.mk),)
$(error no bleed sdk, unpack one to ../bleed-sdk or pass BLEED_SDK=path)
endif
include $(BLEED_SDK)/kmd.mk

COMMON = ./common
FIRMWARE_MODE = APPLICATION
include $(COMMON)/common.mk

include $(BOARDDIR)/HEBI_PCBA_2121_01/board.mk

CSRC := 	$(wildcard ./src/*.c)
CPPSRC := 	$(wildcard ./src/*.cpp) \
			$(wildcard $(COMMON)/modules/*.cpp) \
			$(wildcard $(COMMON)/hardware/*.cpp) \
			$(wildcard $(COMMON)/hardware/drivers/*.cpp)

INCDIR := src $(COMMON)/hardware $(COMMON)/hardware/drivers $(COMMON)/modules

DDEFS := -DSTM32_EXTI_REQUIRED \
		 -DMFS_CFG_MEMORY_ALIGNMENT=8 
#-DCH_USE_MUTEXES=1 -DLWIP_THREAD_STACK_SIZE=2048 -DPORT_INT_REQUIRED_STACK=128 -DCHIBIOS3 -DBASE_BOARD_H=$(BASE_BOARD_H) -DBOARD_CLASS=$(BOARD_CLASS) -DELEC_VERSION_$(REV)
UDEFS :=


ELECTRICAL_TYPE := BIB_B
BOARD_TYPE := BIB

DDEFS += $(COMMON_DDEFS)
CSRC += $(COMMON_CSRC)
CPPSRC += $(COMMON_CPPSRC)
INCDIR += $(COMMON_INC)


##############################################################################
# Common rules
# The stuff that actually compiles everything!

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################
# User Test
#------------------------------------
APP              = test
# App sources
APP_SRCS         = main.c px4flow/flow.c px4flow/motion_histogram.c
# App includes
APP_INC	         = .
# Compiler flags
APP_CFLAGS       =
# Linker flags
APP_LDFLAGS      =

# Custom linker
APP_LINK_SCRIPT  =

COMMON_CFLAGS = -O3 -g

include $(RULES_DIR)/pmsis_rules.mk

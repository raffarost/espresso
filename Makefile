#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := espresso
PROJECT_VER := 2.0.1

# Add RainMaker components and other common application components
EXTRA_COMPONENT_DIRS += $(PROJECT_PATH)/../../components $(PROJECT_PATH)/../common

include $(IDF_PATH)/make/project.mk

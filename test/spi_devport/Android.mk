ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_C_INCLUDES :=

LOCAL_SRC_FILES:= main.c 
LOCAL_CFLAGS += -Wall -Wextra

LOCAL_MODULE := akm_spi_dev
LOCAL_MODULE_TAGS := eng
LOCAL_FORCE_STATIC_EXECUTABLE := false
LOCAL_STATIC_LIBRARIES :=
LOCAL_SHARED_LIBRARIES := libc libcutils
include $(BUILD_EXECUTABLE)

endif  # TARGET_SIMULATOR != true


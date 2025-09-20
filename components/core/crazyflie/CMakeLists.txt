idf_component_register(SRCS "./hal/src/ledseq.c"
                "./hal/src/usec_time.c"
                "./modules/src/system.c"
                "./utils/src/cfassert.c"
                "./utils/src/cpuid.c"
                "./utils/src/debug_cf.c"
                "./utils/src/eprintf.c"
                "./utils/src/sleepus.c"
                "./utils/src/version.c"
                INCLUDE_DIRS "./hal/interface" "./modules/interface" "./utils/interface"
                REQUIRES platform config led motors gcs_comm wifi esp_timer)

idf_component_get_property( FREERTOS_ORIG_INCLUDE_PATH freertos ORIG_INCLUDE_PATH)
target_include_directories(${COMPONENT_TARGET} PUBLIC
  "${FREERTOS_ORIG_INCLUDE_PATH}"
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-fno-strict-aliasing"
                                                "-Wno-error=stringop-truncation"
                                                "-Wno-error=stringop-overflow"
                                                "-Wno-absolute-value"
                                                "-Wno-address-of-packed-member"
                                                "-Wno-stringop-truncation"
                                                "-Wno-format")
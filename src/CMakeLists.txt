add_custom_command(
        OUTPUT KO_FILE
        COMMAND make ARGS CROSS=${COMPILER} KERNEL=${KERNEL_SOURCES}
        COMMAND chmod ARGS 777 lora.ko
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        USES_TERMINAL
        VERBATIM
)

add_custom_target(KernelModul ALL DEPENDS KO_FILE)

add_library(dummy.ko src/lora.c)

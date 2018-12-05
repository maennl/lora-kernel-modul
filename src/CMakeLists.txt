add_custom_command(
        OUTPUT KO_FILE
        COMMAND make ARGS CROSS=${COMPILER} KERNEL=${KERNEL_SOURCES}
        COMMAND chmod ARGS 777 lora_module.ko
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        USES_TERMINAL
        VERBATIM
)

add_custom_target(KernelModul ALL DEPENDS KO_FILE)

add_custom_command(
        OUTPUT KO_CLEAN
        COMMAND make ARGS CROSS=${COMPILER} KERNEL=${KERNEL_SOURCES} clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        USES_TERMINAL
        VERBATIM
)

add_custom_target(KernelModul-Clean ALL DEPENDS KO_CLEAN)

file( GLOB SRCS src/*.c src/*.h )
add_library(dummy.ko ${SRCS})

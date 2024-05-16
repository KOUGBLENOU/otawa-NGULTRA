set(ELF "matmult.elf")

add_custom_command(
    OUTPUT "owcet.out"
    COMMAND owcet -s ultra ${ELF}
    DEPENDS ${ELF}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating owcet.out"
)

add_custom_target(run_owcet
    DEPENDS "owcet.out"
)

add_custom_target(test ALL
    DEPENDS run_owcet
)

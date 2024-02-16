idf_component_register(
                            SRCS main.c)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

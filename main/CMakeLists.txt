set(srcs 
    "main.c"
    "display.c"
    "horizon-display.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")

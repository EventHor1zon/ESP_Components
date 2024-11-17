set(srcs
    "Utilities.c"
    "framebuffer.c"
)

set(includes "inc/")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "inc"
                    )

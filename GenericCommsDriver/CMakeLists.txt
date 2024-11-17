set(srcs
    "GenericCommsDriver.c"   
)

set(includes "inc/")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
                    REQUIRES "driver" "GenericCommsDriver"
)

set(srcs
    "PeripheralManager.c"   
    "CommandAPI.c"
)

set(includes "inc/")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS ${includes}
REQUIRES "esp_event" "driver" "GenericCommsDriver" "CBuffer" 
)

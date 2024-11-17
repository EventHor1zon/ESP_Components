
#include "esp_types.h"
#include "inc/CommandAPI.h"

uint8_t get_param_data_size(datatype_t t) {
    uint8_t sz = 0;
    
    switch (t) {
        case DATATYPE_NONE:
            sz = 0;
            break;
        case DATATYPE_BOOL:
        case DATATYPE_STRING:
            sz = sizeof(bool);
            break;
        case DATATYPE_INT8:
        case DATATYPE_UINT8:
            sz = sizeof(uint8_t);
            break;
        case DATATYPE_INT16:
        case DATATYPE_UINT16:
            sz = sizeof(uint16_t);
            break;
        case DATATYPE_INT32:
        case DATATYPE_UINT32:
            sz = sizeof(uint32_t);
            break;
        case DATATYPE_FLOAT:
            sz = sizeof(float);
            break;
        case DATATYPE_DOUBLE:
            sz = sizeof(double);
            break;
        default:
            sz = 0;
            break;
    }
    
    return sz;
}
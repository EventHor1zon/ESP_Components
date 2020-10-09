/***************************************
* \file      PeripheralManager.c
* \brief     This API mangs the components, and communicates with the 
*            API_Manager. 
* \date     Sept 2020
* \author   RJAM
****************************************/

/********* Includes *******************/
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "../inc/PeripheralManager.h"
#include "../inc/CommandAPI.h"

#include "BME280_Driver.h"

/**
 *      The plan - 
 *          Can pass in components from a config file?
 *          An init phase - initialise all compoonents
 *          initialise the PM - 
 *                         incomming message queue
 *                         outgoing message queue
 *                         task
 *           task - wait on incomming messages              
 *                - call approriate function      
 *                - return the requested data
 *                - wait for more messages
 *          
 *          problems - how to capture functionality from peripherals?
 *                   - how to expose functionality to interface?
 *                   - simple get/set interface?    
 ***/

/**  TESTING: incomming command from api manger **/

const char *incommingCommand = "{\"command_type: 1\",\n\"command_id\" : 4,\n\"command_value\" : 2,}";
const char *PM_TAG = "PERIPHERAL_MANAGER";

/****** Function Prototypes ***********/

/************ ISR *********************/

/****** Private Data ******************/

static peripheral_t *peripherals[PM_MAX_PERIPHERALS];
static uint8_t peripheral_num = 0;
/****** Private Functions *************/

/****** Global Data *******************/

/****** Global Functions *************/

static peripheral_t *get_peripheral_from_index(uint8_t index)
{

    peripheral_t *p = NULL;
    if (index > peripheral_num)
    {
        return p;
    }
    else
    {
        p = peripherals[index];
    }

    return p;
}

static peripheral_t *get_peripheral_from_id(uint32_t periph_id)
{
    peripheral_t *periph = NULL;

    for (int i = 0; i < peripheral_num; i++)
    {
        if (peripherals[i] == NULL)
        {
            break;
        }
        else if (peripherals[i]->peripheral_id == periph_id)
        {
            periph = peripherals[i];
            break;
        }
    }

    return periph;
}

static parameter_t *get_parameter_from_id(peripheral_t *periph, uint32_t param_id)
{
    parameter_t *param = NULL;

    for (int i = 0; i < periph->param_len; i++)
    {
        if (periph->params[i].param_id == param_id)
        {
            param = &(periph->params[i]);
            break;
        }
    }

    return param;
}

static action_t *get_action_from_id(peripheral_t *periph, uint32_t action_id)
{
    action_t *act = NULL;
    for (int i = 0; i < periph->actions_len; i++)
    {
        if (periph->actions[i].action_id == action_id)
        {
            act = &(periph->actions[i]);
            break;
        }
    }
    return act;
}

static void pmTask(void *args)
{

    cmd_request_t incomming = {0};
    cmd_rsp_t outgoing = {0};

    while (1)
    {

        while (commandq == NULL)
        {
            vTaskDelay(1000);
        }

        xQueueReceive(commandq, &incomming, portMAX_DELAY);
        outgoing = pm_process_incoming_request(&incomming);
        xQueueSendToBack(respondq, &outgoing, PM_QUEUE_SEND_TIMEOUT);
    }
}

void pm_craft_error_response(uint8_t error_code, char *err_msg, uint32_t uid, cmd_rsp_t *rsp)
{
    rsp->rsp_type = RSP_TYPE_ERR;
    rsp->rsp_uid = uid;
    uint8_t len = strlen(err_msg);
    strncpy(rsp->rsp_data.ersp_data.err_message, err_msg, len);
    rsp->rsp_data.ersp_data.error_code = error_code;
    return;
}

void pm_craft_ok_response(uint32_t uid, cmd_rsp_t *rsp)
{
    rsp->rsp_uid = uid;
    rsp->rsp_type = RSP_TYPE_OK;
    rsp->rsp_data.reserved = 0;
    return;
}

void pm_craft_data_response(uint32_t uid, parameter_t *param_data, uint32_t data, cmd_rsp_t *rsp)
{
    rsp->rsp_uid = uid;
    rsp->rsp_type = RSP_TYPE_DATA;
    rsp->rsp_data.drsp_data.data = data;
    rsp->rsp_data.drsp_data.param_id = param_data->param_id;
    rsp->rsp_data.drsp_data.type = param_data->valueType;
}

// void pm_craft_prminfo_response(uint32_t uid, uint32_t pid, cmd_rsp_t *rsp)
// {
//     rsp->
// }

void pm_craft_pinfo_num_response(uint32_t uid, uint8_t num, cmd_rsp_t *rsp)
{
    rsp->rsp_data.drsp_data.data = num;
    rsp->rsp_data.drsp_data.param_id = 0;
    rsp->rsp_data.drsp_data.type = 0;
    rsp->rsp_uid = uid;
    rsp->rsp_type = RSP_TYPE_DATA;
}

// void pm_craft_pinfo_response(uint32_t uid, peripheral_t *p, cmd_rsp_t *rsp)
// {
//     rsp->rsp_type = RSP_TYPE_DATA;
//     rsp->rsp_uid = uid;
// }

esp_err_t pm_execute_direct_cmd()
{
    return ESP_OK;
}

cmd_rsp_t pm_process_incoming_request(cmd_request_t *request)
{

    esp_err_t status = ESP_OK;
    esp_err_t cmd_status = ESP_OK;
    cmd_rsp_t response = {0};
    uint32_t rsp_data = 0;

    if (request->cmd_type == CMD_TYPE_PINFO)
    {

        periph_info_t pinfo = request->cmd_data.lcmd_data;
        peripheral_t *periph;

        if (pinfo.periph_id == PM_PIFO_NUM_TYPE)
        {
            pm_craft_pinfo_num_response(request->cmd_uid, peripheral_num, &response);
        }
        else
        {
            periph = get_peripheral_from_id(pinfo.periph_id);
            if (periph == NULL)
            {

                pm_craft_error_response(0, "Invalid Peripheral ID", request->cmd_uid, &response);
            }
            else
            {
                //pm_craft_pinfo_response();
            }
        }
    }
    else if (request->cmd_type == CMD_TYPE_SINFO)
    {
        //pm_craft_sinfo_response();
    }
    else if (request->cmd_type == CMD_TYPE_PCMD)
    {
        /** the command to call depends on : command type. param_type **/

        periph_cmd_t pcmd = request->cmd_data.pcmd_data;
        peripheral_t *periph = get_peripheral_from_id(pcmd.periph_id);
        if (periph == NULL)
        {
            pm_craft_error_response(PM_ERR_INVALID_ID, "Invalid Peripheral ID", request->cmd_uid, &response);
        }
        else
        {
            // GET //
            if (pcmd.pcmd_type == PCMD_TYPE_GET)
            {
                parameter_t *param = get_parameter_from_id(periph, pcmd.param_id);
                if (param == NULL)
                {
                    pm_craft_error_response(PM_ERR_INVALID_ID, "Invalid Param ID", request->cmd_uid, &response);
                }
                else if (param->get == NULL)
                {
                    pm_craft_error_response(PM_ERR_INVALID_CMD, "Param not gettable", request->cmd_uid, &response);
                }
                else
                {
                    uint32_t data = 0;
                    cmd_status = param->get(periph->handle, &data);
                    if (cmd_status != ESP_OK)
                    {
                        pm_craft_error_response(cmd_status, "Get Error", request->cmd_uid, &response);
                    }
                    else
                    {
                        pm_craft_data_response(param->param_id, param, data, &response);
                    }
                }
            }
            // SET //
            else if (pcmd.pcmd_type == PCMD_TYPE_SET)
            {
                parameter_t *param = get_parameter_from_id(periph, pcmd.param_id);
                if (param == NULL)
                {
                    pm_craft_error_response(PM_ERR_INVALID_ID, "Invalid Param ID", request->cmd_uid, &response);
                }
                else if (param->set == NULL)
                {
                    pm_craft_error_response(PM_ERR_INVALID_CMD, "Param not settable", request->cmd_uid, &response);
                }
                else
                {
                    if (pcmd.ptype == PARAMTYPE_UINT8)
                    {
                        uint8_t data8 = (uint8_t)pcmd.data;
                        cmd_status = param->set(periph->handle, &data8);
                    }
                    else if (pcmd.ptype == PARAMTYPE_UINT16)
                    {

                        uint16_t data16 = (uint16_t)pcmd.data;
                        cmd_status = param->set(periph->handle, &data16);
                    }
                    else if (pcmd.ptype == PARAMTYPE_UINT32)
                    {
                        uint32_t data32 = pcmd.data;
                        cmd_status = param->set(periph->handle, &data32);
                    }
                    else if (pcmd.ptype == PARAMTYPE_FLOAT)
                    {
                        float dataF = (float)pcmd.data;
                        cmd_status = param->set(periph->handle, &dataF);
                    }
                    else
                    {
                    }
                }

                if (cmd_status != ESP_OK)
                {
                    pm_craft_error_response(cmd_status, "Get Error", request->cmd_uid, &response);
                }
                else
                {
                    pm_craft_ok_response(param->param_id, &response);
                }
            }

            // ACTION //
            else if (pcmd.pcmd_type == PCMD_TYPE_ACT)
            {
                /** Get the action **/
                action_t *act = get_action_from_id(periph, pcmd.param_id);
                if (act == NULL)
                {
                    pm_craft_error_response(PM_ERR_INVALID_ID, "Invalid action ID", request->cmd_uid, &response);
                }
                else
                {
                    /** run the command **/
                    cmd_status = act->action(periph->handle);
                    if (cmd_status != ESP_OK)
                    {
                        pm_craft_error_response(cmd_status, "Action Error", request->cmd_uid, &response);
                    }
                    else
                    {
                        pm_craft_ok_response(act->action_id, &response);
                    }
                }
            }
            else
            {
                pm_craft_error_response(PM_ERR_INVALID_TYPE, "Action Error", request->cmd_uid, &response);
            }
        }
    }
    else if (request->cmd_type == CMD_TYPE_DIRECT)
    {
        //status = pm_execute_direct_cmd();
        //pm_craft_dcmd_response();
    }
    else
    {
        pm_craft_error_response(PM_ERR_INVALID_CMD, "Invalid Command type", request->cmd_uid, &response);
    }

    return response;
}

esp_err_t peripheral_manager_init()
{
    esp_err_t initStatus = ESP_OK;

    /** peripheral init code goes here **/
    /** use a random peripheral to check process **/

    bm_initData_t bme = {0};
    bme.devType = BME_280_DEVICE;
    bme.addressPinState = 0;
    bme.sampleMode = BM_FORCE_MODE;
    bme.sampleType = BM_MODE_TEMP_PRESSURE_HUMIDITY;
    bme.i2cChannel = PM_I2C_BUS_PRIMARY;

    bm_controlData_t *bmHandle = bm280_init(&bme);

    peripheral_t *bmeP = heap_caps_calloc(1, sizeof(peripheral_t), MALLOC_CAP_DEFAULT);
    bmeP->handle = bmHandle;
    bmeP->ptype = PTYPE_ENVIRO_SENSOR;
    bmeP->stype = STYPE_ENVIRO_SENSOR_BME_290;
    bmeP->actions = bm_action_mappings;
    bmeP->actions_len = bm_action_len;
    bmeP->params = bm_param_mappings;
    bmeP->param_len = bm_param_len;
    bmeP->peripheral_id = (uint32_t)(bmeP->ptype << 16) | (uint32_t)(bmeP->stype << 8) | (uint32_t)peripheral_num;

    peripherals[peripheral_num] = bmeP;
    peripheral_num++;

    respondq = NULL;
    respondq = xQueueCreate(PM_MAX_QUEUE_LEN, sizeof(cmd_rsp_t));

    if (respondq == NULL)
    {
        ESP_LOGE(PM_TAG, "Error creating response queue");
        initStatus = ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(pmTask, "pmTask", 5012, NULL, 2, NULL) != pdTRUE)
    {
        ESP_LOGE(PM_TAG, "Error creating control task");
        initStatus = ESP_ERR_NO_MEM;
    }

    return initStatus;
}
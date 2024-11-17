/***************************************
 * \file      PeripheralManager.c
 * \brief     This API mangs the components, and communicates with the
 *            API_Manager.
 * \date     Sept 2020
 * \author   RJAM
 ****************************************/

/********* Includes *******************/
#include "CommandAPI.h"
#include "GenericCommsDriver.h"
#include "PeripheralManager.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include <string.h>
#if CONFIG_ENABLE_STREAM
#include "StreamComponent.h"
#endif
#include "CircularBuffer.h"

/******** Global Data **********/

const char *PM_TAG = "PERIPHERAL_MANAGER";
/** TODO: Define this somewhere better **/
const char *DEVNAME = "Led-Bunny";
const uint8_t DEVICE_ID = 0x90;

static bool is_init = false;

QueueHandle_t command_queue; /** < Handle for the main command queue **/
QueueHandle_t respondq;      /** < Handle for the main respond queue **/

#ifdef CONFIG_USE_EVENTS

ESP_EVENT_DEFINE_BASE(PM_EVENT_BASE);

event_map_t *event_map = NULL;
esp_event_loop_handle_t pm_event_loop = NULL;
esp_event_handler_instance_t pm_event_instance = NULL;

void pm_event_handler(void *args, esp_event_base_t base, int32_t id, void *event_args);

static esp_err_t event_loop_init(void);
static event_map_t *find_event_map(uint32_t id);
static cmd_rsp_t pm_handle_periph_cmd(cmd_request_t *request);

#endif

/****** Function Prototypes ***********/

/** \name           pm_is_streamable
 *  \brief          Checks a given parameter is streamable
 *  \param cmd      - pointer to the peripheral command
 *  \param param    - pointer to the parameter struct
 *  \return Boolean
 */
static bool pm_is_streamable(uint8_t periph_id, uint8_t param_id);

/** \name            set_within_limits
 *  \brief           Checks a given set data value is lte
 *                   than the maximum value allowed
 *  \param cmd      - pointer to the peripheral command
 *  \param param    - pointer to the parameter struct
 *  \return Boolean
 */
static bool set_within_limits(periph_cmd_t *cmd, parameter_t *param);

/** \name                creates an error command response
 *  \brief               Populates a command response struct
 *                       with error details
 *  \param  error_code - code associated with error
 *  \param  err_msg    - string error message
 *  \param  rsp        - the command response to populate
 *  \return ESP_OK or PM_ERROR_
 */
static void pm_create_error_response(uint32_t error_code, char *err_msg, cmd_rsp_t *rsp);

/** \name               pm_handle_param_info_request
 *  \brief              Handles a 'parameter info' request.
 *  \param request      pointer to the command request
 *  \param response     pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static void pm_handle_param_info_request(uint8_t periph_id, uint8_t param_id, cmd_rsp_t *rsp);

/** \name               pm_handle_periph_info_request
 *  \brief              Handles a peripheral info request
 *  \param request      - pointer to the command request
 *  \param response     - pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static void pm_handle_periph_info_request(uint8_t periph_id, cmd_rsp_t *rsp);

/** \name               pm_handle_device_info_request
 *  \brief              Handles a 'device info' request.
 *  \param request      - pointer to the command request
 *  \param response     - pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static void pm_handle_device_info_request(cmd_rsp_t *rsp);

/** \name               handle_get_request
 *  \brief              Handles a 'get' request. Calls the requested function to
 *                      get the data and sets the data type response fields.
 *                      If an error is encountered, the response will be populated
 *                      with an error response.
 *  \param request      - pointer to the command request
 *  \param response     - pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static esp_err_t handle_get_request(cmd_request_t *request, cmd_rsp_t *response);

/** \name               handle_set_request
 *  \brief              Handles a 'set' request. Checks maximum, calls the requested function to
 *                      set the data and sets the ack type response fields
 *                      If an error is encountered, the response will be populated
 *                      with an error response.
 *  \param request      - pointer to the command request
 *  \param response     - pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static esp_err_t handle_set_request(cmd_request_t *request, cmd_rsp_t *response);

/** \name               handle_invoke_request
 *  \brief              Handles a 'invoke' request. This request calls the
 *                      action function associated with the parameter.
 *  \param request      - pointer to the command request
 *  \param response     - pointer to the command response
 *  \return esp_ok or PM_ERROR type
 */
static esp_err_t handle_invoke_request(cmd_request_t *request, cmd_rsp_t *response);

/** \name               pm_handle_periph_cmd
 *  \brief              Handles a Peripheral command request.
 *                      Sends to the appropriate handler based on the
 *                      command type. Returns the command response -
 *                      if an error encountered during processing, the response
 *                      is the error type and contains details of the error
 *  \param request      - pointer to the command request
 *  \return populated command response
 */
static cmd_rsp_t pm_handle_periph_cmd(cmd_request_t *request);

/** \name pm_command_task
 *  \brief  PM management task
 *          Waits for a request to arrive from the command_queue
 *          then processes the command, builds the request
 *          and returns the response via the response queue
 */
static void pm_command_task(void *args);

#ifdef CONFIG_ENABLE_STREAM

static cmd_rsp_t pm_process_stream(cmd_request_t *request);

static bool pm_is_streamable(uint8_t periph_id, uint8_t param_id);

#endif

/************ ISR *********************/

/****** Private Data ******************/

static uint8_t peripheral_num = 0;

static peripheral_t peripherals[PM_MAX_PERIPHERALS];

/****** Private Functions *************/

static bool set_within_limits(periph_cmd_t *cmd, parameter_t *param)
{
    if (cmd->data_t == DATATYPE_FLOAT) {
        return (uint32_t)cmd->data.float_data <= param->max_valid;
    } else if (
        cmd->data_t == DATATYPE_INT8 || cmd->data_t == DATATYPE_INT16
        || cmd->data_t == DATATYPE_INT32)
    {
        return cmd->data.sint_data > param->max_valid;
    } else if (
        cmd->data_t == DATATYPE_INT8 || cmd->data_t == DATATYPE_INT16
        || cmd->data_t == DATATYPE_INT32 || cmd->data_t == DATATYPE_BOOL)
    {
        return cmd->data.uint_data > param->max_valid;
    } else if (cmd->data_t == DATATYPE_STRING) {
        return true;
    }

    return false;
}

static esp_err_t check_valid_periph_id(cmd_request_t *request)
{
    if (request->data.cmd_data.periph_id == 0 && request->data.cmd_data.cmd_type == CMD_TYPE_INFO) {
        /** will not return a peripheral,
         *  but this is expected for info request
         ***/
        return ESP_OK;
    }

    peripheral_t *periph = get_peripheral_from_id(request->data.cmd_data.periph_id);

    /** return lookup success **/
    return (periph == NULL ? ESP_ERR_INVALID_ARG : ESP_OK);
}

static esp_err_t check_valid_param_id(cmd_request_t *request)
{
    if (request->data.cmd_data.param_id == 0 && request->data.cmd_data.cmd_type == CMD_TYPE_INFO) {
        /** will not return a parameter,
         *  but this is expected for info request
         ***/
        return ESP_OK;
    }

    peripheral_t *periph = get_peripheral_from_id(request->data.cmd_data.periph_id);

    parameter_t *param = get_parameter_from_id(periph, request->data.cmd_data.param_id);

    /** return lookup success **/
    return (param == NULL ? ESP_ERR_INVALID_ARG : ESP_OK);
}

/************** Handler Functions *******************/

static void pm_create_error_response(uint32_t error_code, char *err_msg, cmd_rsp_t *rsp)
{
    size_t len;
    len = strlen(err_msg);
    // leave space for the string terminator
    len = (len > CMD_API_ERR_MAX_LEN - 1) ? CMD_API_ERR_MAX_LEN - 1 : len;
    strncpy(&rsp->rsp_data.error.err_message[0], err_msg, len);
    rsp->rsp_data.error.error_code = error_code;
    rsp->rsp_type = RSP_TYPE_ERR;
    return;
}

static void pm_handle_param_info_request(uint8_t periph_id, uint8_t param_id, cmd_rsp_t *rsp)
{
    peripheral_t *periph;
    parameter_t *param;
    uint32_t len;

    periph = get_peripheral_from_id(periph_id);
    /** should only call this function with a valid
     *  periph_id - if periph is null set error response & return.
     **/
    if (periph == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid peripheral id", rsp);
        return;
    }

    param = get_parameter_from_id(periph, param_id);

    if (param == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PARAM_ID, "Invalid parameter id", rsp);
        return;
    }

    rsp->rsp_type = RSP_TYPE_PARAM_INFO;
    rsp->rsp_data.param_info.cmd_type = param->types;
    rsp->rsp_data.param_info.param_id = param->param_id;
    rsp->rsp_data.param_info.max_value = param->max_valid;
    rsp->rsp_data.param_info.value_type = param->value_type;
    rsp->rsp_data.param_info.periph_id = periph->peripheral_id;

    len = strlen(param->param_name);

    len = (len > PM_MAX_PARAM_NAME_LENGTH - 1 ? PM_MAX_PARAM_NAME_LENGTH - 1 : len);
    strncpy(&rsp->rsp_data.param_info.name[0], param->param_name, len);

    return;
}

static void pm_handle_periph_info_request(uint8_t periph_id, cmd_rsp_t *rsp)
{
    peripheral_t *periph;
    uint32_t len;
    periph = get_peripheral_from_id(periph_id);

    /** should only call this function with a valid
     *  periph_id - if periph is null set error response & return.
     **/
    if (periph == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid peripheral id", rsp);
        return;
    }

    rsp->rsp_type = RSP_TYPE_PERIPH_INFO;
    rsp->rsp_data.periph_info.periph_id = periph->peripheral_id;
    rsp->rsp_data.periph_info.periph_type = periph->periph_type;
    rsp->rsp_data.periph_info.param_num = periph->param_len;

    len = strlen(periph->peripheral_name);
    len = (len > PM_MAX_PERIPH_NAME_LENGTH - 1 ? PM_MAX_PERIPH_NAME_LENGTH - 1 : len);
    strncpy(&rsp->rsp_data.periph_info.name[0], periph->peripheral_name, len);

    for (uint8_t i = 0; i < periph->param_len; i++) {
        rsp->rsp_data.periph_info.param_ids[i] = periph->params[i].param_id;
    }

    return;
}

static void pm_handle_device_info_request(cmd_rsp_t *rsp)
{
    rsp->rsp_type = RSP_TYPE_DEV_INFO;
    rsp->rsp_data.dev_info.num_periphs = peripheral_num;
    rsp->rsp_data.dev_info.device_id = DEVICE_ID;
    strncpy(&rsp->rsp_data.dev_info.name[0], DEVNAME, strlen(DEVNAME));
    for (uint8_t i = 0; i < peripheral_num; i++) {
        rsp->rsp_data.dev_info.periph_ids[i] = peripherals[i].peripheral_id;
    }
    return;
}

static esp_err_t handle_get_request(cmd_request_t *request, cmd_rsp_t *response)
{
    esp_err_t cmd_status = ESP_OK;
    parameter_t *param;
    peripheral_t *periph;

    /** load the peripheral/parameter objects **/
    periph = get_peripheral_from_id(request->data.cmd_data.periph_id);

    if (periph == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid Peripheral Id", response);
        return PM_ERR_INVALID_PERIPH_ID;
    }

    param = get_parameter_from_id(periph, request->data.cmd_data.param_id);

    if (param == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PARAM_ID, "Invalid Parameter Id", response);
        return PM_ERR_INVALID_PARAM_ID;
    }

    if (param->get == NULL) {
        pm_create_error_response(PM_ERR_INVALID_METHOD, "Param is not Gettable", response);
        return PM_ERR_INVALID_ARG;
    }

    switch (param->value_type) {
        case DATATYPE_FLOAT:
            cmd_status = param->get(
                (handle_t *)periph->handle,
                &response->rsp_data.data.data.float_data);

            if (cmd_status == ESP_OK) {
                ESP_LOGI("API", "Got %f", response->rsp_data.data.data.float_data);
            }
            break;
        case DATATYPE_STRING:
            cmd_status = param->get(
                (handle_t *)periph->handle,
                &response->rsp_data.data.data.str_data[0]);
            if (cmd_status == ESP_OK) {
                ESP_LOGI("API", "Got %f", response->rsp_data.data.data.float_data);
            }
            break;
        /** We can store any of the below into a uint32 **/
        case DATATYPE_BOOL:
        case DATATYPE_UINT8:
        case DATATYPE_UINT16:
        case DATATYPE_UINT32:
            cmd_status = param->get(
                (handle_t *)periph->handle,
                &response->rsp_data.data.data.uint_data);
            break;
        /** We can store any of the below into an int32.
         *  Hell we could use the uint32, but saves remembering to
         *  typecast properly down the line.
         *  **/
        case DATATYPE_INT8:
        case DATATYPE_INT16:
        case DATATYPE_INT32:
            cmd_status = param->get(
                (handle_t *)periph->handle,
                &response->rsp_data.data.data.sint_data);
            break;
        default: ESP_LOGE(PM_TAG, "Error: Unrecognised data type"); break;
    }

    if (cmd_status != ESP_OK) {
        pm_create_error_response((PM_ERR_GET_FAILED_BASE + cmd_status), "Get Error", response);
        cmd_status = PM_ERR_GET_FAILED_BASE;
    } else {
        /** craft the rest of the get response */
        response->rsp_type = RSP_TYPE_DATA;
        response->rsp_data.data.data_t = param->value_type;
        response->rsp_data.data.param_id = param->param_id;
        response->rsp_data.data.periph_id = periph->peripheral_id;
    }

    return cmd_status;
}

static esp_err_t handle_set_request(cmd_request_t *request, cmd_rsp_t *response)
{
    esp_err_t cmd_status;
    parameter_t *param;
    peripheral_t *periph;
    periph_cmd_t *cmd = &request->data.cmd_data;
    /** load the peripheral/parameter objects **/
    periph = get_peripheral_from_id(request->data.cmd_data.periph_id);

    if (periph == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid Peripheral Id", response);
        return PM_ERR_INVALID_PERIPH_ID;
    }

    param = get_parameter_from_id(periph, request->data.cmd_data.param_id);

    if (param == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PARAM_ID, "Invalid Parameter Id", response);
        return PM_ERR_INVALID_PARAM_ID;
    }

    if (param->set == NULL) {
        pm_create_error_response(PM_ERR_INVALID_CMD, "Param not settable", response);
        return PM_ERR_INVALID_ARG;
    }

    if (request->data.cmd_data.data_t != DATATYPE_STRING
        && !set_within_limits(&request->data.cmd_data, param))
    {
        pm_create_error_response(PM_ERR_SET_OUT_OF_BOUNDS, "Value greater than max", response);
    }

    switch (cmd->data_t) {
        case DATATYPE_BOOL:
        case DATATYPE_UINT8:
        case DATATYPE_UINT16:
        case DATATYPE_UINT32:
            cmd_status = param->set((handle_t *)periph->handle, (void *)&cmd->data.uint_data);
            break;
        case DATATYPE_INT8:
        case DATATYPE_INT16:
        case DATATYPE_INT32:
            cmd_status = param->set((handle_t *)periph->handle, (void *)&cmd->data.sint_data);
            break;
        case DATATYPE_FLOAT:
            cmd_status = param->set((handle_t *)periph->handle, (void *)&cmd->data.float_data);
            break;
        case DATATYPE_STRING:
            cmd_status = param->set((handle_t *)periph->handle, (void *)&cmd->data.str_data[0]);
            break;
        default:
            ESP_LOGI(PM_TAG, "Unknown data type");
            pm_create_error_response(PM_ERR_INVALID_TYPE, "Invalid data type", response);
            cmd_status = ESP_ERR_INVALID_ARG;
    }

    if (cmd_status == ESP_OK) {
        response->rsp_type = RSP_TYPE_ACK;
        response->rsp_data.ack.opt = 1;
    }

    return cmd_status;
}

static esp_err_t handle_invoke_request(cmd_request_t *request, cmd_rsp_t *response)
{
    esp_err_t cmd_status;
    parameter_t *param;
    peripheral_t *periph;
    /** load the peripheral/parameter objects **/
    periph = get_peripheral_from_id(request->data.cmd_data.periph_id);

    if (periph == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid Peripheral Id", response);
        return PM_ERR_INVALID_PERIPH_ID;
    }

    param = get_parameter_from_id(periph, request->data.cmd_data.param_id);

    if (param == NULL) {
        pm_create_error_response(PM_ERR_INVALID_PARAM_ID, "Invalid Parameter Id", response);
        return PM_ERR_INVALID_PARAM_ID;
    }

    if (param->act == NULL) {
        pm_create_error_response(PM_ERR_INVALID_CMD, "Param not action", response);
        return PM_ERR_INVALID_METHOD;
    } else {
        cmd_status = param->act((handle_t *)periph->handle);
    }

    if (cmd_status != ESP_OK) {
        pm_create_error_response((PM_ERR_ACT_FAILED_BASE + cmd_status), "Action Error", response);
    } else {
        response->rsp_type = RSP_TYPE_ACK;
        response->rsp_data.ack.opt = 1;
    }

    return cmd_status;
}

static cmd_rsp_t pm_handle_periph_cmd(cmd_request_t *request)
{
    esp_err_t err = ESP_OK;
    esp_err_t status = ESP_OK;
    esp_err_t cmd_status = ESP_OK;

    periph_cmd_t cmd = request->data.cmd_data;

    cmd_rsp_t command_response = {0};

    /** check the periph/param ids vs request types **/
    if (check_valid_periph_id(request) != ESP_OK) {
        pm_create_error_response(PM_ERR_INVALID_PERIPH_ID, "Invalid Periph Id", &command_response);
        err = PM_ERR_INVALID_PERIPH_ID;
    } else if (check_valid_param_id(request) != ESP_OK) {
        pm_create_error_response(PM_ERR_INVALID_PARAM_ID, "Invalid Param Id", &command_response);
        err = PM_ERR_INVALID_PARAM_ID;
    }

    if (err == ESP_OK) {
        /** process the command **/
        switch (cmd.cmd_type) {
            case CMD_TYPE_INFO:
                if (cmd.periph_id == 0) {
                    /* device info request */
                    pm_handle_device_info_request(&command_response);
                } else if (cmd.param_id == 0) {
                    /* peripheral info request */
                    pm_handle_periph_info_request(cmd.periph_id, &command_response);
                } else {
                    pm_handle_param_info_request(cmd.periph_id, cmd.param_id, &command_response);
                }
                break;

            case CMD_TYPE_GET: err = handle_get_request(request, &command_response); break;

            case CMD_TYPE_SET: err = handle_set_request(request, &command_response); break;

            case CMD_TYPE_ACT: err = handle_invoke_request(request, &command_response); break;

            default:
                pm_create_error_response(PM_ERR_INVALID_CMD, "Invalid cmd type", &command_response);
                break;
        }
    }

    ESP_LOGI(
        PM_TAG,
        "Crafted response (uid: %u) type (%u)",
        command_response.rsp_uid,
        command_response.rsp_type);

    return command_response;
}

static void pm_command_task(void *args)
{
    cmd_request_t incomming = {0};
    cmd_rsp_t outgoing = {0};
    ESP_LOGI(PM_TAG, "Starting PM Task...");

    while (command_queue == NULL) {
        /** wait for the command queue if not initialied **/
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1) {
        /** wait forever for incomming commands **/
        xQueueReceive(command_queue, &incomming, portMAX_DELAY);

        /** process peripheral command **/
        if (incomming.cmd_type == REQ_PKT_TYPE_PERIPH_CMD) {
            outgoing = pm_handle_periph_cmd(&incomming);
        }

        /** process stream - in development **/
        else if (incomming.cmd_type == REQ_PKT_TYPE_STREAM)
        {
#ifdef CONFIG_ENABLE_STREAM
            outgoing = pm_process_stream(&incomming);
#else
            pm_create_error_response(0, "Stream not supported", &outgoing);
            outgoing.rsp_uid = incomming.cmd_uid;
#endif
        } else {
            pm_create_error_response(0, "Invalid command type", &outgoing);
            outgoing.rsp_uid = incomming.cmd_uid;
        }

        /** send the response to API manager **/
        ESP_LOGI(PM_TAG, "Sending response (type: %u)", outgoing.rsp_type);

        outgoing.rsp_args = incomming.rsp_args;

        xQueueSendToBack(respondq, &outgoing, PM_QUEUE_SEND_TIMEOUT);
    }

    /** Here be dragons **/
}

#ifdef CONFIG_ENABLE_STREAM

static bool pm_is_streamable(uint8_t periph_id, uint8_t param_id)
{
    bool is_streamable = false;
    ESP_LOGI(PM_TAG, "Checking if streamable");
    peripheral_t *p = get_peripheral_from_id(periph_id);
    if (p != NULL) {
        parameter_t *prm = get_parameter_from_id(p, param_id);
        if (prm != NULL) {
            is_streamable = (prm->types & STREAM_FLAG) > 0 ? 1 : 0;
        } else {
            ESP_LOGE(PM_TAG, "Error: Invalid param id");
        }
    } else {
        ESP_LOGE(PM_TAG, "Error: Invalid peripheral id");
    }

    return is_streamable;
}

static cmd_rsp_t pm_process_stream(cmd_request_t *request)
{
    /** TODO: Process stream requests - set up the neccesary
     * buffers & init the stream task.
     *
     * Make sure to track when streaming - there will be different types of stream requests -
     *  start,
     *  end,
     *  OK/continue/ping/pong - these should be dealt with by the API Manager
     *
     * TODO: Make the stream handle pointer a constant in StreamComponent.h?
     *       This would give visibility to both PM & AM
     *       Plus could display the current stream status in the driver handle
     * TODO: Make a PM function to initialise this stream interface -
     *          Create then sleep the task.
     *          Asign the memory for the stream_handle
     *          When new stream, api manager checks if already streaming
     *          if not, zeros the struct, then creates a new stream
     * TODO: If streaming enabled (Set a Def flag) then this function is called from PM init
     * TODO: Create a stream state machine!!!
     *       Great opportuniy AND good practice!
     *
     **/

    stream_cmd_t scmd = request->data.strm_data;
    esp_err_t err = ESP_OK;
    stream_handle_t *shandle = NULL;
    uint8_t param_ids[6] = {0};

    ESP_LOGI(
        PM_TAG,
        "Got a stream request rate %u num_params: %u periph_id: %u",
        scmd.rate,
        scmd.param_num,
        scmd.periph_id);

    /** Check the parameters are streamable **/
    for (uint8_t i = 0; i < scmd.param_num; i++) {
        printf("Checking param id %u", scmd.param_ids[i]);
        if (!pm_is_streamable(scmd.periph_id, scmd.param_ids[i])) {
            pm_create_error_response(100, "Error: Parameter is not streamable", &response);
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    /** check rate **/
    if (err == ESP_OK && scmd.rate >= STREAM_DRATE_END) {
        pm_create_error_response(100, "Error: Invalid stream rate", &response);
        err = ESP_ERR_INVALID_RESPONSE;
    }

    if (!err) {
        memcpy(param_ids, scmd.param_ids, (sizeof(uint8_t) * 6));
        err = start_new_stream(scmd.param_num, scmd.rate, scmd.periph_id, param_ids);
        if (err) {
            pm_create_error_response(100, "Error creating stream", &response);
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }

    if (!err) {
        pm_craft_ok_response(&response);
    }

    cmd_rsp_t cmd_rsp = {0};
    cmd_rsp.rsp_uid = request->cmd_uid;
    cmd_rsp.rsp_data = response;

    return cmd_rsp;
}
#endif

#ifdef CONFIG_USE_EVENTS

void pm_event_handler(void *args, esp_event_base_t base, int32_t id, void *event_args)
{
    uint32_t val = 0;

    event_map_t *evt = find_event_map(id);

    if (evt != NULL) {
        ESP_LOGI(PM_TAG, "Event ID matched - %lu", evt->event_id);
        if (evt->act != NULL) {
            evt->act(evt->handle);
        }
        if (evt->get != NULL) {
            evt->get(evt->handle, (void *)&val);
        }
        if (evt->set != NULL) {
            val = *(uint32_t *)event_args;
            evt->set(evt->handle, (void *)&val);
        }
    } else {
        ESP_LOGI(PM_TAG, "No event found");
    }
    return;
}

static esp_err_t event_loop_init(void)
{
    esp_err_t err = ESP_OK;

    esp_event_loop_args_t loop_args = {
        .queue_size = 5,
        .task_core_id = 0,
        .task_name = "pm_event_loop",
        .task_priority = 3,
        .task_stack_size = 5012,
    };

    err = esp_event_loop_create(&loop_args, &pm_event_loop);

    if (!err) {
        err = esp_event_handler_instance_register_with(
            pm_event_loop,
            PM_EVENT_BASE,
            ESP_EVENT_ANY_ID,
            pm_event_handler,
            NULL,
            &pm_event_instance);
    }

    return err;
}

static event_map_t *find_event_map(uint32_t id)
{
    /** walk the event map list - return event_map_t * or null **/
    event_map_t *search = NULL;

    if (event_map != NULL) {
        search = event_map;
        while (search != NULL) {
            if (search->event_id == id) {
                break;
            } else {
                search = (event_map_t *)search->next;
            }
        }
    }
    return search;
}

esp_err_t add_event_map(event_map_init_t *map_init)
{
    esp_err_t err = ESP_OK;
    event_map_t *map = NULL;

    if (map_init != NULL) {
        map = heap_caps_calloc(1, sizeof(event_map_t), MALLOC_CAP_DEFAULT);
        if (map == NULL) {
            err = ESP_ERR_NO_MEM;
            ESP_LOGE(PM_TAG, "Error allocating space for event map [%u]", err);
        } else {
            map->act = map_init->act;
            map->get = map_init->get;
            map->set = map_init->set;
            map->handle = map_init->handle;
            map->args = map_init->args;
            map->event_id = map_init->event_id;
            if (event_map == NULL) {
                /** if this is the first map, set event_map = map */
                event_map = map;
            } else {
                /** find the last map **/
                event_map_t *search = event_map;
                bool fin = false;
                while (!fin) {
                    if (search->next == NULL) {
                        /** reached the end of the list - add new map **/
                        search->next = map;
                        map->prev = search;
                        fin = true;
                    } else {
                        /* check next list item **/
                        search = search->next;
                    }
                }
            }
        }
    } else {
        err = ESP_ERR_INVALID_ARG;
    }

    return err;
}

esp_event_loop_handle_t pm_get_event_loop()
{
    return pm_event_loop;
}

#endif

/****** Global Functions *************/

/** get peripheral pointer from id **/
peripheral_t *get_peripheral_from_id(uint32_t periph_id)
{
    peripheral_t *periph = NULL;
    ESP_LOGI(PM_TAG, "Looking up peripheral with id: %lu)", periph_id);
    for (int i = 0; i < peripheral_num; i++) {
        ESP_LOGI(PM_TAG, "Periph id: %lu", peripherals[i].peripheral_id);

        if (peripherals[i].handle == NULL) {
            break;
        }
        if (peripherals[i].peripheral_id == periph_id) {
            periph = &peripherals[i];
            break;
        }
    }

    return periph;
}

/** get parameter from periph object and param ID **/
parameter_t *get_parameter_from_id(peripheral_t *periph, uint8_t param_id)
{
    parameter_t *param = NULL;

    for (int i = 0; i < periph->param_len; i++) {
        if (periph->params[i].param_id == param_id) {
            param = &periph->params[i];
            break;
        }
    }

    return param;
}

/** Register a new peripheral with the manager **/
esp_err_t pm_add_new_peripheral(peripheral_t *template, uint8_t id, void *handle)
{
    esp_err_t status = ESP_OK;
    peripheral_t *object;

    /** check the number of peripherals **/
    if (peripheral_num >= PM_MAX_PERIPHERALS) {
        return ESP_ERR_NO_MEM;
    }

    /** don't allow id of zero **/
    if (id == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /** select the next free peripheral slot **/
    object = &peripherals[peripheral_num];

    /** copy the template **/
    memcpy(object, template, sizeof(peripheral_t));

    object->handle = handle;
    object->peripheral_id = id;

    /** increment the peripheral counter **/
    peripheral_num++;

    ESP_LOGI(
        PM_TAG,
        "Added Peripheral %s - id: %lu Index: %u",
        object->peripheral_name,
        object->peripheral_id,
        peripheral_num - 1);

    return status;
}

/** init function **/
esp_err_t peripheral_manager_init(pm_init_t *init_data)
{
    esp_err_t initStatus = ESP_OK;

    if (is_init == true) {
        ESP_LOGE(PM_TAG, "Peripheral manager can only be initialised once");
        initStatus = ESP_ERR_INVALID_ARG;
    }

    /** peripheral init code goes here **/
    /** use a random peripheral to check process **/
    if (init_data->init_i2c) {
        initStatus = gcd_i2c_init(
            init_data->sda,
            init_data->scl,
            init_data->i2c_speed,
            init_data->i2c_channel,
            false);
        if (initStatus != ESP_OK) {
            ESP_LOGE(PM_TAG, "Error intialising I2C bus");
        }
    }

    if (init_data->init_spi) {
        initStatus = gcd_spi_init(
            init_data->sck,
            init_data->mosi,
            init_data->miso,
            init_data->spi_channel,
            false);
        if (initStatus != ESP_OK) {
            ESP_LOGE(PM_TAG, "Error intialising SPI bus");
        }
    }

    if (init_data->init_uart) {
        initStatus = gcd_uart_init(
            init_data->uart_channel,
            init_data->uart_tx,
            init_data->uart_rx,
            init_data->uart_cts,
            init_data->uart_rts,
            init_data->uart_baud,
            1,
            0);
        if (initStatus != ESP_OK) {
            ESP_LOGE(PM_TAG, "Error intialising SPI bus");
        }
    }

    if (init_data->init_gpio_isr) {
        initStatus = gpio_install_isr_service(PM_CONFIG_GPIO_ISR_PRIO);
        if (initStatus != ESP_OK) {
            ESP_LOGE(PM_TAG, "Error intialising GPIO ISR service!");
        }
    }

    respondq = NULL;
    respondq = xQueueCreate(PM_MAX_QUEUE_LEN, sizeof(cmd_rsp_t));

    if (respondq == NULL) {
        ESP_LOGE(PM_TAG, "Error creating response queue");
        initStatus = ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(pm_command_task, "pm_command_task", 5012, NULL, 6, NULL, 0)
        != pdTRUE) {
        ESP_LOGE(PM_TAG, "Error creating control task");
        initStatus = ESP_ERR_NO_MEM;
    }

#ifdef CONFIG_ENABLE_STREAM
    initStatus = stream_init();
    if (initStatus != ESP_OK) {
        ESP_LOGE(PM_TAG, "Error starting stream component");
    }
#endif

#ifdef CONFIG_USE_EVENTS
    initStatus = event_loop_init();
    if (initStatus != ESP_OK) {
        ESP_LOGE(PM_TAG, "Error starting event loop");
    }
#endif

    if (initStatus == ESP_OK) {
        is_init = true;
    }

    return initStatus;
}

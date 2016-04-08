// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

namespace Service {

class Interface;

namespace BOSS {

enum class PropertyType
{
    // Constants for referencing TaskPolicy attributes
    TASK_PRIORITY,              //
    TASK_SCHEDULING_POLICY,     //
    TASK_TARGET_DURATION,       //
    TASK_EXEC_INTERVAL,         //
    TASK_EXEC_COUNT,            //
    TASK_PERMISSION,            //

                                // Constants for referencing TaskAction attributes
    ACTION_CODE,                //
    ACTION_URL,                 //
    ACTION_OFFSET,              //
    ACTION_FILEDESC,            //
    ACTION_FILEPATH,            //
    ACTION_FILEPATH_W,          //
    ACTION_FILE_HANDLE,         //
    ACTION_HTTP_HEADER,         //
    ACTION_CLIENT_CERT,         //
    ACTION_ROOT_CA,             //
    ACTION_PRIVATE_CLIENT_CERT, //
    ACTION_PRIVATE_ROOT_CA,     //
    ACTION_AP_INFO,             //
    ACTION_CLIENT_CERT_NUM,     //
    ACTION_ROOT_CA_NUM,         //
    ACTION_LAST_MODIFIED_TIME,  //
    ACTION_SERIAL_ID,           //
    SIGNAL_TASK_EVENT,          //

                                // Constants for referencing TaskOption attributes
    TASK_EXEC_OPTION,           //
    TASK_STEP,                  //
    TASK_OPTION_TARGET_STEP,    //
    TASK_OPTION_PARAM1,         //
    TASK_OPTION_PARAM2,         //

                                // Constants for referencing TaskStatus attributes
    TASK_STATE_CODE,            //
    TASK_STATE_TASK,            //
    TASK_STATE_RESUME,          //
    TASK_RESULT_CODE,           //
    TASK_SERVICE_STATUS,        //
    TASK_SERVICE_TERMINATED,    //
    TASK_COMM_ERROR_CODE,       //
    TASK_CURRENT_PRIORITY,      //
    TASK_EXECUTE_COUNT,         //
    TASK_PENDING_TIME,          //
    TASK_REMAIN_TIME,           //
    TASK_START_TIME,            //
    TASK_STEP_START_TIME,       //
    TASK_PROGRESS,              //
    TASK_DATA_SIZE,             //
    TASK_CURRENT_STEP,          //
    TASK_ACTIVE_RATE,           //
    TASK_REQUEST_RATE,          //
    TASK_LAST_MODIFIED_TIME,    //

                                // Constants for referencing TaskError attributes
    TASK_ERROR_RESULT_CODE,     //
    TASK_ERROR_CODE,            //
    TASK_ERROR_MESSAGE,         //

                                // Constants for referencing ApplicationIdList attributes. (Enumerator used during internal processing.)
    TASK_APPID_LIST_SIZE,       //
    TASK_APPID_LIST,            //

                                // Constants for referencing TaskIdList attributes. (Enumerator used during internal processing.)
    TASK_TASKID_LIST_SIZE,      //
    TASK_TASKID_LIST,           //

                                // Constants for referencing StepIdList attributes. (Enumerator used during internal processing.)
    TASK_STEPID_LIST_SIZE,      //
    TASK_STEPID_LIST,           //

                                // Constants for referencing NsDataIdList attributes. (Enumerator used during internal processing.)
    TASK_NSDATA_LIST_SIZE,      //
    TASK_NSDATA_LIST,           //

                                // Constants used during internal processing.
    ACTION_SIGNAL_TASK_EVENT,   //
    ACTION_HTTP_HEADER_VALUE,   //
    TASK_ID,                    //

                                // Constants for referencing attributes added anew in SDK version x.x and onward.
    ACTION_FILE_PARAM,                              //
    ACTION_CFG_INFO,                                //

    ACTION_DATASTORE_GAME_ID,                       //
    ACTION_DATASTORE_ACCESS_KEY,                    //

    ACTION_DATASTORE_DOWNLOAD_NEWS_SUBJECT,         //
    ACTION_DATASTORE_DOWNLOAD_NEWS_MESSAGE,         //
    ACTION_DATASTORE_DOWNLOAD_NEWS_JUMP_PARAM,      //
    ACTION_DATASTORE_DOWNLOAD_NEWS_MODE,            //
    ACTION_DATASTORE_DOWNLOAD_NEWS_SERIAL_ID,       //

    ACTION_DATASTORE_UPLOAD_PERIOD,                 //
    ACTION_DATASTORE_UPLOAD_DATA_TYPE,              //
    ACTION_DATASTORE_UPLOAD_DST_PRINCIPAL_ID_NUM,   //
    ACTION_DATASTORE_UPLOAD_DST_KIND,               //
    ACTION_DATASTORE_UPLOAD_DST_PRINCIPAL_ID,       //
};


void InitializeSession(Service::Interface* self);
void SetOptoutFlag(Service::Interface* self);
void GetOptoutFlag(Service::Interface* self);
void GetTaskIdList(Service::Interface* self);
void SendProperty(Service::Interface* self);
void ReceiveProperty(Service::Interface* self);
void GetTaskInfo(Service::Interface* self);

/// Initialize BOSS service(s)
void Init();

/// Shutdown BOSS service(s)
void Shutdown();

} // namespace BOSS
} // namespace Service

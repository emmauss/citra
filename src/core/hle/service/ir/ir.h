// Copyright 2015 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

namespace Service {

class Interface;

namespace IR {

/**
 * IR::GetHandles service function
 *  Outputs:
 *      1 : Result of function, 0 on success, otherwise error code
 *      2 : Translate header, used by the ARM11-kernel
 *      3 : Shared memory handle
 *      4 : Event handle
 */
void GetHandles(Interface* self);

/**
* IR::InitializeIrNopShared service function
*  Inputs:
*      1 : unknown
*      2 : unknown
*      3 : unknown
*      4 : unknown
*      5 : unknown
*      6 : BaudRate (u8)
*      7 : 0
*      8 : Handle
*  Outputs:
*      1 : Result of function, 0 on success, otherwise error code
*/
void InitializeIrNopShared(Interface* self);

/**
* IR::FinalizeIrNop service function
*  Outputs:
*      1 : Result of function, 0 on success, otherwise error code
*/
void FinalizeIrNop(Interface* self);

/**
* IR::GetConnectionStatusEvent service function
*  Outputs:
*      1 : Result of function, 0 on success, otherwise error code
*      2 : Connection Status Event handle
*/
void GetConnectionStatusEvent(Interface* self);

/**
* IR::Disconnect service function
*  Outputs:
*      1 : Result of function, 0 on success, otherwise error code
*/
void Disconnect(Interface* self);

/**
* IR::RequireConnection service function
*  Inputs:
*      1 : unknown (u8)
*  Outputs:
*      1 : Result of function, 0 on success, otherwise error code
*/
void RequireConnection(Interface* self);

/// Initialize IR service
void Init();

/// Shutdown IR service
void Shutdown();

} // namespace IR
} // namespace Service

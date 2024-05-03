/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <uORB/topics/actuator_armed.h>

#include <uavcan/uavcan.hpp>
#include <dronecan/remoteid/BasicID.hpp>

#include <px4_platform_common/module_params.h>

class UavcanRemoteIDController : public ModuleParams
{
public:
	UavcanRemoteIDController(uavcan::INode &node);
	~UavcanRemoteIDController() = default;

	int init();

private:
    typedef uavcan::MethodBinder<UavcanRemoteIDController *, void (UavcanRemoteIDController::*)(const uavcan::TimerEvent &)>
            TimerCbBinder;

	static constexpr unsigned MAX_RATE_HZ = 10;
    uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	void periodic_update(const uavcan::TimerEvent &);

	uavcan::INode &_node;

	//DEFINE_PARAMETERS(
	//	(ParamInt<px4::params::UAVCAN_LGT_ANTCL>) _param_mode_anti_col,
	//	(ParamInt<px4::params::UAVCAN_LGT_STROB>) _param_mode_strobe,
	//	(ParamInt<px4::params::UAVCAN_LGT_NAV>) _param_mode_nav,
	//	(ParamInt<px4::params::UAVCAN_LGT_LAND>) _param_mode_land
	//)
};

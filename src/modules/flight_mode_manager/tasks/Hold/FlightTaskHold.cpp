/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
/**
 * @file FlightTaskHold.cpp
 */

#include "FlightTaskHold.hpp"

using namespace matrix;

bool FlightTaskHold::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	_has_stopped = false;
	bool ret = FlightTask::activate(last_setpoint);

	_velocity_setpoint(0) = 0.0f;
	_velocity_setpoint(1) = 0.0f;
	_velocity_setpoint(2) = 0.0f;

	return ret;
}

bool FlightTaskHold::updateInitialize()
{
	if (!(PX4_ISFINITE(_velocity(0)) && PX4_ISFINITE(_velocity(1)) && PX4_ISFINITE(_velocity(2))
	      && PX4_ISFINITE(_position(0)) && PX4_ISFINITE(_position(1)) && PX4_ISFINITE(_position(2)))) {
		return false;
	}

	_sub_vehicle_attitude_setpoint.update();
	return FlightTask::updateInitialize();
}

bool FlightTaskHold::updateFinalize()
{
	return true;
}

bool FlightTaskHold::update()
{
	const bool has_stopped = sqrt(_velocity(0) * _velocity(0) + _velocity(1) * _velocity(1) + _velocity(2) * _velocity(
					      2)) < 0.1f;

	if (has_stopped && !_has_stopped) {
		_has_stopped = has_stopped;
		_position_setpoint(0) = _position(0);
		_position_setpoint(1) = _position(1);
		_position_setpoint(2) = _position(2);
		_velocity_setpoint(0) = NAN;
		_velocity_setpoint(1) = NAN;
		_velocity_setpoint(2) = NAN;
	}

	if (_weather_vane.enabled() && _has_stopped) {

		_yawspeed_setpoint = _weather_vane.calculate_weathervane_yawrate(
					     matrix::Quatf(_sub_vehicle_attitude_setpoint.get().q_d).dcm_z(),
					     _sub_vehicle_local_position.get().heading);
	}

	return true;
}

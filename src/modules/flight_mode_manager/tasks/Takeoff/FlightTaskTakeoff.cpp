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
 * @file FlightTaskTakeoff.cpp
 */

#include "FlightTaskTakeoff.hpp"

bool FlightTaskTakeoff::doesCommandApply(const vehicle_command_s &command)
{
	if (command.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
		return true;
	}

	return  false;
}

bool FlightTaskTakeoff::applyCommandParameters(const vehicle_command_s &command)
{
	if (PX4_ISFINITE(command.param4)) {
		_yaw_setpoint = command.param4;

	} else {
		_yaw_setpoint = _yaw;
	}

	_position_setpoint = _position;

	if (PX4_ISFINITE(command.param7)) {
		// XXX: For command long without frame the takeoff altitude is currently AMSL.
		if (_sub_vehicle_local_position.get().z_global) {
			_position_setpoint(2) = _position(2) - (command.param7 - _sub_vehicle_local_position.get().ref_alt);

			if (_position_setpoint(2) > _position(2)) {
				PX4_WARN("Takeoff altitude below current altitude");
				return false;
			}

		} else {
			PX4_WARN("Altitude in AMSL not available yet");
			return false;
		}

	} else {
		_position_setpoint(2) = _position(2) - _param_mis_takeoff_alt.get();
	}

	_velocity_setpoint(0) = NAN;
	_velocity_setpoint(1) = NAN;
	_velocity_setpoint(2) = NAN;

	_acceleration_setpoint(0) = NAN;
	_acceleration_setpoint(1) = NAN;
	_acceleration_setpoint(2) = NAN;

	return true;
}

bool FlightTaskTakeoff::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
	return ret;
}

bool FlightTaskTakeoff::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	return ret;
}

bool FlightTaskTakeoff::update()
{
	bool ret = FlightTask::update();

	_constraints.want_takeoff = _checkTakeoff();

	return ret;
}

bool FlightTaskTakeoff::isFinished() const
{
	const bool has_reached_altitude = fabsf(_position_setpoint(2) - _position(2)) < 0.1f;
	const bool has_low_z_vel = fabsf(_velocity(2)) < 0.2f;
	return has_reached_altitude && has_low_z_vel;
}

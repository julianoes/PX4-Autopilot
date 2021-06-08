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

#include "../../../commander/px4_custom_mode.h"

using namespace matrix;

bool FlightTaskHold::doesCommandApply(const vehicle_command_s &command)
{
	if (command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE) {
		uint8_t base_mode = static_cast<uint8_t>(command.param1);
		uint8_t custom_main_mode = static_cast<uint8_t>(command.param2);
		uint8_t custom_sub_mode = static_cast<uint8_t>(command.param3);

		if (base_mode & VEHICLE_MODE_FLAG_CUSTOM_MODE_ENABLED
		    && custom_main_mode == PX4_CUSTOM_MAIN_MODE_AUTO
		    && custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER) {
			return true;
		}

	} else if (command.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {
		if (static_cast<int>(command.param2) == 1) {
			// MAV_DO_REPOSITION_FLAGS_CHANGE_MODE.
			return true;
		}
	}

	return  false;
}

bool FlightTaskHold::applyCommandParameters(const vehicle_command_s &command)
{
	if (!FlightTaskHold::doesCommandApply(command)) {
		return false;
	}

	if (command.command == vehicle_command_s::VEHICLE_CMD_DO_REPOSITION) {
		if (PX4_ISFINITE(command.param1) && command.param1 > 0.0f) {
			_constraints.speed_xy = command.param1;
		}

		if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {

			if (!map_projection_initialized(&_global_local_proj_ref)) {
				return false;
			}

			map_projection_project(&_global_local_proj_ref,
					       command.param5, command.param6,
					       &_position_setpoint(0), &_position_setpoint(1));

			_velocity_setpoint(0) = NAN;
			_velocity_setpoint(1) = NAN;
		}

		if (PX4_ISFINITE(command.param7)) {
			_position_setpoint(2) = -(command.param7 - _sub_vehicle_local_position.get().ref_alt);
			_velocity_setpoint(2) = NAN;
		}

		// We need to do yaw last as it depends on the position setpoint.
		if (PX4_ISFINITE(command.param4)) {
			_yaw_setpoint = math::radians(command.param4);

		} else {
			_set_heading_from_mode();
		}
	}

	return true;
}

void FlightTaskHold::_set_heading_from_mode()
{
	Vector2f v{}; // Vector that points towards desired location

	switch (_param_mpc_yaw_mode.get()) {
	// FALLTHROUGH
	case 0: // Heading points towards where we go.

	// FALLTHROUGH
	case 3: // Same as 0 as we're going straight.
	case 4: // Same as 0 but yaw first. TODO: implement yaw first
		if (PX4_ISFINITE(_position_setpoint(0)) && PX4_ISFINITE(_position_setpoint(1))) {
			v = Vector2f(_position_setpoint) - Vector2f(_position);
		}

		break;

	case 1: // Heading points towards home.
		if (_sub_home_position.get().valid_lpos) {
			v = Vector2f(&_sub_home_position.get().x) - Vector2f(_position);
		}

		break;

	case 2: // Heading point away from home.
		if (_sub_home_position.get().valid_lpos) {
			v = Vector2f(_position) - Vector2f(&_sub_home_position.get().x);
		}

		break;
	}

	if (PX4_ISFINITE(v.length())) {
		if (!_compute_heading_from_2D_vector(_yaw_setpoint, v)) {
			_yaw_setpoint = NAN;
		}

	} else {
		_yaw_setpoint = NAN;
	}
}

bool FlightTaskHold::activate(const vehicle_local_position_setpoint_s &last_setpoint)
{
	_has_stopped = false;
	bool ret = FlightTask::activate(last_setpoint);

	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;
	_position_setpoint(2) = NAN;
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
	const bool has_stopped = sqrt(_velocity(0) * _velocity(0)
				      + _velocity(1) * _velocity(1)
				      + _velocity(2) * _velocity(2)) < 0.1f;


	if (has_stopped && !_has_stopped) {
		_has_stopped = has_stopped;

		if (!PX4_ISFINITE(_position_setpoint(0)) && !PX4_ISFINITE(_position_setpoint(1))) {
			_position_setpoint(0) = _position(0);
			_position_setpoint(1) = _position(1);
		}

		if (!PX4_ISFINITE(_position_setpoint(2))) {
			_position_setpoint(2) = _position(2);
		}

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

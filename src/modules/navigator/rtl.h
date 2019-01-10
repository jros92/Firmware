/***************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.h
 * Helper class for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once

#include <px4_module_params.h>

#include "navigator_mode.h"
#include "mission_block.h"

#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include <vector>
#include <string>
#include <iostream>
//#include <iomanip>
#include <lib/mathlib/mathlib.h>
//#include <fstream>

#define NUM_RISK_VALUES 4	// Number of risk values; for now we have the values 1,2,3,4; i.e. 4 values
#define MAX_INTERPOLATION_STEP_DISTANCE 0.00000005 // in degrees for lat and lon

class Navigator;

class RTL : public MissionBlock, public ModuleParams
{
public:
	enum RTLType {
		RTL_HOME = 0,
		RTL_LAND,
		RTL_MISSION,
	};

	RTL(Navigator *navigator);

	~RTL() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

	void set_return_alt_min(bool min);

	int rtl_type() const;


private:
	/**
	 * Set the RTL item
	 */
	void		set_rtl_item();

	/**
	 * Move to next RTL item
	 */
	void		advance_rtl();

	enum RTLState {
		RTL_STATE_NONE = 0,
		RTL_STATE_CLIMB,
		RTL_STATE_RETURN,
		RTL_STATE_TRANSITION_TO_MC,
		RTL_STATE_DESCEND,
		RTL_STATE_LOITER,
		RTL_STATE_LAND,
		RTL_STATE_LANDED,
	} _rtl_state{RTL_STATE_NONE};

	struct RTLPosition {
		double lat;
		double lon;
		float alt;
		float yaw;
		uint8_t safe_point_index; ///< 0 = home position

		void set(const home_position_s &home_position)
		{
			lat = home_position.lat;
			lon = home_position.lon;
			alt = home_position.alt;
			yaw = home_position.yaw;
			safe_point_index = 0;
		}
	};

	RTLPosition _destination; ///< the RTL position to fly to (typically the home position or a safe point)


	// Polygon struct for Contingency Management
	struct RiskZonePolygon {
		unsigned vertex_count;
		std::vector<double> lat_vertex;
		std::vector<double> lon_vertex;
		uint8_t risk_value; ///< 0 = home position

		// void set(const home_position_s &home_position)
		// {
		// 	lat = home_position.lat;
		// 	lon = home_position.lon;
		// 	alt = home_position.alt;
		// 	yaw = home_position.yaw;
		// 	safe_point_index = 0;
		// }
	};

	std::vector<RiskZonePolygon> riskZones;

	/**
	 * Get the risk zones
	 */
	void		loadRiskZones();

	/**
	 * Check if path from pos0 (current) to pos1 (SLZ) intersects the provided risk zone polygon
	 * Return fraction of distance of intersection relative to total distance between the two points
	 */
	float checkPathAgainstRiskZone(double p0_lat, 
		double p0_lon, 
		double p1_lat, 
		double p1_lon, 
		const RiskZonePolygon &polygon);
	
	/**
	 * Check point inside polygon (2D)
	 */
	bool		insidePolygon(const RiskZonePolygon &polygon, double lat, double lon);

	bool _rtl_alt_min{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RTL_RETURN_ALT>) _param_return_alt,
		(ParamFloat<px4::params::RTL_DESCEND_ALT>) _param_descend_alt,
		(ParamFloat<px4::params::RTL_LAND_DELAY>) _param_land_delay,
		(ParamFloat<px4::params::RTL_MIN_DIST>) _param_rtl_min_dist,
		(ParamInt<px4::params::RTL_TYPE>) _param_rtl_type
	)
};

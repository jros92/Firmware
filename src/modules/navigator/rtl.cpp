/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.cpp
 * Helper class to access RTL
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl.h"
#include "navigator.h"
#include <dataman/dataman.h>
#include <navigator/navigation.h>


static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	// Load all risk zones
	loadRiskZones();
	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Loaded %d RISK ZONES", (int)riskZones.size());
}

void
RTL::on_inactive()
{
	// Reset RTL state.
	_rtl_state = RTL_STATE_NONE;
}

int
RTL::rtl_type() const
{
	return _param_rtl_type.get();
}

void
RTL::on_activation()
{
	// find the RTL destination: go through the safe points & home position


	// Get home and current global position
	const home_position_s &home_position = *_navigator->get_home_position();
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	// Load all safe points ("rally points")
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Found %d SAFE POINTS (not including HOME or LAND)", (int)num_safe_points);

	// Declarations
	double distance_to_safepoint[num_safe_points+1];
	double risk_path_fraction_to_safepoint_for_rz[num_safe_points+1][riskZones.size()];
	double risk_distance_to_safepoint_total_per_riskvalue[num_safe_points+1][NUM_RISK_VALUES];
	double weighted_risk_for_safepoints[num_safe_points+1];

	int closest_index = 0;
	mission_safe_point_s safest_safe_point;

	// Take home position into account as a SLZ
	// Check distance
	// double dlat = home_position.lat - global_position.lat;
	// double dlon = home_position.lon - global_position.lon;
	// double min_dist_squared = dlat * dlat + dlon * dlon;
	float dist_to_home_meters = get_distance_to_next_waypoint(home_position.lat, home_position.lon, global_position.lat, global_position.lon);
	distance_to_safepoint[0] = (double) dist_to_home_meters;
	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Current distance to HOME: %.2f m", (double)dist_to_home_meters);

	// Check risk for home position
	for (unsigned current_rz_seq = 0; current_rz_seq < riskZones.size(); ++current_rz_seq) {
		risk_path_fraction_to_safepoint_for_rz[0][current_rz_seq] = checkPathAgainstRiskZone(global_position.lat, 
			global_position.lon, home_position.lat, home_position.lon, riskZones[current_rz_seq]);

		double risk_overflown_distance_for_rz = risk_path_fraction_to_safepoint_for_rz[0][current_rz_seq] * (double)dist_to_home_meters;

		// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
		// 	"[CM] Analysis for path to Safe Point #%d for Risk Zone %d (value=%d): fraction %.5f, risk distance: %.2f", 
		// 	(int)current_seq, 
		// 	(int)current_rz_seq, 
		// 	(int)riskZones[current_rz_seq].risk_value, 
		// 	(double)risk_path_fraction_to_safepoint_for_rz[current_seq][current_rz_seq], 
		// 	(double)risk_overflown_distance_for_rz);

		risk_distance_to_safepoint_total_per_riskvalue[0][riskZones[current_rz_seq].risk_value-1] += risk_overflown_distance_for_rz;
		
	}

	// Assess weighted risk for home position based on the overflown risk zones weighted with their risk value and weighting factor
	double min_weighted_risk_for_safepoints = 
			risk_distance_to_safepoint_total_per_riskvalue[0][0] * risk_weights[0] + 
			risk_distance_to_safepoint_total_per_riskvalue[0][1] * risk_weights[1] +
			risk_distance_to_safepoint_total_per_riskvalue[0][2] * risk_weights[2] +
			risk_distance_to_safepoint_total_per_riskvalue[0][3] * risk_weights[3];

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
				"[CM] Path to HOME :: Distances [m]: Total: %.2f, R1: %.2f, R2: %.2f, R3: %.2f, R4: %.2f, Weighted: %.2f", 
				(double)dist_to_home_meters,
				risk_distance_to_safepoint_total_per_riskvalue[0][0],
				risk_distance_to_safepoint_total_per_riskvalue[0][1],
				risk_distance_to_safepoint_total_per_riskvalue[0][2],
				risk_distance_to_safepoint_total_per_riskvalue[0][3],
				min_weighted_risk_for_safepoints);


	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Total Weighted Risk to fly HOME :: %.2f", min_weighted_risk_for_safepoints);


	// // Store safe points for later
	// std::vector<mission_safe_point_s> mission_safe_points;

	// find the closest point and risk for all points
	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		mission_safe_point_s mission_safe_point;

		if (dm_read(DM_KEY_SAFE_POINTS, current_seq, &mission_safe_point, sizeof(mission_safe_point_s)) !=
		    sizeof(mission_safe_point_s)) {
			PX4_ERR("dm_read failed");
			continue;
		}

		// // Store safe point for later
		// mission_safe_points.push_back(mission_safe_point);

		// TODO: handle mission_safe_point.frame
		// TODO: take altitude into account for distance measurement

		// dlat = mission_safe_point.lat - global_position.lat;
		// dlon = mission_safe_point.lon - global_position.lon;
		// double dist_squared = dlat * dlat + dlon * dlon;

		const float dist_to_safepoint_meters = get_distance_to_next_waypoint(mission_safe_point.lat, mission_safe_point.lon, global_position.lat, global_position.lon);
		distance_to_safepoint[current_seq] = dist_to_safepoint_meters;
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
			"[CM] Distance to Safe Point #%d (Lat: %f, Lon: %f): %.2f m", 
			(int)current_seq, 
			(double)mission_safe_point.lat, 
			(double)mission_safe_point.lon, 
			(double)dist_to_safepoint_meters);


		// Check risk for every risk zone
		for (unsigned current_rz_seq = 0; current_rz_seq < riskZones.size(); ++current_rz_seq) {
			risk_path_fraction_to_safepoint_for_rz[current_seq][current_rz_seq] = checkPathAgainstRiskZone(global_position.lat, 
				global_position.lon, mission_safe_point.lat, mission_safe_point.lon, riskZones[current_rz_seq]);

			double risk_overflown_distance_for_rz = risk_path_fraction_to_safepoint_for_rz[current_seq][current_rz_seq] * distance_to_safepoint[current_seq];

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
				"[CM] Analysis for path to Safe Point #%d for Risk Zone %d (value=%d): fraction %.5f, risk distance: %.2f", 
				(int)current_seq, 
				(int)current_rz_seq, 
				(int)riskZones[current_rz_seq].risk_value, 
				(double)risk_path_fraction_to_safepoint_for_rz[current_seq][current_rz_seq], 
				(double)risk_overflown_distance_for_rz);

			risk_distance_to_safepoint_total_per_riskvalue[current_seq][riskZones[current_rz_seq].risk_value-1] += risk_overflown_distance_for_rz;
			
		}

		// Assess weighted risk for all safe points AND home position based on the overflown risk zones weighted with their risk value and weighting factor
		weighted_risk_for_safepoints[current_seq] = 
			risk_distance_to_safepoint_total_per_riskvalue[current_seq][0] * risk_weights[0] + 
			risk_distance_to_safepoint_total_per_riskvalue[current_seq][1] * risk_weights[1] +
			risk_distance_to_safepoint_total_per_riskvalue[current_seq][2] * risk_weights[2] +
			risk_distance_to_safepoint_total_per_riskvalue[current_seq][3] * risk_weights[3];


		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
				"[CM] Path to SLZ #%d :: Distances [m]: Total: %.2f, R1: %.2f, R2: %.2f, R3: %.2f, R4: %.2f, Weighted: %.2f", 
				(int)current_seq,
				distance_to_safepoint[current_seq],
				risk_distance_to_safepoint_total_per_riskvalue[current_seq][0],
				risk_distance_to_safepoint_total_per_riskvalue[current_seq][1],
				risk_distance_to_safepoint_total_per_riskvalue[current_seq][2],
				risk_distance_to_safepoint_total_per_riskvalue[current_seq][3],
				weighted_risk_for_safepoints[current_seq]);



		if (weighted_risk_for_safepoints[current_seq] < min_weighted_risk_for_safepoints) {
			closest_index = current_seq;
			min_weighted_risk_for_safepoints = weighted_risk_for_safepoints[current_seq];
			safest_safe_point = mission_safe_point;
		}
	}

	
	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Safest SLZ: ID: %d, distance %.2f m", (int)closest_index, (double)distance_to_safepoint[closest_index]);

	// Set the destination for RTL
	if (closest_index == 0) {
		_destination.set(home_position);

	} else {
		_destination.safe_point_index = closest_index;
		_destination.lat = safest_safe_point.lat;
		_destination.lon = safest_safe_point.lon;
		// 	_destination.lat = mission_safe_points[closest_index-1].lat;
		// 	_destination.lon = mission_safe_points[closest_index-1].lon;


		// TODO: for now we use the same altitude as home
		_destination.alt = home_position.alt;
		_destination.yaw = home_position.yaw;
	}


	// float dist_to_safepoint_meters = get_distance_to_next_waypoint(_destination.lat, _destination.lon, global_position.lat, global_position.lon);
	
	// float path_riskzone_portion = checkPathAgainstRiskZone(global_position.lat, global_position.lon, _destination.lat, _destination.lon, riskZones[0]);
	// float path_riskzone_risk_distance = path_riskzone_portion * dist_to_safepoint_meters;
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] The path to the closest safe point (%d) passes through risk zones %.2f of its total distance, leading to a risk zone distance of %.2f m", (int)closest_index, (double)path_riskzone_portion, (double)path_riskzone_risk_distance);


	/* Some state machine stuff and updating status throughout the system*/

	if (_navigator->get_land_detected()->landed) {
		// For safety reasons don't go into RTL if landed.
		_rtl_state = RTL_STATE_LANDED;

	} else if ((rtl_type() == RTL_LAND) && _navigator->on_mission_landing()) {
		// RTL straight to RETURN state, but mission will takeover for landing.

	} else if ((global_position.alt < home_position.alt + _param_return_alt.get())
		   || _rtl_alt_min) {

		// If lower than return altitude, climb up first.
		// If rtl_alt_min is true then forcing altitude change even if above.
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// Otherwise go straight to return
		_rtl_state = RTL_STATE_RETURN;
	}

	set_rtl_item();
}

void
RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}
}

void
RTL::set_return_alt_min(bool min)
{
	_rtl_alt_min = min;
}

void
RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing.
	// Landing using planned mission landing, fly to DO_LAND_START instead of returning HOME.
	// Do nothing, let navigator takeover with mission landing.
	if (rtl_type() == RTL_LAND) {
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// Otherwise use regular RTL.
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const home_position_s &home = *_navigator->get_home_position();
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// Check if we are pretty close to destination already.
	const float destination_dist = get_distance_to_next_waypoint(_destination.lat, _destination.lon, gpos.lat, gpos.lon);

	// Compute the return altitude.
	float return_alt = math::max(_destination.alt + _param_return_alt.get(), gpos.alt);

	// We are close to home, limit climb to min.
	if (destination_dist < _param_rtl_min_dist.get()) {
		return_alt = _destination.alt + _param_descend_alt.get();
	}

	// Compute the loiter altitude.
	const float loiter_altitude = math::min(_destination.alt + _param_descend_alt.get(), gpos.alt);

	switch (_rtl_state) {
	case RTL_STATE_CLIMB: {

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude = return_alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _navigator->get_local_position()->yaw;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home, %d m above destination)",
						     (int)ceilf(return_alt), (int)ceilf(return_alt - home.alt), (int)ceilf(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_RETURN: {

			// Don't change altitude.
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = return_alt;
			_mission_item.altitude_is_relative = false;

			// Use destination yaw if close to destination.
			// Check if we are pretty close to destination already.
			if (destination_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = _destination.yaw;

			} else {
				// Use current heading to destination.
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _destination.lat, _destination.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above home, %d m above destination)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt), (int)ceilf(_mission_item.altitude - _destination.alt));

			break;
		}

	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_STATE_DESCEND: {
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;

			// Except for vtol which might be still off here and should point towards this location.
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else {
				_mission_item.yaw = _destination.yaw;
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			// Disable previous setpoint to prevent drift.
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above destination)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - _destination.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			const bool autoland = (_param_land_delay.get() > FLT_EPSILON);

			// Don't change altitude.
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.altitude = loiter_altitude;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _destination.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = math::max(_param_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			break;
		}

	case RTL_STATE_LAND: {
			// Land at destination.
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = _destination.lat;
			_mission_item.lon = _destination.lon;
			_mission_item.yaw = _destination.yaw;
			_mission_item.altitude = _destination.alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at destination");
			break;
		}

	case RTL_STATE_LANDED: {
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// Execute command if set. This is required for commands like VTOL transition.
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// Convert mission item to current position setpoint and make it valid.
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;

		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_DESCEND:

		// Only go to land if autoland is enabled.
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else {
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}

// TODO: Take either vector of RiskZonePolygons, or interpolated vectors already (and put interpolation in own function - cleaner?) so interpolation only has to be done once per path
float
RTL::checkPathAgainstRiskZone(double p0_lat, double p0_lon, double p1_lat, double p1_lon, const RiskZonePolygon &polygon)
{
	/* First, interpolate the path between p0 and p1 using the defined maximum interpolation step size */
	std::vector<double> lats, lons;

	int NPOINTS_LAT = (p1_lat - p0_lat) / MAX_INTERPOLATION_STEP_DISTANCE;
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] NPOINTS_LAT=%d", NPOINTS_LAT);
	int NPOINTS_LON = (p1_lon - p0_lon) / MAX_INTERPOLATION_STEP_DISTANCE;
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] NPOINTS_LON=%d", NPOINTS_LON);

	if (NPOINTS_LAT < 0) {
		NPOINTS_LAT *= (-1);
	}
	if (NPOINTS_LON < 0) {
		NPOINTS_LON *= (-1);
	}

	// Take the larger number of steps; making the other vector's step size smaller
	int NPOINTS = math::max(NPOINTS_LON, NPOINTS_LAT);
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] NPOINTS=%d", NPOINTS);

	// Compute actual step sizes for lat and lon, respectively
	// If step size is negative, it will be subtracted in the interpolation loop
	double INTERPOLATION_STEP_LAT = (p1_lat - p0_lat) / (double)NPOINTS;
	double INTERPOLATION_STEP_LON = (p1_lon - p0_lon) / (double)NPOINTS;

	// Interpolate
	double lat, lon;
	lat = p0_lat;
	lon = p0_lon;
	for (int i = 0; i < NPOINTS; i++) {
		// if (lat_increasing) lat += INTERPOLATION_STEP_DISTANCE;
		// else lat -= INTERPOLATION_STEP_DISTANCE;
		lat += INTERPOLATION_STEP_LAT;
		lats.push_back(lat);

		// if (lon_increasing) lon += INTERPOLATION_STEP_DISTANCE;
		// else lon -= INTERPOLATION_STEP_DISTANCE;
		lon += INTERPOLATION_STEP_LON;
		lons.push_back(lon);
	}

	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] INTERP LATS: [0]: %.2f, [1]: %.2f, [2]: %.2f, ... , [end-1]: %.2f, [end]: %.2f", lats[0], lats[1], lats[2], lats[lats.size()-2], lats[lats.size()-1]);
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] INTERP LONS: [0]: %.2f, [1]: %.2f, [2]: %.2f, ... , [end-1]: %.2f, [end]: %.2f", lons[0], lons[1], lons[2], lons[lons.size()-2], lons[lons.size()-1]);

	// bool pointIsInsidePoly[NPOINTS];
	unsigned int count_inside = 0;
	for (int i = 0; i < NPOINTS; i++) {
		if (insidePolygon(polygon, lats[i], lons[i])) ++count_inside;
	}


	return (double)count_inside / (double)NPOINTS;
}


bool
RTL::insidePolygon(const RiskZonePolygon &polygon, double lat, double lon)
{

	/* Adaptation of algorithm originally presented as
	 * PNPOLY - Point Inclusion in Polygon Test
	 * W. Randolph Franklin (WRF)
	 * Only supports non-complex polygons (not self intersecting)
	 */

	bool c = false;

	for (unsigned i = 0, j = polygon.vertex_count - 1; i < polygon.vertex_count; j = i++) {

		if (((double)polygon.lon_vertex[i] >= lon) != ((double)polygon.lon_vertex[j] >= lon) &&
		    (lat <= (double)(polygon.lat_vertex[j] - polygon.lat_vertex[i]) * (lon - (double)polygon.lon_vertex[i]) /
		     (double)(polygon.lon_vertex[j] - polygon.lon_vertex[i]) + (double)polygon.lat_vertex[i])) {
			c = !c;
		}
	}

	return c;
}


void
RTL::loadRiskZones()
{

	// Polygon 0 - Test Poly
	// 50.0416403, 8.6902687 // top left vertex
	// 50.0416436, 8.6906747 // TR vertex
	// 50.0415425, 8.6907306 // BR vertex 
	// 50.0415099, 8.6902687 // BL vertex
	RiskZonePolygon riskZone0;
	riskZone0.vertex_count = 4;
	riskZone0.lat_vertex = {50.0416403, 50.0416436, 50.0415425, 50.0415099};
	riskZone0.lon_vertex = {8.6902687, 8.6906747, 8.6907306, 8.6902687};
	riskZone0.risk_value = 2;
	riskZones.push_back(riskZone0);

	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Risk Zone 1 (risk=%d): %.2f", riskZone1.risk_value, riskZone1.lat_vertex[2]);

	// bool inside1 = insidePolygon(riskZone1, 50.041640, 8.6902689);
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Test Point 1 should be inside, and is %s ", inside1 ? "inside" : "outside");
	// bool inside2 = insidePolygon(riskZone1, 50.0416405, 8.6902686);
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Test Point 2 should not be inside. and is %s ", inside2 ? "inside" : "outside");

	// Polygon 1
	RiskZonePolygon riskZone1;
	riskZone1.vertex_count = 7;
	riskZone1.lat_vertex = {50.0420392689768, 50.042037546394084, 50.04215640445622, 50.04231660398742, 50.042849737286964, 50.04284887601013, 50.0420392689768};
	riskZone1.lon_vertex = {8.690931200981149, 8.691706359386453, 8.691706359386453, 8.692006766796121, 8.692009449005127, 8.690928518772125, 8.690931200981149};
	riskZone1.risk_value = 4;
	riskZones.push_back(riskZone1);

	// Polygon 2
	RiskZonePolygon riskZone2;
	riskZone2.vertex_count = 5;
	riskZone2.lat_vertex = {50.04204099155947, 50.04204099155947, 50.04305041436184, 50.04305041436184, 50.04204099155947};
	riskZone2.lon_vertex = {8.692025542259216, 8.69242787361145, 8.69242787361145, 8.692025542259216, 8.692025542259216};
	riskZone2.risk_value = 3;
	riskZones.push_back(riskZone2);

	// Polygon 3 - DUPLICATE FROM 1
	// RiskZonePolygon riskZone3;
	// riskZone3.vertex_count = 7;
	// riskZone3.lat_vertex = {50.0420392689768, 50.042037546394084, 50.04215640445622, 50.04231660398742, 50.042849737286964, 50.04284887601013, 50.0420392689768};
	// riskZone3.lon_vertex = {8.690931200981149, 8.691706359386453, 8.691706359386453, 8.692006766796121, 8.692009449005127, 8.690928518772125, 8.690931200981149};
	// riskZone3.risk_value = 4;
	// riskZones.push_back(riskZone3);

	// Polygon 4
	RiskZonePolygon riskZone4;
	riskZone4.vertex_count = 29;
	riskZone4.lat_vertex = {50.04187217816588, 50.041882513696834, 50.04184720062356, 50.04179293902132, 50.04176537754906, 50.041724896608024, 50.041658576694644, 50.04145875510307, 50.04123481612416, 50.040979868938095, 50.04097297845482, 50.041241706569856, 50.04138985091314, 50.04152421399193, 50.04162756995048, 50.04177571310302, 50.04186528781067, 50.04196003010803, 50.04199964955876, 50.04191352027633, 50.041875623343095, 50.04189629440128, 50.041934191318234, 50.04194797200785, 50.04197553337527, 50.04196864303489, 50.04194797200785, 50.0419445268358, 50.04187217816588};
	riskZone4.lon_vertex = {8.688844442367554, 8.692728281021129, 8.693089038133621, 8.693428337574005, 8.694070726633072, 8.694308102130888, 8.694519996643077, 8.694879412651062, 8.695067167282104, 8.695142269134521, 8.695362210273743, 8.695335388183592, 8.695260286331177, 8.695147633552551, 8.695061802864075, 8.695072531700134, 8.695120811462402, 8.695230782032013, 8.695010840892792, 8.694847226142883, 8.69464874267579, 8.694283962249768, 8.693978190422069, 8.693313002586375, 8.693001866340648, 8.69269073009492, 8.692502975463878, 8.688844442367554, 8.688844442367554};
	riskZone4.risk_value = 2;
	riskZones.push_back(riskZone4);

	// // Polygon 5
	// RiskZonePolygon riskZone5;
	// riskZone5.vertex_count = ;
	// riskZone5.lat_vertex = {};
	// riskZone5.lon_vertex = {};
	// riskZone5.risk_value = ;
	// riskZones.push_back(riskZone5);

	// // Polygon 6
	// RiskZonePolygon riskZone6;
	// riskZone6.vertex_count = ;
	// riskZone6.lat_vertex = {};
	// riskZone6.lon_vertex = {};
	// riskZone6.risk_value = ;
	// riskZones.push_back(riskZone6);

	// // Polygon 7
	// RiskZonePolygon riskZone7;
	// riskZone7.vertex_count = ;
	// riskZone7.lat_vertex = {};
	// riskZone7.lon_vertex = {};
	// riskZone7.risk_value = ;
	// riskZones.push_back(riskZone7);

	// // Polygon 8
	// RiskZonePolygon riskZone8;
	// riskZone8.vertex_count = ;
	// riskZone8.lat_vertex = {};
	// riskZone8.lon_vertex = {};
	// riskZone8.risk_value = ;
	// riskZones.push_back(riskZone8);

	// // Polygon 9
	// RiskZonePolygon riskZone9;
	// riskZone9.vertex_count = ;
	// riskZone9.lat_vertex = {};
	// riskZone9.lon_vertex = {};
	// riskZone9.risk_value = ;
	// riskZones.push_back(riskZone9);

	// // Polygon 10
	// RiskZonePolygon riskZone10;
	// riskZone10.vertex_count = ;
	// riskZone10.lat_vertex = {};
	// riskZone10.lon_vertex = {};
	// riskZone10.risk_value = ;
	// riskZones.push_back(riskZone10);

	// // Polygon 11
	// RiskZonePolygon riskZone11;
	// riskZone11.vertex_count = ;
	// riskZone11.lat_vertex = {};
	// riskZone11.lon_vertex = {};
	// riskZone11.risk_value = ;
	// riskZones.push_back(riskZone11);

	// // Polygon 12
	// RiskZonePolygon riskZone12;
	// riskZone12.vertex_count = ;
	// riskZone12.lat_vertex = {};
	// riskZone12.lon_vertex = {};
	// riskZone12.risk_value = ;
	// riskZones.push_back(riskZone12);

	// // Polygon 13
	// RiskZonePolygon riskZone13;
	// riskZone13.vertex_count = ;
	// riskZone13.lat_vertex = {};
	// riskZone13.lon_vertex = {};
	// riskZone13.risk_value = ;
	// riskZones.push_back(riskZone13);
}
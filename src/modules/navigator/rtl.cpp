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

	// Save time to get total flight time until touchdown.
	time_cont_initiated = hrt_absolute_time();
	double time_cont_initiated_secs = (double)time_cont_initiated / (double)1000000;

	// Get home and current global position
	const home_position_s &home_position = *_navigator->get_home_position();
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Contingency Landing Procedure initiated at boottime = %.2f s , pos (lat,lon): %.15f, %.15f", time_cont_initiated_secs, global_position.lat, global_position.lon);

	// Load all safe points ("rally points")
	mission_stats_entry_s stats;
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));
	int num_safe_points = 0;

	if (ret == sizeof(mission_stats_entry_s)) {
		num_safe_points = stats.num_items;
	}

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Found %d SAFE POINTS (not including HOME or LAND)", (int)num_safe_points);

	// Initialize variables being computed in the loops
	double distance_to_safepoint[num_safe_points+1] = {0};	// the distance to a safepoint/SLZ in meters
	double risk_path_fraction_to_slz_for_rz[num_safe_points+1][riskZones.size()] = {{0}};
	double risk_dist_to_slz_per_rcat[num_safe_points+1][NUM_RISK_CATEGORIES] = {{0}}; // the total overflown distance to a safepoint/SLZ over risk zones of each category, in meters
	double weighted_risk_for_slzs[num_safe_points+1] = {0}; // Final weighted risk per SLZ

	int closest_index = 0;
	mission_safe_point_s safest_safe_point;

	// Take home position into account as a SLZ
	// Check distance
	// double dlat = home_position.lat - global_position.lat;
	// double dlon = home_position.lon - global_position.lon;
	// double min_dist_squared = dlat * dlat + dlon * dlon;
	distance_to_safepoint[0] = get_distance_to_next_waypoint(home_position.lat, home_position.lon, global_position.lat, global_position.lon);
	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Current distance to HOME: %.2f m", distance_to_safepoint[0]);

	// Check risk for home position
	double min_weighted_risk_for_slzs = 0;
	double min_dist_to_slz = distance_to_safepoint[0];
	for (unsigned current_rz_seq = 0; current_rz_seq < riskZones.size(); ++current_rz_seq) {
		risk_path_fraction_to_slz_for_rz[0][current_rz_seq] = checkPathAgainstRiskZone(global_position.lat, 
			global_position.lon, home_position.lat, home_position.lon, riskZones[current_rz_seq]);

		double risk_overflown_distance_for_rz = risk_path_fraction_to_slz_for_rz[0][current_rz_seq] * distance_to_safepoint[0];

		// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
		// 	"[CM] Analysis for path to Safe Point #%d for Risk Zone %d (value=%d): fraction %.5f, risk distance: %.2f", 
		// 	(int)current_seq, 
		// 	(int)current_rz_seq, 
		// 	(int)riskZones[current_rz_seq].risk_value, 
		// 	(double)risk_path_fraction_to_slz_for_rz[current_seq][current_rz_seq], 
		// 	(double)risk_overflown_distance_for_rz);

		risk_dist_to_slz_per_rcat[0][riskZones[current_rz_seq].risk_value-1] += risk_overflown_distance_for_rz;

		// Assess weighted risk for home position based on the overflown risk zones weighted with their risk value and weighting factor
		min_weighted_risk_for_slzs += risk_dist_to_slz_per_rcat[0][riskZones[current_rz_seq].risk_value-1] * RISK_WEIGHTS[riskZones[current_rz_seq].risk_value-1];
	}

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
				"[CM] Path to HOME :: Distances [m]: Total: %.2f, R1: %.2f, R2: %.2f, R3: %.2f, R4: %.2f, Weighted: %.2f", 
				distance_to_safepoint[0],
				risk_dist_to_slz_per_rcat[0][0],
				risk_dist_to_slz_per_rcat[0][1],
				risk_dist_to_slz_per_rcat[0][2],
				risk_dist_to_slz_per_rcat[0][3],
				min_weighted_risk_for_slzs);

	// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Total Weighted Risk to fly HOME :: %.2f", min_weighted_risk_for_slzs);

	// // Store safe points for later
	// std::vector<mission_safe_point_s> mission_safe_points;

	// find the closest point and risk for all points
	for (int current_seq = 1; current_seq <= num_safe_points; ++current_seq) {
		// DEBUG message
		// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] DEBUG :: distance_to_safepoint[%d]: %.2f, risk_path_fraction_to_slz_for_rz[%d][0]: %f, risk_path_fraction_to_slz_for_rz[][1]: %f, risk_path_fraction_to_slz_for_rz[][2]: %f, risk_path_fraction_to_slz_for_rz[][3]: %f, risk_dist_to_slz_per_rcat[0]: %f, risk_dist_to_slz_per_rcat[1]: %f, risk_dist_to_slz_per_rcat[2]: %f, risk_dist_to_slz_per_rcat[3]: %f, weighted_risk_for_slzs[]: %f", 
		// 	current_seq,
		// 	distance_to_safepoint[current_seq], 
		// 	current_seq,
		// 	risk_path_fraction_to_slz_for_rz[current_seq][0], 
		// 	risk_path_fraction_to_slz_for_rz[current_seq][1], 
		// 	risk_path_fraction_to_slz_for_rz[current_seq][2], 
		// 	risk_path_fraction_to_slz_for_rz[current_seq][3], 
		// 	risk_dist_to_slz_per_rcat[current_seq][0],
		// 	risk_dist_to_slz_per_rcat[current_seq][1],
		// 	risk_dist_to_slz_per_rcat[current_seq][2],
		// 	risk_dist_to_slz_per_rcat[current_seq][3],
		// 	weighted_risk_for_slzs[current_seq]);

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
		// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
		// 	"[CM] Distance to Safe Point #%d (Lat: %f, Lon: %f): %.2f m", 
		// 	(int)current_seq, 
		// 	(double)mission_safe_point.lat, 
		// 	(double)mission_safe_point.lon, 
		// 	(double)dist_to_safepoint_meters);

		// Check risk for every risk zone
		for (unsigned current_rz_seq = 0; current_rz_seq < riskZones.size(); ++current_rz_seq) {
			risk_path_fraction_to_slz_for_rz[current_seq][current_rz_seq] = checkPathAgainstRiskZone(global_position.lat, 
				global_position.lon, mission_safe_point.lat, mission_safe_point.lon, riskZones[current_rz_seq]);

			double risk_overflown_distance_for_rz = risk_path_fraction_to_slz_for_rz[current_seq][current_rz_seq] * distance_to_safepoint[current_seq];

			// mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
			// 	"[CM] Analysis for path to Safe Point #%d for Risk Zone %d (value=%d): fraction %.5f, risk distance: %.2f", 
			// 	(int)current_seq, 
			// 	(int)current_rz_seq, 
			// 	(int)riskZones[current_rz_seq].risk_value, 
			// 	(double)risk_path_fraction_to_slz_for_rz[current_seq][current_rz_seq], 
			// 	(double)risk_overflown_distance_for_rz);

			risk_dist_to_slz_per_rcat[current_seq][riskZones[current_rz_seq].risk_value-1] += risk_overflown_distance_for_rz;
			
		}

		// Assess weighted risk for all safe points AND home position based on the overflown risk zones weighted with their risk value and weighting factor
		weighted_risk_for_slzs[current_seq] = 
			risk_dist_to_slz_per_rcat[current_seq][0] * RISK_WEIGHTS[0] + 
			risk_dist_to_slz_per_rcat[current_seq][1] * RISK_WEIGHTS[1] +
			risk_dist_to_slz_per_rcat[current_seq][2] * RISK_WEIGHTS[2] +
			risk_dist_to_slz_per_rcat[current_seq][3] * RISK_WEIGHTS[3];

		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), 
				"[CM] Path to SLZ #%d :: Distances [m]: Total: %.2f, R1: %.2f, R2: %.2f, R3: %.2f, R4: %.2f, Weighted: %.2f", 
				(int)current_seq,
				distance_to_safepoint[current_seq],
				risk_dist_to_slz_per_rcat[current_seq][0],
				risk_dist_to_slz_per_rcat[current_seq][1],
				risk_dist_to_slz_per_rcat[current_seq][2],
				risk_dist_to_slz_per_rcat[current_seq][3],
				weighted_risk_for_slzs[current_seq]);

		if ((weighted_risk_for_slzs[current_seq] <= min_weighted_risk_for_slzs) && (distance_to_safepoint[current_seq] < min_dist_to_slz)) {
			closest_index = current_seq;
			min_weighted_risk_for_slzs = weighted_risk_for_slzs[current_seq];
			min_dist_to_slz = distance_to_safepoint[current_seq];
			safest_safe_point = mission_safe_point;
		}
	}

	
	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Safest SLZ: ID: %d, distance %.2f m, weighted risk: %.2f", 
		(int)closest_index, (double)distance_to_safepoint[closest_index], weighted_risk_for_slzs[closest_index]);


	// Compute flight termination risk at current position
	// Check risk for every risk zone
	double ft_risk_at_curr_pos = RISK_VALUES_FT[0];
	for (unsigned current_rz_seq = 0; current_rz_seq < riskZones.size(); ++current_rz_seq) {
		if (insidePolygon(riskZones[current_rz_seq], global_position.lat, global_position.lon)) {
			ft_risk_at_curr_pos = RISK_VALUES_FT[riskZones[current_rz_seq].risk_value-1];
		}
	}

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Weighted Risk for Flight Termination right now: %.3f", ft_risk_at_curr_pos);

	bool landing_safest_option = false;
	if (ft_risk_at_curr_pos < weighted_risk_for_slzs[closest_index]) {
		landing_safest_option = true;
		closest_index = -1;
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Landing / Flight Termination at the spot is safest.");

	} else {
		mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] Flying to SLZ is safer than landing/FT. Setting destination.");
	}

	// Set the destination for RTL
	if (closest_index == -1) {
		_destination.set(global_position);
	} else if (closest_index == 0) {
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
	} else if (landing_safest_option) {
		_rtl_state = RTL_STATE_LAND;
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

			hrt_abstime time_cont_total = hrt_absolute_time() - time_cont_initiated;
			double time_cont_total_secs = (double)time_cont_total / (double)1000000;
			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "[CM] LANDED. Total flight duration after contingency: %.2f s", time_cont_total_secs);

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
	int NPOINTS_LON = (p1_lon - p0_lon) / MAX_INTERPOLATION_STEP_DISTANCE;

	if (NPOINTS_LAT < 0) {
		NPOINTS_LAT *= (-1);
	}
	if (NPOINTS_LON < 0) {
		NPOINTS_LON *= (-1);
	}

	// Take the larger number of steps; potentially making the other vector's step size smaller
	int NPOINTS = math::max(NPOINTS_LON, NPOINTS_LAT);

	// Compute actual step sizes for lat and lon, respectively
	// If step size is negative, it will be subtracted in the interpolation loop
	double INTERPOLATION_STEP_LAT = (p1_lat - p0_lat) / (double)NPOINTS;
	double INTERPOLATION_STEP_LON = (p1_lon - p0_lon) / (double)NPOINTS;

	// Interpolate the 2D trajectory
	double lat, lon;
	lat = p0_lat;
	lon = p0_lon;
	for (int i = 0; i < NPOINTS; i++) {
		lat += INTERPOLATION_STEP_LAT;
		lats.push_back(lat);

		lon += INTERPOLATION_STEP_LON;
		lons.push_back(lon);
	}

	/* Check the interpolated trajectory and compute the fraction of it going through the polygon */
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

// Hard-coded risk zones for now
void
RTL::loadRiskZones()
{

	// Polygon 0 - Test Poly
	// 50.0416403, 8.6902687 // top left vertex
	// 50.0416436, 8.6906747 // TR vertex
	// 50.0415425, 8.6907306 // BR vertex 
	// 50.0415099, 8.6902687 // BL vertex

	// RiskZonePolygon riskZone0;
	// riskZone0.vertex_count = 4;
	// riskZone0.lat_vertex = {50.0416403, 50.0416436, 50.0415425, 50.0415099};
	// riskZone0.lon_vertex = {8.6902687, 8.6906747, 8.6907306, 8.6902687};
	// riskZone0.risk_value = 2;
	// riskZones.push_back(riskZone0);

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

	// Polygon 3
	RiskZonePolygon riskZone3;
	riskZone3.vertex_count = 29;
	riskZone3.lat_vertex = {50.04187217816588, 50.041882513696834, 50.04184720062356, 50.04179293902132, 50.04176537754906, 50.041724896608024, 50.041658576694644, 50.04145875510307, 50.04123481612416, 50.040979868938095, 50.04097297845482, 50.041241706569856, 50.04138985091314, 50.04152421399193, 50.04162756995048, 50.04177571310302, 50.04186528781067, 50.04196003010803, 50.04199964955876, 50.04191352027633, 50.041875623343095, 50.04189629440128, 50.041934191318234, 50.04194797200785, 50.04197553337527, 50.04196864303489, 50.04194797200785, 50.0419445268358, 50.04187217816588};
	riskZone3.lon_vertex = {8.688844442367554, 8.692728281021129, 8.693089038133621, 8.693428337574005, 8.694070726633072, 8.694308102130888, 8.694519996643077, 8.694879412651062, 8.695067167282104, 8.695142269134521, 8.695362210273743, 8.695335388183592, 8.695260286331177, 8.695147633552551, 8.695061802864075, 8.695072531700134, 8.695120811462402, 8.695230782032013, 8.695010840892792, 8.694847226142883, 8.69464874267579, 8.694283962249768, 8.693978190422069, 8.693313002586375, 8.693001866340648, 8.69269073009492, 8.692502975463878, 8.688844442367554, 8.688844442367554};
	riskZone3.risk_value = 2;
	riskZones.push_back(riskZone3);

	// Polygon 4
	RiskZonePolygon riskZone4;
	riskZone4.vertex_count = 15;
	riskZone4.lat_vertex = {50.04195141717964, 50.04195658493685, 50.04196605915698, 50.04196778174229, 50.04195141717961, 50.04194194295661, 50.04190060087062, 50.04188337499094, 50.04187906852005, 50.04192041062462, 50.04203582381132, 50.04203840768544, 50.042028933479564, 50.042027210896514, 50.04195141717964};
	riskZone4.lon_vertex = {8.688849806785583, 8.692519068717974, 8.692710846662537, 8.693023324012772, 8.693341165781035, 8.69395807385446, 8.694308102130904, 8.694605827331559, 8.694662153720872, 8.694845885038376, 8.695051074028015, 8.694076091051118, 8.692519068717974, 8.688855171203613, 8.688849806785583};
	riskZone4.risk_value = 3;
	riskZones.push_back(riskZone4);

	// Polygon 5
	RiskZonePolygon riskZone5;
	riskZone5.vertex_count = 12;
	riskZone5.lat_vertex = {50.042003094726866, 50.041961752693545, 50.042116785134894, 50.04237172628318, 50.042843708348975, 50.04398058040487, 50.043984025430795, 50.0428402632412, 50.04247163528284, 50.04234761028657, 50.042182243126476, 50.042003094726866};
	riskZone5.lon_vertex = {8.695024251937866, 8.695238828659058, 8.69537830352783, 8.695458769798277, 8.695453405380249, 8.69543194770813, 8.69519591331482, 8.695222735404968, 8.695217370986938, 8.695206642150879, 8.695142269134521, 8.695024251937866};
	riskZone5.risk_value = 2;
	riskZones.push_back(riskZone5);

	// Polygon 6
	RiskZonePolygon riskZone6;
	riskZone6.vertex_count = 5;
	riskZone6.lat_vertex = {50.04204788188944, 50.042051327054054, 50.04249230608439, 50.04249230608439, 50.04204788188944};
	riskZone6.lon_vertex = {8.694713115692139, 8.694847226142883, 8.694844543933868, 8.694699704647064, 8.694713115692139};
	riskZone6.risk_value = 3;
	riskZones.push_back(riskZone6);

	// Polygon 7
	RiskZonePolygon riskZone7;
	riskZone7.vertex_count = 5;
	riskZone7.lat_vertex = {50.04302802125341, 50.042712794080465, 50.04271451663896, 50.04302802125341, 50.04302802125341};
	riskZone7.lon_vertex = {8.694713115692139, 8.694710433483124, 8.694841861724852, 8.694841861724852, 8.694713115692139};
	riskZone7.risk_value = 3;
	riskZones.push_back(riskZone7);

	// Polygon 8
	RiskZonePolygon riskZone8;
	riskZone8.vertex_count = 5;
	riskZone8.lat_vertex = {50.042712794080465, 50.04271753111614, 50.04250134955727, 50.042497473783385, 50.042712794080465};
	riskZone8.lon_vertex = {8.694844543933868, 8.69347259402275, 8.693471923470497, 8.694844543933868, 8.694844543933868};
	riskZone8.risk_value = 3;
	riskZones.push_back(riskZone8);

	// Polygon 9
	RiskZonePolygon riskZone9;
	riskZone9.vertex_count = 11;
	riskZone9.lat_vertex = {50.04203065606261, 50.042034101228474, 50.04399091548187, 50.044114936232425, 50.04424584667705, 50.04423895666256, 50.043973690352296, 50.04395302018863, 50.04279892192888, 50.042781696371556, 50.04203065606261};
	riskZone9.lon_vertex = {8.695834279060364, 8.697137832641602, 8.697052001953125, 8.696971535682678, 8.696558475494385, 8.696386814117432, 8.696386814117432, 8.69592010974884, 8.695968389511108, 8.695780634880066, 8.695834279060364};
	riskZone9.risk_value = 3;
	riskZones.push_back(riskZone9);

	// Polygon 10
	RiskZonePolygon riskZone10;
	riskZone10.vertex_count = 7;
	riskZone10.lat_vertex = {50.04372220275644, 50.04382555398271, 50.043818663907864, 50.044442211674564, 50.04436642177071, 50.04372220275644, 50.04372220275644};
	riskZone10.lon_vertex = {8.69518518447876, 8.69518518447876, 8.691843152046202, 8.690872192382812, 8.690738081932068, 8.691735863685608, 8.69518518447876};
	riskZone10.risk_value = 2;
	riskZones.push_back(riskZone10);

	// Polygon 11
	RiskZonePolygon riskZone11;
	riskZone11.vertex_count = 5;
	riskZone11.lat_vertex = {50.04306074963911, 50.04306074963911, 50.04300907323047, 50.04302285361155, 50.04306074963911};
	riskZone11.lon_vertex = {8.695212006568907, 8.69244396686554, 8.69243860244751, 8.695217370986938, 8.695212006568907};
	riskZone11.risk_value = 3;
	riskZones.push_back(riskZone11);

	// Polygon 12
	RiskZonePolygon riskZone12;
	riskZone12.vertex_count = 6;
	riskZone12.lat_vertex = {50.0424957512171, 50.04249919634958, 50.04255431843562, 50.04255087330711, 50.0424957512171, 50.0424957512171};
	riskZone12.lon_vertex = {8.6951744556427, 8.69521737098693 ,8.695222735404968, 8.694852590560913, 8.694857954978943, 8.6951744556427};
	riskZone12.risk_value = 3;
	riskZones.push_back(riskZone12);

	// Polygon 13
	RiskZonePolygon riskZone13;
	riskZone13.vertex_count = 5;
	riskZone13.lat_vertex = {50.042051327054054, 50.042051327054054, 50.042981512456436, 50.042981512456436, 50.042051327054054};
	riskZone13.lon_vertex = {8.692460060119627, 8.693114519119261, 8.693114519119261, 8.692460060119627, 8.692460060119627};
	riskZone13.risk_value = 2;
	riskZones.push_back(riskZone13);

	// Polygon 14
	RiskZonePolygon riskZone14;
	riskZone14.vertex_count = 5;
	riskZone14.lat_vertex = {50.04206510771006, 50.04206510771006, 50.04247163528284, 50.04247163528284, 50.04206510771006};
	riskZone14.lon_vertex = {8.693511486053467, 8.69469165802002, 8.69469165802002, 8.693511486053467, 8.693511486053467};
	riskZone14.risk_value = 2;
	riskZones.push_back(riskZone14);

	// Polygon 15
	RiskZonePolygon riskZone15;
	riskZone15.vertex_count = 5;
	riskZone15.lat_vertex = {50.04274724523836, 50.04274724523836, 50.04298840265143, 50.04298840265143, 50.04274724523836};
	riskZone15.lon_vertex = {8.693511486053467, 8.694380521774292, 8.694380521774292,  8.693511486053467,  8.693511486053467};
	riskZone15.risk_value = 2;
	riskZones.push_back(riskZone15);
	
	// Polygon 16
	RiskZonePolygon riskZone16;
	riskZone16.vertex_count = 5;
	riskZone16.lat_vertex = {50.042561208691936, 50.042561208691936, 50.04300907323047, 50.04300907323047, 50.042561208691936};
	riskZone16.lon_vertex = {8.694857954978943, 8.695056438446045, 8.695056438446045, 8.694857954978943, 8.69485795497894};
	riskZone16.risk_value = 2;
	riskZones.push_back(riskZone16);
	
	// Polygon 17
	RiskZonePolygon riskZone17;
	riskZone17.vertex_count = 5;
	riskZone17.lat_vertex = {50.04256465381973, 50.04256465381973, 50.04301251832611, 50.04301251832611, 50.04256465381973};
	riskZone17.lon_vertex = {8.695115447044373, 8.695212006568907, 8.695212006568907, 8.695115447044373, 8.695115447044373};
	riskZone17.risk_value = 2;
	riskZones.push_back(riskZone17);
	
	// Polygon 18
	RiskZonePolygon riskZone18;
	riskZone18.vertex_count = 6;
	riskZone18.lat_vertex = {50.04204788188944, 50.04204788188944, 50.04214090124743, 50.04249230608439, 50.04249230608437, 50.04204788188944};
	riskZone18.lon_vertex = {8.694857954978943, 8.694989383220673, 8.695059120655062, 8.695056438446045, 8.694852590560924, 8.694857954978943};
	riskZone18.risk_value = 2;
	riskZones.push_back(riskZone18);
	
	// Polygon 19
	RiskZonePolygon riskZone19;
	riskZone19.vertex_count = 5;
	riskZone19.lat_vertex = {50.04217535281579, 50.042361390857536, 50.0424888609514, 50.0424888609514, 50.04217535281579};
	riskZone19.lon_vertex = {8.695120811462406, 8.695198595523834, 8.695212006568907, 8.695120811462406, 8.695120811462406};
	riskZone19.risk_value = 2;
	riskZones.push_back(riskZone19);
	
}
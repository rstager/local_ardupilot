#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    // initialise waypoint speed
    set_desired_speed_to_default();

    // when entering guided mode we set the target as the current location.
    set_desired_location(rover.current_loc);

    // guided mode never travels in reverse
    rover.set_reverse(false);

    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            if (!_reached_destination || rover.is_boat()) {
                // check if we've reached the destination
                _distance_to_destination = get_distance(rover.current_loc, _destination);
                if (!_reached_destination && (_distance_to_destination <= rover.g.waypoint_radius || location_passed_point(rover.current_loc, _origin, _destination))) {
                    _reached_destination = true;
                    rover.gcs().send_mission_item_reached_message(0);
                }
                // drive towards destination
                calc_steering_to_waypoint(_reached_destination ? rover.current_loc : _origin, _destination);
                calc_throttle(calc_reduced_speed_for_turn_or_distance(_desired_speed), true);
            } else {
                stop_vehicle();
            }
            break;
        }
        case Guided_ADV:
        {
            if (!_reached_destination) {
                float speed_scaled;
                Location location;

                if( ahrs.get_position(location)) {
                    if (_radius == 0.0) {
                        // check if we've reached the destination
                        _distance_to_destination = get_distance(location, _destination);
                        Vector2f vtodest = location_diff(_destination, location);

                        if (!_reached_destination && (_distance_to_destination <= rover.g.waypoint_radius
                                                      || location_passed_point(location, _origin, _destination)
                                                      || vtodest * _target_final_yaw_vector > 0)) {
                            _reached_destination = true;
                            rover.gcs().send_mission_item_reached_message(_sequence_number);
                            _des_att_time_ms = AP_HAL::millis(); // we are repurposing this as a timer for next guided_target.
                        }

                        // drive towards destination
                        calc_steering_to_waypoint(_origin, _destination, false);
                    } else {

                        Vector2f rvec = location_diff(_center, location);
                        float rangle = atan2(rvec.y, rvec.x);
                        Vector2f tangent(cos(rangle + M_PI_2 * _direction), sin(rangle + M_PI_2 * _direction));
                        float d = rvec.length() - _radius;
                        Vector2f ned(1.0, 0.0);
                        float groundspeed=_desired_speed;
                        Vector2f velocity(cos(rover.ahrs.yaw)*groundspeed,sin(rover.ahrs.yaw)*groundspeed);
                        float V = _desired_speed;

                        float ddot = (velocity % tangent) * _direction;

                        float VCL1 = V * _CL1;
                        float accel_adj = 2 * VCL1 * (ddot + VCL1 * d);
                        float base_accel = V * V / _radius * _turn_gain;
                        float lat_accel = (base_accel + accel_adj) * _direction;

                        calc_steering_from_lateral_acceleration(lat_accel);

                        // detect end of turn
                        float indicator = (velocity % _target_leading_vector) * _direction;

                        if (!_reached_destination && indicator < 0) {
                            _reached_destination = true;
                            rover.gcs().send_mission_item_reached_message(_sequence_number);
                            _des_att_time_ms = AP_HAL::millis(); // we are repurposing this as a timer for next guided_target.
                            lat_accel = 0.0;
                        }

                        calc_steering_from_lateral_acceleration(lat_accel);

                        _nav_lat_accel = lat_accel;
                        _nav_bearing = degrees(atan2(rvec.y, rvec.x)) + 90.0 * _direction;
                        if (_nav_bearing < 0) _nav_bearing += 360;
                        if (_nav_bearing > 360) _nav_bearing += 360;
                        _nav_target_bearing = _nav_bearing;
                        _nav_target_distance = _radius * ned.angle(_target_leading_vector);
                        _nav_xtrack = d * _direction;

                    }

                    if (!_reached_destination) {
                        speed_scaled = calc_reduced_speed_for_turn_or_distance(_desired_speed);
                    } else {
                        speed_scaled = _desired_speed_final;
                    }
                    calc_throttle(speed_scaled, true);
                }
            } else {
                // continue to fly along this path, expecting an update within 2 seconds.
                if (have_attitude_target && (millis() - _des_att_time_ms) > 2000) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "target not updated within  2 secs, stopping");
                    have_attitude_target = false;
                }
                if (have_attitude_target) {
                    calc_steering_to_waypoint(_origin, _extended_destination);
                    calc_throttle(_desired_speed_final, true);
                } else {
                    stop_vehicle();
                    g2.motors.set_steering(0.0f);
                }
            }
            break;
        }

        case Guided_HeadingAndSpeed:
        {
            // stop vehicle if target not updated within 3 seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                calc_steering_to_heading(_desired_yaw_cd, _desired_speed < 0);
                calc_throttle(_desired_speed, true);
            } else {
                stop_vehicle();
                g2.motors.set_steering(0.0f);
            }
            break;
        }

        case Guided_TurnRateAndSpeed:
        {
            // stop vehicle if target not updated within 3 seconds
            if (have_attitude_target && (millis() - _des_att_time_ms) > 3000) {
                gcs().send_text(MAV_SEVERITY_WARNING, "target not received last 3secs, stopping");
                have_attitude_target = false;
            }
            if (have_attitude_target) {
                // run steering and throttle controllers
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds / 100.0f), g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right, _desired_speed < 0);
                g2.motors.set_steering(steering_out * 4500.0f);
                calc_throttle(_desired_speed, true);
            } else {
                stop_vehicle();
                g2.motors.set_steering(0.0f);
            }
            break;
        }

        default:
            gcs().send_text(MAV_SEVERITY_WARNING, "Unknown GUIDED mode");
            break;
    }
}

// return distance (in meters) to destination
float ModeGuided::get_distance_to_destination() const
{
    if ((_guided_mode != Guided_WP && _guided_mode != Guided_ADV) || _reached_destination) {
        return 0.0f;
    }
    return _distance_to_destination;
}

// set desired location
void ModeGuided::set_desired_location(const struct Location& destination)
{
    // call parent
    Mode::set_desired_location(destination);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_WP;
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_destination.lat, _destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
}

// set desired location
void ModeGuided::set_desired_adv(const struct Location& destination,const struct Location& origin,
                                                  const float target_speed, const float target_final_speed,
                                                  const float target_final_yaw_degree,
                                                  const float radius_with_sign, const uint16_t sequence_number,
                                 const float L1_dist, const float turn_gain, const float lead_angle_degree)
{
    if (sequence_number!=0)
        gcs().send_text(MAV_SEVERITY_INFO, "%9.3f seconds without command",(AP_HAL::millis()-_des_att_time_ms)/1000.);

    // can't call parent because it assumes the origin...this will be a maintenance headache, but we need to do the same as parent
    have_attitude_target = true;
    _des_att_time_ms = AP_HAL::millis();

    // record targets
    _origin = origin;
    _destination = destination;
    _extended_destination=destination;

    location_update(_extended_destination,target_final_yaw_degree,target_final_speed*2.0);

    // initialise distance
    _distance_to_destination = get_distance(_origin, _destination);
    _reached_destination = false;

    // set final desired speed
    _desired_speed_final = target_final_speed;

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_ADV;
    _desired_speed=target_speed;
    _radius = fabs(radius_with_sign);
    _sequence_number = sequence_number;
    _target_final_yaw_vector(cos(radians(target_final_yaw_degree)),sin(radians(target_final_yaw_degree)));

    // Turn specific config
    if (_radius != 0) {
        _turn_gain=fmax(turn_gain,0.1);
        _direction= (radius_with_sign>0)?1:-1;

        // calculate center based on target, final_yaw and direction
        float radial_angle_degrees=target_final_yaw_degree+90.0*_direction;
        _center = _destination;
        location_update(_center,radial_angle_degrees,_radius);

        // Determine the yaw that triggers end of turn
        float _target_leading_yaw_radians = radians(target_final_yaw_degree-lead_angle_degree*_direction);
        _target_leading_vector(cos(_target_leading_yaw_radians),sin(_target_leading_yaw_radians));

        // precalculate acceleration term
        _CL1=sqrt(1/pow(L1_dist,2)-pow(1/(2*_radius),2));
    }

    // TODO: add new log_Write
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_destination.lat, _destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
}

// set desired attitude
void ModeGuided::set_desired_heading_and_speed(float yaw_angle_cd, float target_speed)
{
    // call parent
    Mode::set_desired_heading_and_speed(yaw_angle_cd, target_speed);

    // handle guided specific initialisation and logging
    _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

    // record targets
    _desired_yaw_cd = yaw_angle_cd;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_cd, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

void ModeGuided::set_desired_heading_delta_and_speed(float yaw_delta_cd, float target_speed)
{
    // handle initialisation
    if (_guided_mode != ModeGuided::Guided_HeadingAndSpeed) {
        _guided_mode = ModeGuided::Guided_HeadingAndSpeed;
        _desired_yaw_cd = ahrs.yaw_sensor;
    }
    set_desired_heading_and_speed(wrap_180_cd(_desired_yaw_cd + yaw_delta_cd), target_speed);
}

// set desired velocity
void ModeGuided::set_desired_turn_rate_and_speed(float turn_rate_cds, float target_speed)
{
    // handle initialisation
    _guided_mode = ModeGuided::Guided_TurnRateAndSpeed;
    _des_att_time_ms = AP_HAL::millis();
    _reached_destination = false;

    // record targets
    _desired_yaw_rate_cds = turn_rate_cds;
    _desired_speed = target_speed;
    have_attitude_target = true;

    // log new target
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_desired_yaw_rate_cds, 0.0f, 0.0f), Vector3f(_desired_speed, 0.0f, 0.0f));
}

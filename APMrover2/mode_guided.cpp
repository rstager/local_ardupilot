#include "mode.h"
#include "Rover.h"

bool ModeGuided::_enter()
{
    if (rover.failsafe.triggered)
        return false;

    // set desired location to reasonable stopping point
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
        return false;
    }
    _guided_mode = Guided_WP;

    // initialise waypoint speed
    g2.wp_nav.set_desired_speed_to_default();

    sent_notification = false;

    return true;
}

void ModeGuided::update()
{
    switch (_guided_mode) {
        case Guided_WP:
        {
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller
                navigate_to_waypoint();
            } else {
                // send notification
                if (!sent_notification) {
                    sent_notification = true;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }
        case Guided_ADV:
        {
            if (!_reached_destination) {
                if (!_turning) {
                    // Straight line
                    navigate_to_waypoint();
                    _reached_destination=g2.wp_nav.reached_destination();
                } else {
                    // make an arc
                    Vector2f rvec = _center.get_distance_NE(rover.current_loc);
                    float rangle = atan2f(rvec.y, rvec.x);
                    Vector2f tangent(cosf(rangle + M_PI_2 * _direction), sinf(rangle + M_PI_2 * _direction));
                    float d = rvec.length() - _radius;
                    Vector2f velocity(cosf(rover.ahrs.yaw)*_desired_speed,sinf(rover.ahrs.yaw)*_desired_speed);

                    float ddot = (velocity % tangent) * _direction; // radial velocity
                    float turn_rate = ((_desired_speed  / _radius * _turn_gain + 2  * _CL1 * (ddot + _desired_speed * _CL1 * d)) * _direction);

                    // detect end of turn - Stop turning when we are pointing in the exit direction.
                    // This is independent if we are on the correct radius, or passed the destination
                    float indicator = (velocity % _target_leading_vector) * _direction;
                    // Or when we pass the finish line
                    bool indicator2 = rover.current_loc.past_interval_finish_line(_extended_destination, _destination);


                    if ( indicator < 0 || indicator2) {
                        _reached_destination=true;
                        turn_rate = 0.0;
                        attitude_control.get_steering_rate_pid().reset_I();
                        calc_steering_from_turn_rate(turn_rate,_desired_speed_final,false);
                        calc_throttle(_desired_speed_final, true);
                        // todo: set extended destination
                    } else {
                        calc_steering_from_turn_rate(turn_rate,_desired_speed,false);
                        calc_throttle(_desired_speed, true);
                    }
                    _distance_to_destination = rover.current_loc.get_distance(_destination);
                    _nav_xtrack = d * _direction;
                    _nav_bearing_cd = degrees(wrap_2PI(rangle + M_PI_2 * _direction))*100;
                    rover.current_loc.get_bearing_to(_adv_destination);
                    _wp_bearing_cd = rover.current_loc.get_bearing_to(_destination);
                    _desired_lat_accel = turn_rate/_desired_speed;

                }
            }

            // Common handling for both goto and turnto.
            if (_reached_destination)  {
                // send notification
                if (!sent_notification) {
                    rover.gcs().send_mission_item_reached_message(_sequence_number);
                    sent_notification = true;
                    _des_att_time_ms = AP_HAL::millis(); // we are repurposing this as a timer for next guided_target.
                }

                // continue to fly along this path, expecting an update within 1 seconds.
                if (have_attitude_target && (millis() - _des_att_time_ms) > 1000) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "target not updated within  1 secs, stopping");
                    have_attitude_target = false;
                }
                if (have_attitude_target) {
                    // todo: Follow path to extended destination
                } else {
                    stop_vehicle();
                    g2.motors.set_steering(0.0f);
                }

                // keep this up to date
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
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
                calc_steering_to_heading(_desired_yaw_cd);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
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
                float steering_out = attitude_control.get_steering_out_rate(radians(_desired_yaw_rate_cds / 100.0f),
                                                                            g2.motors.limit.steer_left,
                                                                            g2.motors.limit.steer_right,
                                                                            rover.G_Dt);
                set_steering(steering_out * 4500.0f);
                calc_throttle(calc_speed_nudge(_desired_speed, is_negative(_desired_speed)), true);
            } else {
                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
            }
            break;
        }

        case Guided_Loiter:
        {
            rover.mode_loiter.update();
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
    switch (_guided_mode) {
    case Guided_WP:
    case Guided_ADV:
        return _distance_to_destination;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        return 0.0f;
    case Guided_Loiter:
        rover.mode_loiter.get_distance_to_destination();
        break;
    }

    // we should never reach here but just in case, return 0
    return 0.0f;
}

// return true if vehicle has reached or even passed destination
bool ModeGuided::reached_destination() const
{
    switch (_guided_mode) {
    case Guided_WP:
    case Guided_ADV:
        return _reached_destination;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
    case Guided_Loiter:
        return true;
    }

    // we should never reach here but just in case, return true is the safer option
    return true;
}

// get desired location
bool ModeGuided::get_desired_location(Location& destination) const
{
    switch (_guided_mode) {
    case Guided_WP:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_destination();
            return true;
        }
        return false;
        break;

    case Guided_ADV:
        if (_turning) {
            destination=_destination;
            return true;
        } else if (g2.wp_nav.is_destination_valid()) {
                destination = g2.wp_nav.get_destination();
                return true;
        } else {
            return false;
        }
        break;
    case Guided_HeadingAndSpeed:
    case Guided_TurnRateAndSpeed:
        // not supported in these submodes
        return false;
    case Guided_Loiter:
        // get destination from loiter
        return rover.mode_loiter.get_desired_location(destination);
    }

    // should never get here but just in case
    return false;
}

// set desired location
bool ModeGuided::set_desired_location(const struct Location& destination,
                                      float next_leg_bearing_cd)
{
    if (g2.wp_nav.set_desired_location(destination, next_leg_bearing_cd)) {

        // handle guided specific initialisation and logging
        _guided_mode = ModeGuided::Guided_WP;
        sent_notification = false;
        rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(g2.wp_nav.get_desired_speed(), 0.0f, 0.0f));
        return true;
    }
    return false;
}

// set desired location
bool ModeGuided::set_desired_adv(const struct Location& destination,const struct Location& origin,
                                                  const float target_speed, const float target_final_speed,
                                                  const float target_final_yaw_degree,
                                                  const float radius_with_sign, const uint16_t sequence_number,
                                 const float L1_dist, const float turn_gain, const float lead_angle_degree)
{
    if (sequence_number!=0)
        gcs().send_text(MAV_SEVERITY_INFO, "%9.3f seconds without command",(AP_HAL::millis()-_des_att_time_ms)/1000.);

    // handle guided_adv initialisation and logging
    _guided_mode = ModeGuided::Guided_ADV;
    _turning = abs(radius_with_sign)>0.001;

    _desired_speed = target_speed;
    _desired_speed_final = target_final_speed;
    _destination=destination;
    _origin = origin;
    _reached_destination = false;

    // Allow some overshoot if not final //
    have_attitude_target = true;
    _target_final_yaw_degree = target_final_yaw_degree;
    _extended_destination=destination;
    _extended_destination.offset_bearing(target_final_yaw_degree,_desired_speed_final*2.0);

    // sequenced based item reached //
    sent_notification=false;
    _sequence_number = sequence_number;

    if (!_turning) { ;
        // goto is just like GUIDED_WP except how we handle the item reached.
        if (!g2.wp_nav.set_desired_location(destination, target_final_yaw_degree * 100, &origin)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Failed to set target.");
            return false;
        }
        g2.wp_nav.set_desired_speed(_desired_speed);
    } else {
        _des_att_time_ms = AP_HAL::millis();

        // initialise distance
        _distance_to_destination = origin.get_distance(destination);

        _radius = fabsf(radius_with_sign);
        _target_final_yaw_vector(cosf(radians(_target_final_yaw_degree)), sinf(radians(_target_final_yaw_degree)));
        _turn_gain = fmaxf(turn_gain, 0.1);
        _direction = (radius_with_sign > 0) ? 1 : -1;

        // calculate center based on target, final_yaw and direction
        float radial_angle_degrees = target_final_yaw_degree + 90.0 * _direction;
        _center = destination;
        _center.offset_bearing(radial_angle_degrees, _radius);

        // Determine the yaw that triggers end of turn
        float _target_leading_yaw_radians = radians(target_final_yaw_degree - lead_angle_degree * _direction);
        _target_leading_vector(cosf(_target_leading_yaw_radians), sinf(_target_leading_yaw_radians));

        // precalculate acceleration term
        _CL1 = sqrtf(1 / powf(L1_dist, 2) - powf(1 / (2 * _radius), 2));
    }
    // TODO: add new log_Write
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(destination.lat, destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
    return true;
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

bool ModeGuided::start_loiter()
{
    if (rover.mode_loiter.enter()) {
        _guided_mode = Guided_Loiter;
        return true;
    }
    return false;
}

// set guided timeout and movement limits
void ModeGuided::limit_set(uint32_t timeout_ms, float horiz_max)
{
    limit.timeout_ms = timeout_ms;
    limit.horiz_max = horiz_max;
}

// clear/turn off guided limits
void ModeGuided::limit_clear()
{
    limit.timeout_ms = 0;
    limit.horiz_max = 0.0f;
}

// initialise guided start time and location as reference for limit checking
//  only called from AUTO mode's start_guided method
void ModeGuided::limit_init_time_and_location()
{
    limit.start_time_ms = AP_HAL::millis();
    limit.start_loc = rover.current_loc;
}

// returns true if guided mode has breached a limit
bool ModeGuided::limit_breached() const
{
    // check if we have passed the timeout
    if ((limit.timeout_ms > 0) && (millis() - limit.start_time_ms >= limit.timeout_ms)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (is_positive(limit.horiz_max)) {
        return (rover.current_loc.get_distance(limit.start_loc) > limit.horiz_max);
    }

    // if we got this far we must be within limits
    return false;
}


// return short-term target heading in degrees (i.e. target heading back to line between waypoints)
float ModeGuided::wp_bearing() const
{
    if (_guided_mode != Guided_ADV || !_turning){
        return Mode::wp_bearing();
    } else {
        return _wp_bearing_cd* 0.01f;
    }
}

// return short-term target heading in degrees
float ModeGuided::nav_bearing() const
{
    if (_guided_mode != Guided_ADV || !_turning){
        return Mode::nav_bearing();
    } else {
        return _nav_bearing_cd* 0.01f;
    }
}

// return cross track error (i.e. vehicle's distance from the line between waypoints)
float ModeGuided::crosstrack_error() const
{
    if (_guided_mode != Guided_ADV || !_turning){
        return Mode::crosstrack_error();
    } else {
        return _nav_xtrack;
    }
}

// return desired lateral acceleration
float ModeGuided::get_desired_lat_accel() const
{
    if (_guided_mode != Guided_ADV || !_turning){
        return Mode::get_desired_lat_accel();
    } else {
        return _desired_lat_accel;
    }
}

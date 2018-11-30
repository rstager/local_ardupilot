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
                if (_radius == 0.0 ){
                    // check if we've reached the destination
                    _distance_to_destination = get_distance(rover.current_loc, _destination);

                    if (!_reached_destination && (_distance_to_destination <= rover.g.waypoint_radius || location_passed_point(rover.current_loc, _origin, _destination))) {
                        _reached_destination = true;
                        rover.gcs().send_mission_item_reached_message(_sequence_number);
                        _des_att_time_ms = AP_HAL::millis(); // we are repurposing this as a timer for next guided_target.
//                        printf("dist_to_destination %f o-d %f, o-loc %f\n",_distance_to_destination,
//                               location_diff(_origin, _destination).length(),location_diff(_origin, rover.current_loc).length());
                    }

                    // drive towards destination
                    calc_steering_to_waypoint(_origin, _destination, false);
                } else {
                    Location location;

                    if( ahrs.get_position(location)) {
                        Vector2f  rvec=location_diff(_center,location);
                        float rangle=atan2(rvec.y,rvec.x);
                        Vector3f tangent(cos(rangle+M_PI_2*_direction),sin(rangle+M_PI_2*_direction),0);
                        float d=rvec.length()-_radius;
                        Vector3f ned(1.0,0.0,0.0);
                        //Vector3f velocity(cos(rover.ahrs.yaw)*rover.ground_speed,sin(rover.ahrs.yaw)*rover.ground_speed,0);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                        Vector2f gvec= ahrs.groundspeed_vector();
                        Vector3f velocity(gvec.x*_desired_speed/gvec.length(),gvec.y*_desired_speed/gvec.length(),0);
#else
                        //float groundspeed = ahrs.groundspeed();
			            float groundspeed=_desired_speed; //
                        Vector3f velocity(cos(rover.ahrs.yaw)*groundspeed,sin(rover.ahrs.yaw)*groundspeed,0);
#endif

                        //Vector3f headingNED(cos(rover.ahrs.yaw),sin(rover.ahrs.yaw),0);  // just used for end detect


//                        rover.ahrs.get_velocity_NED(velocity);
//                        rover.ahrs.get_velocity_NED(headingNED);
//                        headingNED.normalize();

                        //float V=velocity.length();

                        float V= _desired_speed;
                        float d_time= AP_HAL::millis()/1000.;
                        //float ddot = (d-_d_prev)/(d_time-_d_prev_time);

                        Vector3f ddotv = velocity % tangent;
                        float ddot=ddotv.z*_direction;

//                        printf("ddot %f angle %f ddotv %f %f %f velocity %f %f %f tangent %f %f %f\n",ddot,
//                               degrees(velocity.angle(tangent)),
//                               ddotv.x,ddotv.y,ddotv.z,
//                        velocity.x,velocity.y,velocity.z,tangent.x,tangent.y,tangent.z);

                        float VCL1 = V*_CL1;
                        float accel_adj = 2*VCL1*(ddot+VCL1*d);
                        float base_accel = V*V/_radius*_turn_gain;
                        float lat_accel = (base_accel + accel_adj)*_direction;
                        if (_use_loiter) {
                            rover.nav_controller->update_loiter(_center, _radius,_direction);
                            lat_accel = rover.nav_controller->lateral_acceleration();
                        }

                        calc_steering_from_lateral_acceleration(lat_accel);

                        // detect end of turn
                        Vector3f cross=velocity % _target_final_vector;
                        float indicator=cross.z*_direction;
                        if (cross.x<0) {
//                            printf("Toggle indicator\n");
                            indicator *= -1;
                        }
//                        printf("rvec %f %f rangle %f tangent %f %f gvec %f %f cross %f",rvec.x,rvec.y,rangle,tangent.x,tangent.y,
//                        gvec.x,gvec.y,cross.z);

//                        printf("yaw %f accel_tot %f accel %f accel_adj %f V %f @ %f, target @%f result @%f dir %f d %f ddot %f delta_t %f \n",degrees(rover.ahrs.yaw),
//                               lat_accel,base_accel,accel_adj,V, degrees(velocity.angle(ned)),
//                               degrees(ned.angle(_target_final_vector)),indicator,_direction,
//                               d,ddot, d_time-_d_prev_time);
                        _d_prev_time=d_time;
                        _d_prev = d;
                        if (!_reached_destination && indicator < 0) {
                            _reached_destination = true;
                            rover.gcs().send_mission_item_reached_message(_sequence_number);
                            _des_att_time_ms = AP_HAL::millis(); // we are repurposing this as a timer for next guided_target.
//                            printf("rotate_complete %f o-d %f, o-loc %f\n",_distance_to_destination,
//                                   location_diff(_origin, _destination).length(),location_diff(_origin, location).length());
                            lat_accel=0.0;
                        }

                        calc_steering_from_lateral_acceleration(lat_accel);

                        _nav_lat_accel=lat_accel;
                        _nav_bearing=degrees(atan2(rvec.y,rvec.x))+90.0*_direction;
                        _nav_target_bearing=_nav_bearing;
                        _nav_target_distance=_radius*ned.angle(_target_final_vector);
                        _nav_xtrack=d*_direction;

                    } else {
//                        printf("Position not available\n");
                    }

                }

                if (!_reached_destination ){
                    speed_scaled=calc_reduced_speed_for_turn_or_distance(_desired_speed);
                } else {
                    speed_scaled=_desired_speed_final;
                }
                calc_throttle(speed_scaled, true);
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
                                 const float p1, const float p2, const float p3)
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
    float lead_angle=0.0;
    _turn_gain=fmax(p2,0.5);
    _direction= (radius_with_sign>0)?1:-1;
    _target_final_yaw_radians = radians(target_final_yaw_degree-lead_angle*_direction);
    _use_loiter = p3>0.0;
    if (_radius != 0) {
        float radial_angle_degrees=target_final_yaw_degree+90.0*_direction;
        _center = _destination;
        location_update(_center,radial_angle_degrees,_radius);
        _target_final_vector(cos(_target_final_yaw_radians),sin(_target_final_yaw_radians),0.0);
        float L1=p1;
//        printf("L1 %f radius_with_sign %f\n",L1,_radius);
        _CL1=sqrt(1/pow(L1,2)-pow(1/(2*_radius),2));
//        printf("loiter %d lead %f radius_with_sign %f radial angle %f direction %f center %d %d dest %d %d target yaw %f final %f %f CL1 %f\n",
//               _use_loiter, lead_angle, _radius,
//               radial_angle_degrees, _direction,_center.lat,_center.lng,
//               _destination.lat,_destination.lng, degrees(_target_final_yaw_radians),_target_final_vector.x,_target_final_vector.y,_CL1);
    }

//    gcs().send_text(MAV_SEVERITY_INFO,"goto yaw %f d %f", radius_with_sign/100.0,get_distance(destination,rover.current_loc));
//    gcs().send_text(MAV_SEVERITY_INFO,"goto target %d %d ", destination.lat,destination.lng,origin.lat,origin.lng);
//    gcs().send_text(MAV_SEVERITY_INFO,"goto origin %d %d", origin.lat,origin.lng);
//    gcs().send_text(MAV_SEVERITY_INFO,"goto spd %f %f seq %d", target_speed,target_final_speed,sequence_number);


    // TODO: add new log_Write
    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_destination.lat, _destination.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
}


//// set desired location
//void ModeGuided::set_desired_arc(const struct Location& center,
//                                 const float target_speed, const float target_final_speed,
//                                 const float target_final_yaw,
//                                 const float radius, const uint16_t sequence_number)
//{
//    if (sequence_number!=0)
//        gcs().send_text(MAV_SEVERITY_INFO, "%9.3f seconds without command",(AP_HAL::millis()-_des_att_time_ms)/1000.);
//    // can't call parent because it assumes the origin...this will be a maintenance headache, but we need to do the same as parent
//    have_attitude_target = true;
//    _des_att_time_ms = AP_HAL::millis();
//
//    // record targets
//    _center = center; // This is the center of the arc
//    _desired_yaw_cd = target_final_yaw;
////    if ((target_final_yaw - ahrs.yaw_sensor)%(2*3.1415926)>0)
////        _direction= 1;
////    else
////        _direction= -1;
////    _desired_vec;
////    get_velocity_NED(Vector3f &_desired_vec);
//
//    // initialise distance
//    _distance_to_destination = get_distance(_origin, _destination);
//    _reached_destination = false;
//
//    // set final desired speed
//    _desired_speed_final = target_final_speed;
//
//    // handle guided specific initialisation and logging
//    _guided_mode = ModeGuided::Guided_Arc;
//    _desired_speed=target_speed;
//    _desired_yaw_rate_cds = target_speed/radius; //
//    _sequence_number = sequence_number;
//
////    gcs().send_text(MAV_SEVERITY_INFO,"goto yaw %f d %f", turn_rate_cds/100.0,get_distance(destination,rover.current_loc));
////    gcs().send_text(MAV_SEVERITY_INFO,"goto target %d %d ", destination.lat,destination.lng,origin.lat,origin.lng);
////    gcs().send_text(MAV_SEVERITY_INFO,"goto origin %d %d", origin.lat,origin.lng);
////    gcs().send_text(MAV_SEVERITY_INFO,"goto spd %f %f seq %d", target_speed,target_final_speed,sequence_number);
//
//
//    // TODO: add new log_Write
//    rover.Log_Write_GuidedTarget(_guided_mode, Vector3f(_center.lat, _center.lng, 0), Vector3f(_desired_speed, 0.0f, 0.0f));
//}

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

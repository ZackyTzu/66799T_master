#include "main.h"

Drive::Drive(DriveStyle drive_style, MotorGroup& left_motors, MotorGroup& right_motors, IMU& inertial, 
             float wheel_diameter, float motor_gear_ratio, float gyro_scale, 
             // Rotation& fwd_tracker, float fwd_tracker_diameter,
             float fwd_tracker_dist, 
             // Rotation& sideways_tracker, float sideways_tracker_diameter,
             float sideways_tracker_dist):
    drive_style(drive_style),
    DriveL(left_motors), 
    DriveR(right_motors), 
    Gyro(inertial), 

    wheel_diameter(wheel_diameter), 
    wheel_ratio(motor_gear_ratio), 
    gyro_scale(gyro_scale), 
    drive_in_to_deg_ratio(M_PI*wheel_diameter*wheel_ratio/360.0), // 360 is correct while 36000 isn't for some reason. 

    // Fwd_tracker(fwd_tracker), 
    // ForwardTracker_diameter(fwd_tracker_diameter), 
    ForwardTracker_center_distance(fwd_tracker_dist), 
    // ForwardTracker_in_to_deg_ratio(M_PI*fwd_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

    // Sideways_tracker(sideways_tracker), 
    // SidewaysTracker_diameter(sideways_tracker_diameter), 
    SidewaysTracker_center_distance(sideways_tracker_dist), 
    // SidewaysTracker_in_to_deg_ratio(M_PI*sideways_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

    master(CONTROLLER_MASTER)
{
    odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param left_voltage Voltage out of 127.
 * @param right_voltage Voltage out of 127.
 */
void Drive::drive_with_voltage(int left_voltage, int right_voltage){
    DriveL.move(left_voltage);
    DriveR.move(right_voltage);
}

/**
 * Resets default drive constants.
 * Driving includes drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point().
 * 
 * @param drive_max_voltage Max voltage out of 127.
 * @param drive_kp Proportional constant.
 * @param drive_ki Integral constant.
 * @param drive_kd Derivative constant.
 * @param drive_starti Minimum distance in inches for integral to begin
 * @param drive_min_voltage Min voltage out of 127.
 */
void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float drive_min_voltage){ // ADDED MIN_VOLTAGE!!!!!
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
  this->drive_min_voltage = drive_min_voltage;
}

void Drive::set_drive_motion_chain_constants(float motion_chain_drive_min_voltage, float motion_chain_drive_early_exit_range){
  this->motion_chain_drive_min_voltage = motion_chain_drive_min_voltage;
  this->motion_chain_drive_early_exit_range = motion_chain_drive_early_exit_range;
}

void Drive::set_turn_motion_chain_constants(float motion_chain_turn_min_voltage, float motion_chain_turn_early_exit_range){
  this->motion_chain_turn_min_voltage = motion_chain_turn_min_voltage;
  this->motion_chain_turn_early_exit_range = motion_chain_turn_early_exit_range;
}

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param turn_max_voltage Max voltage out of 127.
 * @param turn_kp Proportional constant.
 * @param turn_ki Integral constant.
 * @param turn_kd Derivative constant.
 * @param turn_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of drive_distance() and drive_to_point().
 * 
 * @param heading_max_voltage Max voltage out of 127.
 * @param heading_kp Proportional constant.
 * @param heading_ki Integral constant.
 * @param heading_kd Derivative constant.
 * @param heading_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only left_swing_to_angle() and right_swing_to_angle() use these constants.
 * 
 * @param swing_max_voltage Max voltage out of 127.
 * @param swing_kp Proportional constant.
 * @param swing_ki Integral constant.
 * @param swing_kd Derivative constant.
 * @param swing_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turn_settle_error Error to be considered settled in degrees.
 * @param turn_settle_time Time to be considered settled in milliseconds.
 * @param turn_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param drive_settle_error Error to be considered settled in inches.
 * @param drive_settle_time Time to be considered settled in milliseconds.
 * @param drive_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swing_settle_error Error to be considered settled in degrees.
 * @param swing_settle_time Time to be considered settled in milliseconds.
 * @param swing_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * 
 * @return Gyro scale-corrected heading in the range [0, 360).
 */
float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360(Gyro.get_heading() *360.0/gyro_scale ) ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Left position in inches.
 */
float Drive::get_left_position_in(){
  return( DriveL.get_position() *drive_in_to_deg_ratio ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Right position in inches.
 */

float Drive::get_right_position_in(){
  return( DriveR.get_position() *drive_in_to_deg_ratio );
}

/**
 * Stops both sides of the drive with the desired mode.
 * 
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(MotorBrake mode){
  MotorBrake old_mode = DriveL.get_brake_mode();

    DriveL.set_brake_mode_all(mode);
    DriveR.set_brake_mode_all(mode);
    // chassis.drive_with_voltage(0, 0);
    DriveL.brake();
    DriveR.brake();

    // DriveL.set_brake_mode_all(old_mode);
    // DriveR.set_brake_mode_all(old_mode);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the 
 * current heading of the robot.
 * 
 * @param angle Desired angle in degrees.
 * @param extra_angle_deg Additional angle to add to the desired angle.
 */

void Drive::turn_to_angle(float angle, bool motion_chaining){
  turn_to_angle(angle, 0, 0, motion_chaining);
}

void Drive::turn_to_angle(float angle, float extra_angle_deg, bool motion_chaining){
  turn_to_angle(angle, extra_angle_deg, 0, motion_chaining);
}

void Drive::turn_to_angle(float angle, float extra_angle_deg, float extra_drive_voltage, bool motion_chaining){
  angle += extra_angle_deg;
  tele_turn_target = angle; // vexdash telemetry
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( !turnPID.is_settled() ){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    tele_turn_error = error; // vexdash telemetry

    if(motion_chaining && fabs(error) < motion_chain_turn_early_exit_range){
      break;
    }

    float output = turnPID.compute(error);
    tele_turn_output = output; // vexdash telemetry
    output = clamp(output, -turn_max_voltage, turn_max_voltage);

    if(motion_chaining){
      output = clamp_min_voltage(output, motion_chain_turn_min_voltage);
    }

    drive_with_voltage(output + extra_drive_voltage, -output + extra_drive_voltage);
    delay(10);
  }
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's currently facing. It uses the average of the left and right
 * motor groups to calculate distance driven.
 * 
 * Passing a heading that differs from the one the robot is currently facing
 * makes it arc into that heading while it drives, which is how you get a
 * curved move instead of a turn-then-straight. Combine it with
 * motion_chaining to exit early (with the drive still powered at
 * motion_chain_drive_min_voltage) and blend into the next movement.
 *
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees. Defaults to the current heading.
 * @param motion_chaining Exit early once within motion_chain_drive_early_exit_range.
 * @param extra_drive_voltage Voltage added to both sides, out of 127.
 */

void Drive::drive_distance(float distance){
  drive_distance(distance, get_absolute_heading(), false, 0);
}

void Drive::drive_distance(float distance, bool motion_chaining){
  drive_distance(distance, get_absolute_heading(), motion_chaining, 0);
}

void Drive::drive_distance(float distance, float heading, bool motion_chaining){
  drive_distance(distance, heading, motion_chaining, 0);
}

void Drive::drive_distance(float distance, float heading, bool motion_chaining, float extra_drive_voltage){
  tele_drive_target = distance; // vexdash telemetry
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    drive_error = distance+start_average_position-average_position;
    if(motion_chaining && fabs(drive_error) < motion_chain_drive_early_exit_range){
      break;
    }

    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    tele_drive_output = drive_output; // vexdash telemetry
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    if(motion_chaining){
      drive_output = clamp_min_voltage(drive_output, motion_chain_drive_min_voltage);
    }

    float left_voltage = drive_output+heading_output + extra_drive_voltage;
    float right_voltage = drive_output-heading_output + extra_drive_voltage;
    drive_with_voltage(left_voltage, right_voltage);

    // TEMP DEBUG -- remove once the slow/no-turn issue is diagnosed.
    printf("heading: %.1f, heading_err: %.1f, drive_out: %.1f, heading_out: %.1f, L: %.1f, R: %.1f\n",
           get_absolute_heading(), heading_error, drive_output, heading_output, left_voltage, right_voltage);

    delay(10);
  }
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turn_to_angle(), is optimized for turning the shorter
 * direction.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::swing_to_angle(float angle, bool move_left, bool motion_chaining){
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    if(motion_chaining && fabs(error) < motion_chain_turn_early_exit_range){
      break;
    }

    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);

    if(motion_chaining){
      output = clamp_min_voltage(output, motion_chain_turn_min_voltage);
    }

    if(move_left){
      DriveL.move(output);
      brake_with_mode_group(DriveR, MotorBrake::hold);
    }
    else{
      DriveR.move(-output);
      brake_with_mode_group(DriveL, MotorBrake::hold);
    }
    
    delay(10);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_ForwardTracker_position(){
  // if(drive_style == DriveStyle::ZERO_TRACKER || 
     // drive_style == DriveStyle::TANK_ONE_SIDEWAYS_ROTATION){
    // return DriveR.get_position() * drive_in_to_deg_ratio;
    return get_right_position_in();
  // }
  
  // return Fwd_tracker.get_position()*ForwardTracker_in_to_deg_ratio;
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  // if(drive_style == DriveStyle::ZERO_TRACKER || 
     // drive_style == DriveStyle::TANK_ONE_FORWARD_ROTATION){
    return 0;
  // }
  
  // return Sideways_tracker.get_position()*SidewaysTracker_in_to_deg_ratio;
}

/**
 * Background task for updating the odometry.
 */

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    delay(5);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so set_heading(45) and the robot will know which way 
 * it's facing.
 * 
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_heading(float orientation_deg){
  Gyro.set_heading(orientation_deg*gyro_scale/360.0);
}

/**
 * MUST BE CALLED TO START THE ODOMETRY TASK. 
 * 
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match.
 * 
 * @param X_position Robot's x in inches.
 * @param Y_position Robot's y in inches.
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);

  // TODO (leaks a task per call -- left as-is for now, fix after testing):
  // pros::Task's destructor does NOT stop or remove the underlying FreeRTOS
  // task. suspend() just parks it, and delete only frees the small wrapper
  // object -- the suspended task keeps its TCB and 32KB stack forever. So
  // every call to set_coordinates() (once per autonomous(), plus any routine
  // that resets coordinates) permanently leaks 32KB.
  // The fix is odom_task->remove() instead of suspend(), then delete.
  if (odom_task != nullptr) { // is this if() even necessary
    odom_task->suspend();    // stop task
    delete odom_task;      // free memory
  }
  odom_task = new Task(position_track_task);

  chassis_lemlib.setPose(X_position, Y_position, to_rad(orientation_deg));
}

/**
 * Gets the robot's x.
 * 
 * @return The robot's x position in inches.
 */

float Drive::get_X_position(){
  return(odom.X_position);
}

/**
 * Gets the robot's y.
 * 
 * @return The robot's y position in inches.
 */

float Drive::get_Y_position(){
  return(odom.Y_position);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 */

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
  PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
  // pros::screen::print(TEXT_MEDIUM, 8, "drive_error: %.2f", drive_error);
  // pros::screen::print(TEXT_MEDIUM, 10, "drivePID.is_settled(): %d", drivePID.is_settled());
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    delay(10);

    // pros::screen::print(TEXT_MEDIUM, 8, "drive_error: %.2f", drive_error);
    // pros::screen::print(TEXT_MEDIUM, 9, "drive_output clamped: %.2f", drive_output);
  }
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low, and the robot will drive 
 * backwards to reach a pose if it's faster. .5 is a reasonable value for the lead. 
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is. 
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param drive_min_voltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::drive_to_pose(float X_position, float Y_position, float angle){
  drive_to_pose(X_position, Y_position, angle, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}


void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  float target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float carrot_X = X_position - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = Y_position - cos(to_rad(angle)) * (lead * target_distance + setback);

    drive_error = hypot(carrot_X-get_X_position(),carrot_Y-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-get_X_position(),carrot_Y-get_Y_position()))-get_absolute_heading());

    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) { 
      heading_error = reduce_negative_180_to_180(angle-get_absolute_heading()); 
      drive_error = target_distance;
    }
    
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    delay(10);
  }
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turn_to_angle() except with a point. The
 * extra_angle_deg parameter turns the robot extra relative to the 
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turn_to_point(36, 42, 180).
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param extra_angle_deg Angle turned past the desired heading in degrees.
 */

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    delay(10);
  }
}


/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_arcade(){
  double throttle = 0;
  double turn = 0;
  chassis.drive_stop(MotorBrake::coast);
  lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  toggle.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  left_roller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  right_roller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  // Start closed so whatever auton ended holding isn't dropped when driver starts.
  bool claw_state = false;
  claw.set_value(claw_state);

  // Lift release damping state, see the L1/L2 block below. lift_holding starts
  // true to match the HOLD brake mode set just above.
  double lift_cmd = 0;
  bool lift_holding = true;
  const double lift_hold_ff = 12;  // tune: least output that stops the arm sagging
  // L2 double-tap auto-retract state, see the L1/L2 block below.
  uint32_t l2_last_press = 0;
  uint32_t lift_auto_start = 0;
  bool lift_auto_down = false;
  const uint32_t lift_double_tap_ms = 400;
  const uint32_t lift_auto_timeout_ms = 3000;
  const double lift_bottom_deg = 5;
  const double lift_slew = 8;      // per 10ms tick, so ~145ms from 115 down

  // R1/R2 roller state. Intake latches on until R2 opens the claw; eject only
  // lasts as long as R2 is held.
  enum RollerPhase { roller_idle, roller_intake, roller_eject };
  RollerPhase roller_phase = roller_idle;

  while(1){
  throttle = master.get_analog(ANALOG_LEFT_Y);
  turn = master.get_analog(ANALOG_RIGHT_X);
  
  
  //deadband
  if(fabs(throttle)<5){
    throttle = 0;
  }
  if(fabs(turn)<5){
    turn = 0;
  }

  DriveL.move(throttle + turn);
  DriveR.move(throttle - turn);

  // Brake mode is deliberately sticky when both sticks are back at center:
  // letting go of turn should hold the heading you just turned to (stays
  // BRAKE), while letting go of throttle should let the chassis roll to a
  // stop (stays COAST) -- whichever axis was last active wins and persists
  // until the other axis takes over.
  if(fabs(turn)>5){
    chassis.DriveL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.DriveR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  else if(fabs(throttle)>5){
    chassis.DriveL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.DriveR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
  
    // Lift: L1 raises until 150 deg, L2 lowers until 0 deg.
    // Magnitude, not signed: the sensor counts the arm's travel away from the
    // reset_position() zero in whichever direction it happens to be wired, and
    // a negative reading here (reversed sensor, or PROS_ERR from an unplugged
    // one) used to fail the arm_deg > 0 test and block L2 entirely.
    double arm_deg = fabs(arm_rotation.get_position() / 100.0);
    // Controller screen can only update every 50ms, so print every 10th loop.
    static int print_count = 0;
    if(++print_count >= 10){
      master.print(0, 0, "arm: %.1f     ", arm_deg);
      print_count = 0;
    }
    // Releasing L1/L2 used to call brake() straight from full speed, and
    // E_MOTOR_BRAKE_HOLD latches its setpoint the instant it is called -- the
    // 4-bar was still travelling past that point, so the hold PID hauled it
    // back, overshot, and rang against the elastics. Instead the command slews
    // down to a small upward feedforward and HOLD is latched only once the arm
    // has actually stopped, so hold grabs a position it is already sitting at.
    // The feedforward is what makes this safe on the way down: cutting to zero
    // there would let gravity keep the arm accelerating, so it would never
    // reach a standstill to latch a hold at and would just fall to the bottom.
    // Double-tapping L2 runs the lift back down to 0 deg on its own. L1 aborts
    // it, as does any further L2 press, and so does the timeout -- that last
    // one is what stops an unplugged rotation sensor (arm_deg then reads off
    // PROS_ERR and never falls under lift_bottom_deg) from driving the lift
    // into its hard stop indefinitely.
    if(master.get_digital_new_press(DIGITAL_L2)){
      if(lift_auto_down){
        lift_auto_down = false;
        l2_last_press = 0;
      }
      else if(l2_last_press != 0 && millis() - l2_last_press <= lift_double_tap_ms){
        lift_auto_down = true;
        lift_auto_start = millis();
        l2_last_press = 0;
      }
      else{
        l2_last_press = millis();
      }
    }
    if(lift_auto_down && (master.get_digital(DIGITAL_L1) ||
                          arm_deg <= lift_bottom_deg ||
                          millis() - lift_auto_start >= lift_auto_timeout_ms)){
      lift_auto_down = false;
    }

    double lift_target = lift_hold_ff;
    if(master.get_digital(DIGITAL_L1) && arm_deg < 270){
    lift_target = 127;
    }
    else if((master.get_digital(DIGITAL_L2) || lift_auto_down) && arm_deg > 0){
      // A closed claw is carrying something, so bring it down gentler.
      lift_target = claw_state ? -65 : -30;
    }

    if(lift_cmd < lift_target){
      lift_cmd = fmin(lift_cmd + lift_slew, lift_target);
    }
    else if(lift_cmd > lift_target){
      lift_cmd = fmax(lift_cmd - lift_slew, lift_target);
    }

    // Red cartridge tops out near 100rpm, so 5 is genuinely stopped. Once the
    // hold is latched it stays latched until L1/L2 is pressed again -- a nudge
    // to the arm must not knock it back out of hold.
    if(lift_target != lift_hold_ff){
      if(lift_holding){
        lift_holding = false;
        lift1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        lift2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      }
    }
    else if(!lift_holding && lift_cmd == lift_hold_ff &&
            fabs(lift1.get_actual_velocity()) < 5){
      lift_holding = true;
      lift1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lift2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    if(lift_holding){
      lift1.brake();
      lift2.brake();
    }
    else{
      lift1.move(lift_cmd);
      lift2.move(lift_cmd);
    }

    // toggle: hold Right to spin at 75% (95/127), brake on release.
    if(master.get_digital(DIGITAL_RIGHT)){
      toggle.move(127);
    }
    else{
      toggle.brake();
    }

    // rollers: R1 closes the claw and starts the intake, which runs at 127
    // while R1 is held and drops to 20 once it is released. The intake stays
    // alive at 20 until the claw opens again, which only R2 does. R2 opens the
    // claw and ejects at 100 for exactly as long as it is held.
    if(master.get_digital(DIGITAL_R1)){
      claw_state = false;
      claw.set_value(claw_state);
      roller_phase = roller_intake;
    }

    if(master.get_digital(DIGITAL_R2)){
      claw_state = true;
      claw.set_value(claw_state);
      roller_phase = roller_eject;
    }
    else if(roller_phase == roller_eject){
      roller_phase = roller_idle;
    }

    // Read the intake speed off the live button state rather than latching it
    // at the moment of release, so a missed transition can't strand the
    // rollers at the held speed.
    switch(roller_phase){
      case roller_intake: {
        int roller_speed = master.get_digital(DIGITAL_R1) ? 127 : 20;
        left_roller.move(roller_speed);
        right_roller.move(roller_speed);
        break;
      }
      case roller_eject:  left_roller.move(-100); right_roller.move(-100); break;
      default:            left_roller.brake();    right_roller.brake();    break;
    }

    // intake: hold Left to intake, hold Down to reverse, brake on release.
    if(master.get_digital(DIGITAL_A)){
      intake.move(127);
    }
    else if(master.get_digital(DIGITAL_B)){
      intake.move(-127);
    }
    else{
      intake.brake();
    }

    // MUST stay here. Without it this loop never blocks, so the FreeRTOS idle
    // task (priority 0, vs this task's 8) never gets scheduled. It also keeps
    // this loop from flooding the smart port bus with unthrottled reads and
    // .move() writes.
    delay(10);
  }
}

// /**
//  * Controls a chassis with left stick throttle and strafe, and right stick turning.
//  * Default deadband is 5.
//  */

// void Drive::control_holonomic(){
//   float throttle = deadband(controller(primary).Axis3.value(), 5);
//   float turn = deadband(controller(primary).Axis1.value(), 5);
//   float strafe = deadband(controller(primary).Axis4.value(), 5);
//   DriveLF.move(fwd, to_volt(throttle+turn+strafe), volt);
//   DriveRF.move(fwd, to_volt(throttle-turn-strafe), volt);
//   DriveLB.move(fwd, to_volt(throttle+turn-strafe), volt);
//   DriveRB.move(fwd, to_volt(throttle-turn+strafe), volt);
// }

// /**
//  * Controls a chassis with left stick left drive and right stick right drive.
//  * Default deadband is 5.
//  */

// void Drive::control_tank(){
//   float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
//   float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
//   DriveL.move(fwd, to_volt(leftthrottle), volt);
//   DriveR.move(fwd, to_volt(rightthrottle), volt);
// }

/**
 * Tracking task to run in the background.
 */

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}
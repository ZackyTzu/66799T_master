#include "main.h"

Drive::Drive(DriveStyle drive_style, MotorGroup& left_motors, MotorGroup& right_motors, IMU& inertial, 
             float wheel_diameter, float motor_gear_ratio, float gyro_scale, 
             Rotation& fwd_tracker, float fwd_tracker_diameter, float fwd_tracker_dist, 
             Rotation& sideways_tracker, float sideways_tracker_diameter, float sideways_tracker_dist):
    drive_style(drive_style),
    DriveL(left_motors), 
    DriveR(right_motors), 
    Gyro(inertial), 

    wheel_diameter(wheel_diameter), 
    wheel_ratio(motor_gear_ratio), 
    gyro_scale(gyro_scale), 
    drive_in_to_deg_ratio(M_PI*wheel_diameter*wheel_ratio/360.0), // 360 is correct while 36000 isn't for some reason. 

    Fwd_tracker(fwd_tracker), 
    ForwardTracker_diameter(fwd_tracker_diameter), 
    ForwardTracker_center_distance(fwd_tracker_dist), 
    ForwardTracker_in_to_deg_ratio(M_PI*fwd_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

    Sideways_tracker(sideways_tracker), 
    SidewaysTracker_diameter(sideways_tracker_diameter), 
    SidewaysTracker_center_distance(sideways_tracker_dist), 
    SidewaysTracker_in_to_deg_ratio(M_PI*sideways_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

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
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param wall_max_voltage Max voltage out of 127.
 * @param wall_kp Proportional constant.
 * @param wall_ki Integral constant.
 * @param wall_kd Derivative constant.
 * @param wall_starti Minimum distance in inches for integral to begin.
 */
void Drive::set_wall_constants(float wall_max_voltage, float wall_kp, float wall_ki, float wall_kd, float wall_starti){
  this->wall_max_voltage = wall_max_voltage;
  this->wall_kp = wall_kp;
  this->wall_ki = wall_ki;
  this->wall_kd = wall_kd;
  this->wall_starti = wall_starti;
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
  if(drive_style == DriveStyle::ZERO_TRACKER || 
     drive_style == DriveStyle::TANK_ONE_SIDEWAYS_ROTATION){
    // return DriveR.get_position() * drive_in_to_deg_ratio;
    return get_right_position_in();
  }
  
  return Fwd_tracker.get_position()*ForwardTracker_in_to_deg_ratio;
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  if(drive_style == DriveStyle::ZERO_TRACKER || 
     drive_style == DriveStyle::TANK_ONE_FORWARD_ROTATION){
    return 0;
  }
  
  return Sideways_tracker.get_position()*SidewaysTracker_in_to_deg_ratio;
}

void Drive::wall_distance(WallSide direction, float distance, float heading, float wall_dis_target, float _drive_min_voltage){
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti, turn_settle_error, turn_settle_time, turn_timeout);
  PID wall_PID(wall_dis_target, wall_kp, wall_ki, wall_kd, wall_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;

  drivePID.settle_time=10; //

  int rev_constant=1;
  if(distance<0) rev_constant =-1;
  // Wait for heading to settle too, not just distance -- otherwise the move
  // can end right as the wall correction is mid-swing, leaving the robot
  // still turned away from the target heading.
  while(!drivePID.is_settled() || !headingPID.is_settled()){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    float wall_distance_error = 0;
    if(direction == WallSide::LEFT){
      wall_distance_error = wall_dis_target - distance_sensorL.get();
    } else {
      wall_distance_error = wall_dis_target - distance_sensorR.get();
    }

    // if(fabs(wall_distance_error)>200) wall_distance_error=0;
    float wall_dist_output = wall_PID.compute(wall_distance_error);

    if(direction == WallSide::RIGHT){
      wall_dist_output = -wall_dist_output;
    }

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    wall_dist_output = clamp(wall_dist_output, -wall_max_voltage, wall_max_voltage);

    clamp_min_voltage(drive_output, _drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output+wall_dist_output*rev_constant), right_voltage_scaling(drive_output, heading_output+wall_dist_output*rev_constant));
    // drive_with_voltage(drive_output+heading_output+wall_dist_output*rev_constant, drive_output-heading_output-wall_dist_output*rev_constant);
    delay(10);
  }
  // drive_settle_time=150;
  // drive_max_voltage=drive_min_voltage;
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



// Cascade never allowed to extend past this (motor degrees, measured from the
// one-time tare_position() in initialize(), see main.cpp). Small placeholder
// -- tune once the real extend limit has been tested.
const int CASCADE_EXTEND_LIMIT_DEG = 3800;

// Preset button cascade targets. Change these values to retarget.
int CASCADE_PRESET_DEG = 545;       // Y -> ArmPosition::POS_1, first cascade move
int CASCADE_PRESET_2_DEG = 595;     // X -> ArmPosition::POS_3, first cascade move
int CASCADE_RIGHT_FINAL_DEG = 220;  // Y -> cascade's 2nd move (down a bit), once the arm settles at POS_1
int CASCADE_LEFT_FINAL_DEG = 0;     // X -> cascade's 2nd move, once the arm reaches POS_3
const int CASCADE_MOVE_VELOCITY = 114; // move_absolute() speed, out of 200 rpm
const int CASCADE_MANUAL_DOWN_SPEED = 67; // UP (d-pad) manual retract, out of 127

// Waiting on cascade/arm move_absolute() inside a preset sequence: max error to
// count as "arrived", and how long to wait before giving up and moving on.
const int CASCADE_SETTLE_ERROR_DEG = 20;
const int PRESET_STEP_TIMEOUT_MS = 3000;
// arm_settled is recomputed by arm_task() every 10ms, so it stays stale (true
// for the *previous* target) briefly after arm_set_position() -- wait this long
// before trusting it.
const int ARM_SETTLE_LATENCY_MS = 30;

// Looser than ARM_SETTLE_ERROR_DEG, used only by the A-button claw-open wait
// (arm_wait_settled_solo). If the arm is stuck/jammed short of CLAW_CLEAR or
// DOWN, waiting on the tight global settle would hold the claw shut until
// PRESET_STEP_TIMEOUT_MS expires. This lets it open as soon as the arm is
// reasonably clear, even if it never fully settles. Still well under the
// smallest gap between claw-sequence targets (DOWN=1 to DOWN_HOLD=15, 14 deg)
// so it doesn't open before the arm has actually moved out of the way.
const float ARM_CLAW_OPEN_SETTLE_ERROR_DEG = 10;

// True while a RIGHT/LEFT preset sequence owns the cascade. Cleared as soon as
// the driver takes manual control with L1/L2, which also tells a running
// sequence task to abort instead of fighting the driver. File scope (not local
// to control_arcade) so those tasks can see it.
static bool cascade_preset_active = false;

// Bumped every time something new takes over the arm: another preset press, or
// DOWN. A running sequence task holds the id it started with and gives up as
// soon as it's been superseded, so a later button press can't be undone by an
// older sequence still working through its steps.
static int preset_sequence_id = 0;

// A sequence keeps going only while it still owns the cascade (driver hasn't
// grabbed L1/L2) and hasn't been superseded by a newer press.
static bool preset_still_owns(int seq_id){
  return cascade_preset_active && seq_id == preset_sequence_id;
}

// Wait for the cascade to reach target_deg. Returns false if the sequence was
// cancelled, so the caller can stop early.
static bool cascade_wait_settled(int seq_id, int target_deg){
  int waited_ms = 0;
  while(fabs(cascade1.get_position() - target_deg) > CASCADE_SETTLE_ERROR_DEG){
    if(!preset_still_owns(seq_id)) return false;
    delay(10);
    waited_ms += 10;
    if(waited_ms > PRESET_STEP_TIMEOUT_MS) break;
  }
  return preset_still_owns(seq_id);
}

// Wait for the arm to reach whatever target was last set with
// arm_set_position(). Same cancellation contract as cascade_wait_settled().
static bool arm_wait_settled(int seq_id){
  delay(ARM_SETTLE_LATENCY_MS);
  int waited_ms = 0;
  while(!arm_settled){
    if(!preset_still_owns(seq_id)) return false;
    delay(10);
    waited_ms += 10;
    if(waited_ms > PRESET_STEP_TIMEOUT_MS) break;
  }
  return preset_still_owns(seq_id);
}

// Same idea as arm_wait_settled(), for arm-only sequences (e.g. the A/claw
// sequence below) that never touch the cascade -- so unlike preset_still_owns(),
// this doesn't require cascade_preset_active to still be true, only that a
// later button press hasn't superseded this sequence's id.
static bool arm_wait_settled_solo(int seq_id){
  delay(ARM_SETTLE_LATENCY_MS);
  int waited_ms = 0;
  while(fabs(tele_arm_error) > ARM_CLAW_OPEN_SETTLE_ERROR_DEG){
    if(seq_id != preset_sequence_id) return false;
    delay(10);
    waited_ms += 10;
    if(waited_ms > PRESET_STEP_TIMEOUT_MS) break;
  }
  return seq_id == preset_sequence_id;
}

// Raised by a claw-open sequence task once the arm has cleared and the claw
// should now open; consumed by control_arcade()'s main loop on its next tick.
//
// Behaviour is unchanged from writing bumper_bt_a directly: the claw only ever
// physically moves on the main loop's claw.set_value(bumper_bt_a) call, so it
// still opens on the same iteration it always did. What changes is ownership --
// bumper_bt_a is now written by one task only. Before, the sequence task wrote
// into control_arcade()'s stack while the driver's A handler wrote the same
// variable, so an A press landing mid-sequence could be silently undone (or
// undo the sequence) depending on which task won.
static volatile bool claw_open_pending = false;

// Rotates the arm to clear_pos, then requests the claw open once it gets there.
// Used by A's claw-open handling below whenever the arm is resting somewhere
// the claw would hit something if it opened immediately.
static void start_claw_open_sequence(ArmPosition clear_pos){
  int claw_seq_id = ++preset_sequence_id;
  arm_set_position(clear_pos);
  pros::Task([claw_seq_id]{
    if(!arm_wait_settled_solo(claw_seq_id)) return;
    claw_open_pending = true;
  });
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_arcade(){
  double throttle = 0;
  double turn = 0;
  bool bt_a=false , last_bt_a=false, bumper_bt_a=false;
  bool bt_Right=false , last_bt_Right=false;
  bool bt_y=false , last_bt_y=false;
  bool bt_b=false , last_bt_b=false;
  bool bt_x=false , last_bt_x=false;
  bool bt_down=false , last_bt_down=false;
  // The two pneumatics are no longer independent per-button toggles -- RIGHT
  // and DOWN both write both of them, and a low arm suppresses op_toggle.
  // op_toggle_state is a REQUEST, not the solenoid's state: it survives while
  // the arm is low and only reaches the solenoid once the arm is above
  // ARM_DOWN_HOLD_DEG. See the RIGHT/DOWN handling in the loop below.
  // toggle_state starts TRUE so the driver's first RIGHT press flips it to
  // false (and matches competition_initialize()'s toggle.set_value(true) in
  // main.cpp, so opcontrol doesn't drop the solenoid the instant it starts).
  // DOWN also puts it back true, so a press of RIGHT after either of those
  // lands on false first, then alternates.
  bool toggle_state=true, op_toggle_state=false;
  bool y_engaged=false; // set by Y's press, consumed by X to pick which behavior it runs
  cascade_preset_active = false;
  // Task in_fxn(intake_status);
  chassis.drive_stop(MotorBrake::coast);

  // HOLD so the cascade stops dead and stays there the instant .move(0) is
  // called, in either direction -- previously only move_absolute() (used by
  // the RIGHT/LEFT presets) actively held; letting go of L1/L2 during manual
  // control just coasted, most noticeably as the cascade sagging back down
  // under gravity after a retract.
  cascade1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  cascade2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  // The Y preset: open the claw, extend the cascade, raise the arm to POS_1,
  // and once the arm settles bring the cascade back down a bit to
  // CASCADE_RIGHT_FINAL_DEG. Two callers below in the loop:
  //   - Y itself.
  //   - RIGHT's first press (the one that turns op_toggle on), which runs the
  //     identical move, y_engaged included -- so X after either button behaves
  //     the same.
  auto start_y_sequence = [&]{
    y_engaged = true;
    cascade_preset_active = true;
    int y_seq_id = ++preset_sequence_id;
    claw.set_value(false);
    bumper_bt_a = false; // keep the A toggle in sync, else the claw.set_value(bumper_bt_a) above re-closes this next tick
    cascade1.move_absolute(CASCADE_PRESET_DEG, CASCADE_MOVE_VELOCITY);
    cascade2.move_absolute(CASCADE_PRESET_DEG, CASCADE_MOVE_VELOCITY);
    arm_set_position(ArmPosition::POS_1);
    // Runs on its own task so waiting for the arm doesn't block the rest
    // of control_arcade() (drive, intake, other buttons).
    pros::Task([y_seq_id]{
      if(!arm_wait_settled(y_seq_id)) return;
      cascade1.move_absolute(CASCADE_RIGHT_FINAL_DEG, CASCADE_MOVE_VELOCITY);
      cascade2.move_absolute(CASCADE_RIGHT_FINAL_DEG, CASCADE_MOVE_VELOCITY);
    });
  };

  // The four-step preset sequence: close the claw, extend the cascade, raise
  // the arm to POS_3, retract the cascade, then settle the arm at POS_2.
  // Called by X below, when y_engaged is set. Pulled out of the X handler so
  // it reads as one named move; captures bumper_bt_a/y_engaged by reference so
  // the claw toggle and the Y-engaged flag stay in sync exactly as the inline
  // version did.
  auto start_y_engaged_sequence = [&]{
    claw.set_value(true);
    bumper_bt_a = true; // keep the A toggle in sync, else claw.set_value(bumper_bt_a) at the A handler would reopen it next tick
    y_engaged = false;  // consumed
    cascade_preset_active = true;
    int x_seq_id = ++preset_sequence_id;
    cascade1.move_absolute(CASCADE_PRESET_2_DEG, CASCADE_MOVE_VELOCITY);
    cascade2.move_absolute(CASCADE_PRESET_2_DEG, CASCADE_MOVE_VELOCITY);
    // Run on its own task so the waits between steps don't block the rest
    // of control_arcade() (drive, intake, other buttons). Each wait bails
    // out if the driver takes the cascade back over with L1/L2, or presses
    // another preset.
    pros::Task([x_seq_id]{
      // 1. cascade is already heading to CASCADE_PRESET_2_DEG; give it 100ms
      //    to start moving, then 2. raise the arm to POS_3.
      pros::delay(100);
      if(!preset_still_owns(x_seq_id)) return;
      arm_set_position(ArmPosition::POS_3);
      if(!arm_wait_settled(x_seq_id)) return;

      // 3. retract the cascade all the way back to CASCADE_LEFT_FINAL_DEG.
      cascade1.move_absolute(CASCADE_LEFT_FINAL_DEG, CASCADE_MOVE_VELOCITY);
      cascade2.move_absolute(CASCADE_LEFT_FINAL_DEG, CASCADE_MOVE_VELOCITY);
      if(!cascade_wait_settled(x_seq_id, CASCADE_LEFT_FINAL_DEG)) return;

      // 4. rotate the arm down to POS_2, cascade holds where it is --
      //    move_absolute() keeps it there, and cascade_preset_active stays
      //    true so the main loop won't zero its voltage.
      arm_set_position(ArmPosition::POS_2);
    });
  };

  // NOTE: deliberately does NOT tare the cascade encoders. It used to, and
  // that was the bug where the cascade "started at 0 while raised": autonomous
  // usually ends with the cascade still up, then opcontrol began by calling
  // tare_position() right here, which declared that raised height to be 0. The
  // L2 retract guard below (get_position() <= 0) then refused to bring it down,
  // and every preset target was off by however high auton had left it.
  // Zeroing now happens once in initialize() (see main.cpp), so 0 always means
  // the height the cascade was at when the program started, and cascade_limit
  // re-zeros it below whenever the real bottom stop is actually hit.

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
  
    // A: toggles the claw open/closed. If the arm is currently settled at
    // POS_2 (~160 degrees, reached via X) and this press is about to OPEN
    // the claw, the arm first rotates up to ARM_CLAW_CLEAR_DEG (180 degrees)
    // to clear itself, and the claw only opens once the arm gets there --
    // opening directly at 160 hits the arm. Closing, and opening from any
    // other arm position, toggles immediately as before.
    // A claw-clear sequence that finished waiting for the arm applies its
    // open here, in the task that owns bumper_bt_a. Consumed before the A
    // press below is evaluated, exactly where the sequence task's own write
    // would have already landed.
    if(claw_open_pending){
      claw_open_pending = false;
      bumper_bt_a = false;
    }

    bt_a = master.get_digital(DIGITAL_A);
    if(!bt_a and last_bt_a){
      bool opening = bumper_bt_a; // bumper_bt_a true = closed; about to flip to false = open (see Y above: claw.set_value(false) opens)
      if(opening && arm_settled && arm_target == ArmPosition::POS_2){
        start_claw_open_sequence(ArmPosition::CLAW_CLEAR);
      }
      else if(opening && arm_settled && arm_target == ArmPosition::DOWN_HOLD){
        // B stopped the arm here instead of going all the way down (see B
        // below) -- finish the trip down to DOWN_HOLD_FINAL, then open.
        start_claw_open_sequence(ArmPosition::DOWN_HOLD_FINAL);
      }
      else{
        bumper_bt_a = !bumper_bt_a;
      }
    }
    last_bt_a = bt_a;
    claw.set_value(bumper_bt_a);

    // Gates op_toggle below.
    bool arm_low = arm_get_position_deg() <= ARM_DOWN_HOLD_DEG;

    // RIGHT (d-pad): flips `toggle`, and always requests op_toggle. Since
    // `toggle` starts true and DOWN puts it back true, a press from there turns
    // `toggle` OFF while op_toggle comes on; pressing it again brings `toggle`
    // back on with op_toggle still on, and so on. Only B clears the op_toggle
    // request.
    //
    // That first press also runs the whole Y preset -- claw open, cascade out,
    // arm to POS_1 (287), cascade back down -- which is what gets the arm above
    // ARM_DOWN_HOLD_DEG (28.5); op_toggle can't extend below that. The request
    // stays latched through the move and reaches the solenoid on its own as the
    // arm clears 28.5, early in the trip up, so no second press is needed.
    // Later presses only flip `toggle`; the arm never affects `toggle` at all.
    bt_Right = master.get_digital(DIGITAL_RIGHT);
    if(!bt_Right and last_bt_Right){
      bool first_press = !op_toggle_state; // "first" = op_toggle wasn't requested yet, i.e. since startup or the last B
      toggle_state = !toggle_state;
      op_toggle_state = true;
      if(first_press){
        start_y_sequence();
      }
    }
    last_bt_Right = bt_Right;


    // DOWN (d-pad): forces `toggle` on. Not a toggle anymore -- pressing it
    // repeatedly just re-asserts it. It no longer touches op_toggle; B owns
    // clearing that now (see below).
    bt_down = master.get_digital(DIGITAL_DOWN);
    if(!bt_down and last_bt_down){
      toggle_state = true;
    }
    last_bt_down = bt_down;

    // The arm gates op_toggle but doesn't clear the request behind it: at or
    // below 28.5 the solenoid is held off no matter what RIGHT asked for, and
    // it comes on by itself as soon as the arm is back above. `toggle` is
    // driven straight from its own state, arm position irrelevant.
    toggle.set_value(toggle_state);
    op_toggle.set_value(op_toggle_state && !arm_low);

    // Cascade limit switch: this one reads 1 when pressed, 0 when not
    // pressed. Every time it's triggered, re-zero both cascade encoders
    // so the physical hard stop is always "0 degrees" -- this corrects any
    // encoder drift picked up over the match, and the existing L2 (retract)
    // check below (cascade1.get_position() <= 0) will now also stop the
    // motors right at the switch instead of relying on drifted encoder math.
    if(cascade_limit.get_value() == 1){
      cascade1.tare_position();
      cascade2.tare_position();
    }

    //intake
    if(master.get_digital(DIGITAL_R1)){
      intake.move(127);
    }
    else if(master.get_digital(DIGITAL_R2)){
      intake.move(-127);
    }
    else{
      intake.move(0);
    }

    // cascade (independent of intake, so R1/R2 no longer block L1/L2)
    if (master.get_digital(DIGITAL_L1)){
      cascade_preset_active = false;
      if(cascade1.get_position() >= CASCADE_EXTEND_LIMIT_DEG || cascade2.get_position() >= CASCADE_EXTEND_LIMIT_DEG){
        cascade1.move(0);
        cascade2.move(0);
      }
      else{
        cascade1.move(100);
        cascade2.move(100);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      cascade_preset_active = false;
      if(cascade1.get_position() <= 0 || cascade2.get_position() <= 0){
        cascade1.move(0);
        cascade2.move(0);
      }
      else{
        cascade1.move(-97);
        cascade2.move(-97);
      }
    }
    // UP (d-pad): manual retract, slower (67) than L2 for fine adjustment.
    // Deliberately NOT bounded by the encoder like L1/L2 -- it keeps driving
    // down past 0 (i.e. after drift has left the encoder reading negative at
    // the real bottom), and the limit switch is the only thing that stops it.
    // The tare above already re-zeroed both encoders on this same tick, so by
    // the time the switch stops the motors here, 0 means the hard stop again.
    // Letting go drops through to the else below, which stops the motors.
    else if(master.get_digital(DIGITAL_UP)){
      cascade_preset_active = false;
      if(cascade_limit.get_value() == 1){
        cascade1.move(0);
        cascade2.move(0);
      }
      else{
        cascade1.move(-CASCADE_MANUAL_DOWN_SPEED);
        cascade2.move(-CASCADE_MANUAL_DOWN_SPEED);
      }
    }
    else{
      if(!cascade_preset_active){
        cascade1.move(0);
        cascade2.move(0);
      }
    }

    // Y: claw open, cascade to POS_1, arm to POS_1, then once the arm
    // settles at POS_1, cascade moves down a bit to CASCADE_RIGHT_FINAL_DEG.
    bt_y = master.get_digital(DIGITAL_Y);
    if(bt_y and !last_bt_y){
      // Y must never bring op_toggle on. Clearing the request here (rather than
      // inside start_y_sequence, which RIGHT shares and whose latch has to
      // survive) is what stops a request left over from RIGHT firing the
      // solenoid the moment this sequence lifts the arm past ARM_DOWN_HOLD_DEG.
      // Y also forces `toggle` ON, same as DOWN does, regardless of where
      // RIGHT/DOWN left it. Written to the solenoids here as well as to the
      // state, so the press lands on this tick instead of the next pass
      // through the set_value() pair above.
      op_toggle_state = false;
      toggle_state = true;
      toggle.set_value(true);
      op_toggle.set_value(false);
      start_y_sequence(); // defined above control_arcade's loop; RIGHT's first press runs it too
    }
    last_bt_y = bt_y;

    // B: arm to DOWN -- cascade untouched. Except if the arm is currently
    // headed to or settled at POS_2 (~160 degrees, reached via X) with the
    // claw closed: then it stops at DOWN_HOLD (~30 degrees) instead of going
    // all the way down, so a held game piece doesn't get slammed into the
    // ground while still gripped. See A above for how DOWN_HOLD gets the arm
    // the rest of the way down (to DOWN_HOLD_FINAL, ~10 degrees) once the
    // claw opens. Deliberately not gated on arm_settled -- X sets arm_target
    // to POS_2 synchronously, so B pressed mid-transit (before the arm
    // physically gets there) should still catch this case instead of
    // falling through to DOWN.
    bt_b = master.get_digital(DIGITAL_B);
    if(bt_b and !last_bt_b){
      // Cancels any preset sequence still stepping through its moves, so it
      // can't send the arm back up after this.
      ++preset_sequence_id;
      y_engaged = false; // arm's leaving the Y-raised state, so X shouldn't run the full sequence next
      // The arm is heading back down, so drop the op_toggle request with it --
      // this is what re-arms RIGHT, whose next press counts as a "first press"
      // and runs the Y sequence again. (Used to live on DOWN.)
      op_toggle_state = false;
      if(arm_target == ArmPosition::POS_2 && bumper_bt_a){
        arm_set_position(ArmPosition::DOWN_HOLD);
      }
      else{
        arm_set_position(ArmPosition::DOWN);
      }
    }
    last_bt_b = bt_b;

    // X: replaces the old dedicated LEFT (d-pad) button. If Y was pressed
    // since the last reset, run the full former-X sequence (extend cascade,
    // raise arm to POS_3, retract cascade, settle arm at POS_2). Otherwise
    // just do the former-LEFT move: arm to POS_2 only, cascade untouched.
    bt_x = master.get_digital(DIGITAL_X);
    if(bt_x and !last_bt_x){
      if(y_engaged){
        start_y_engaged_sequence(); // defined above control_arcade's loop
      }
      else{
        // Cancels any preset sequence still stepping through its moves, so it
        // can't send the arm back up after this.
        ++preset_sequence_id;
        arm_set_position(ArmPosition::POS_2);
      }
    }
    last_bt_x = bt_x;

    // MUST stay here. Without it this loop never blocks, so the FreeRTOS idle
    // task (priority 0, vs this task's 8) never gets scheduled -- and idle is
    // what reclaims a finished task's TCB and 32KB stack. Every Y/X/A preset
    // press below spawns a pros::Task, so without this delay each press leaks
    // 32KB permanently and the brain eventually runs out of heap and freezes
    // mid-match until it's restarted. It also keeps this loop from flooding
    // the smart port bus with unthrottled reads and .move() writes.
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
/****************************************************************************
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
 * @file main.c
 *
 * Example implementation of a fixed wing attitude controller. This file is a complete
 * fixed wing controller for manual attitude control or auto waypoint control.
 * There is no need to touch any other system components to extend / modify the
 * complete control architecture.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch> change
 */

//#include "params.h"
#include "controllers.h"
//#include <math.h>
#include <cmath>
// #include <algorithm>
// #include <iostream>
//#include <lib/matrix/matrix/math.hpp>
// #include <bitset>


#include <poll.h>

#include <stdlib.h>
#include <mathlib/mathlib.h>
//#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/airspeed_validated.h>

// #include <iomanip>

/*
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
*/
//#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

using namespace time_literals;


Controllers::Controllers() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_actuators_0_pub(ORB_ID(actuator_controls_0)),
	//_attitude_sp_pub(ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	// if (vtol) {
	// 	int32_t vt_type = -1;

	// 	if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
	// 		_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
	// 	}
	// }

	/* fetch initial parameter values */
	//parameters_update();

//	parameters_init(&ph);
//	parameters_update(&ph, &p);

	init_ECL_variables();
	initialise_NSADLC_HPF();

	// limit to 50 Hz
	_local_pos_sub.set_interval_ms(20);

	airspeed_sub.set_interval_ms(20);

	// set initial maximum body rate setpoints
	// _roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
	// _pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
	// _pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
	// _yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
}

Controllers::~Controllers()
{
	perf_free(_loop_perf);
}

bool
Controllers::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle local position callback registration failed!");
		return false;
	}

	if (!airspeed_sub.registerCallback()) {
		PX4_ERR("airspeed callback registration failed!");
		return false;
	}

	return true;
}


/* Prototypes */

/**
 * Initialize all parameter handles and values
 *
 */
extern "C" int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
extern "C" int parameters_update(const struct param_handles *h, struct params *p);

/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
// extern "C" __EXPORT int ex_fixedwing_actual_custom_control_mod_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
//static void usage(const char *reason);


/**
 * Control roll and pitch angle.
 *
 * This very simple roll and pitch controller takes the current roll angle
 * of the system and compares it to a reference. Pitch is controlled to zero and yaw remains
 * uncontrolled (tutorial code, not intended for flight).
 *
 * @param att_sp The current attitude setpoint - the values the system would like to reach.
 * @param att The current attitude. The controller should make the attitude match the setpoint
 * @param rates_sp The angular rate setpoint. This is the output of the controller.
 */
void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators);

/**
 * Control heading.
 *
 * This very simple heading to roll angle controller outputs the desired roll angle based on
 * the current position of the system, the desired position (the setpoint) and the current
 * heading.
 *
 * @param pos The current position of the system
 * @param sp The current position setpoint
 * @param att The current attitude
 * @param att_sp The attitude setpoint. This is the output of the controller
 */
void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp);

/**
 *Matrix Multiplication
 *
 * Dampens the pitch rate(q) of the aircraft.
 *
 * @param rn Row of matrix n
 * @param cn column of matix n
 * @param matn matrix n
 */
float matrix_mult(int r1, int c1, int r2, int c2, float mat1[10][10], float mat2[10][10]);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
//static bool thread_running = false;		/**< Daemon status flag */
//static int deamon_task;				/**< Handle of deamon task / thread */
static struct params p;
static struct param_handles ph;

void control_attitude(const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_attitude_s *att,
		      struct vehicle_rates_setpoint_s *rates_sp,
		      struct actuator_controls_s *actuators)
{

	/*
	 * The PX4 architecture provides a mixer outside of the controller.
	 * The mixer is fed with a default vector of actuator controls, representing
	 * moments applied to the vehicle frame. This vector
	 * is structured as:
	 *
	 * Control Group 0 (attitude):
	 *
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 *    4  -  flaps  (-1..+1)
	 *    ...
	 *
	 * Control Group 1 (payloads / special):
	 *
	 *    ...
	 */

	/*
	 * Calculate roll error and apply P gain
	 */

	matrix::Eulerf att_euler = matrix::Quatf(att->q);
	matrix::Eulerf att_sp_euler = matrix::Quatf(att_sp->q_d);

	float roll_err = att_euler.phi() - att_sp_euler.phi();
	actuators->control[0] = roll_err * p.roll_p;

	/*
	 * Calculate pitch error and apply P gain
	 */
	float pitch_err = att_euler.theta() - att_sp_euler.theta();
	actuators->control[1] = pitch_err * p.pitch_p;
}

void control_heading(const struct vehicle_global_position_s *pos, const struct position_setpoint_s *sp,
		     const struct vehicle_attitude_s *att, struct vehicle_attitude_setpoint_s *att_sp)
{

	/*
	 * Calculate heading error of current position to desired position
	 */

	float bearing = get_bearing_to_next_waypoint(pos->lat, pos->lon, sp->lat, sp->lon);

	matrix::Eulerf att_euler = matrix::Quatf(att->q);

	/* calculate heading error */
	float yaw_err = att_euler.psi() - bearing;
	/* apply control gain */
	float roll_body = yaw_err * p.hdng_p;

	/* limit output, this commonly is a tuning parameter, too */
	if (roll_body < -0.6f) {
		roll_body = -0.6f;

	} else if (att_sp->roll_body > 0.6f) {
		roll_body = 0.6f;
	}

	matrix::Eulerf att_spe(roll_body, 0, bearing);
	matrix::Quatf(att_spe).copyTo(att_sp->q_d);
}


void matrix_mult(float mat1[][10], float mat2[][10],float mat_mult[][10], int r1, int c1, int r2, int c2)
{
	if(c1!=r2){ // incorrect dimentions
		PX4_INFO("Incorrect dimenstion size");
		thread_should_exit=true;
	}else{
		// Initializing elements of matrix mult to 0.
   		for (int i = 0; i < r1; ++i) {
      			for (int j = 0; j < c2; ++j) {
         			mat_mult[i][j] = 0;
      			}
   		}

		// Multiplying first and second matrices and storing it in result
   		for (int i = 0; i < r1; ++i) {
      			for (int j = 0; j < c2; ++j) {
         			for (int k = 0; k < c1; ++k) {
            				mat_mult[i][j] += mat1[i][k] * mat2[k][j];
         			}
      			}
   		}
	}

}


// Two funtions below from: https://www.geeksforgeeks.org/program-dot-product-cross-product-two-vector/
// Function that return
// dot product of two vector array.
float dotProduct(float vect_A[], float vect_B[])
{

    float product = 0;

    // Loop for calculate dot product
    for (int i = 0; i < 3; i++){
        product = product + vect_A[i] * vect_B[i];
    }
    return product;
}

// Function to find
// cross product of two vector array.
void crossProduct(float vect_A[], float vect_B[], float cross_P[])

{

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

void Controllers::init_ECL_variables(){
	_hdot_max=2;//altitude controller saturation block max limit
	_hdot_min=-2;//altitude controller saturation block min limit
	// _hdot_max=3.5;//altitude controller saturation block max limit
	// _hdot_min=-3.5;//altitude controller saturation block min limit
	_phi_max=(float)(30.0*M_PI/180.0);//heading controller roll angle saturation block max limit
	_phi_min=(float)(-30.0*M_PI/180.0);//heading controller roll angle saturation block max limit
	_integrator_I[0][0]=0;_integrator_I[1][0]=0;//Airspeed and Climb Rate integrator
	_integrator_RA=0;//Roll Angle Controller intergrator
	_x_guide=0;//intial value must be 0
	_loc_guide=1;//initial value must be 1(location of first destination)
	// K_q=-0.034739;//pitch rate damper gain
	// float temp[2][6]={{-0.0106,24.6502,-0.3462,-38.3493,0.0285,-12.5189},{34.2713,13.9895,-0.0457,-13.7433,41.4445,6.3751}};
	// float temp[2][6]={{0.1053,-0.4295,-0.2568,-9.0674,0.1108,-0.3560},{15.2188,15.7673,-0.0806,-18.3016,16.9485,0.9283}};
	// float temp[2][6]={{0.0319,0.0458,-0.0918,-2.9371,0.0323,-0.1105},{15.2361,15.7865,-0.1288,-19.7905,16.9658,0.8698}};//best so far
	// std::copy(&temp[0][0],&temp[0][0]+2*6,&K_ASCR[0][0]);//airspeed and climbrate controller gain
	// K_h=1.75;//altitude controller gain
	// K_P_RAC=0.03679;//roll angle controller proportional control gain
	// K_I_RAC=0.0147;//roll angle controller intergrator control gain
	K_psi=1.25;//heading controller gain
	K_y=0.017;//guidance 2 controller gain
	vbar_trim=18;//trim airspeed
	alpha_trim=0.0649;//trim angle of attack(and pitch)
	T_Trim=26.5513;
	dE_Trim=-0.0558;
	Ki_AS=3.7136;//airspeed controller gain
	Kp_AS=8.9168;//airspeed controller gain
	Ki_f=-0.1131;//NSADLC gain
	Nc=0.0104;//NSADLC gain
	Ki_e=0.0623;//NSADLC gain
	Kc=0.0069;//NSADLC gain
	Kq=-0.0762;//NSADLC gain
	Km=0.1213;//NSADLC gain
	Ki_CR=0.9;//Climb Rate gain
	Kp_CR=2.70;//Climb Rate gain
	Kp_alt=0.80;//altitude controller gain
	Ki_LSA=-0.2717;//LSA controller gain
	K_r=-0.3176;//LSA controller gain
	K_B=-0.0899;//LSA controller gain
	Ki_RR=-0.7783;//Roll Rate gain
	Kp_RR=-0.06444;//Roll Rate gain
	Kp_RA=1.5;//Roll Angle gain
	Kp_G=0.0170;//Guidance 1 controller gain
	Kd_G=0.0650;//Guidance 1 controller gain
	_intergrator_yaw=0;
	Ki_yaw=-2.25;//Yaw/sideslip(beta) controller
	Kp_yaw=-1.9;//Yaw/sideslip(beta) controller
	_intergrator_AS=0;
	_intergrator_DLC=0;
	_intergrator_NSA=0;
	_intergrator_CR=0;
	_intergrator_LSA=0;
	_intergrator_RR=0;
	_min_thrust=0.00;
	_max_thrust=40.00;
	_min_deflection=-1.00;
	_max_deflection=1.00;
	//state machine variables
	Land=true;
	Tau_yaw=3;
	gamma=(float)(4*M_PI/180.0);
	gamma_land=(float)(2*M_PI/180.0);
	// dg=250;
	// dis_t=75;
	dg=250;
	dis_t=71.5;
	Ki_alt_lim_mpc=0.65;
	Ki_alt_lim_normal=0.5;
	_intergrator_alt_lim_mpc=0;
	_intergrator_alt_lim_normal=0;
	_min_alt_lim_mpc=-2.0;
	_max_alt_lim_mpc=2.0;
	_min_alt_lim_normal=-0.1;
	_max_alt_lim_normal=0.1;
	FW_UAV_ideal_vel_I_land=18.0;
	waypoint_early_switching_point = 75.0;
	_intergrator_guid1_lim=0;
	Ki_y_lim = 0.002;
	_min_guid1_lim = -4.0*M_PI/180.0;
	_max_guid1_lim = 4.0*M_PI/180.0;
	_Cw_min = -19.62;//-2g
	_Cw_max = 0.0;//-0g
	_Bw_min = -9.81;
	_Bw_max = 9.81;
}

void Controllers::initialise_NSADLC_HPF()
{
	float f_cut=8.5;//1/T_c from MATLAB code !!!CHECK!!!
	NSADLC_HPF.setfCut(f_cut);
	NSADLC_HPF.setY(0);
	NSADLC_HPF.setU(0);
}

//Actual Controllers
float Controllers::airspeed_controller(const Control_Data &state_data,float vbar_ref,const float dt){
	float edot_a=(state_data.airspeed-vbar_trim)-vbar_ref;
	// _intergrator_AS = _intergrator_AS + edot_a*dt;
	// float dT = -Ki_AS*_intergrator_AS - Kp_AS*edot_a;
	// return dT;

	//Anti-Windup
	// float Ta=1,Ti=1;
	// float edot_a_antwind=edot_a*1/Ti+-1/Ta*(dT_AS_lim-dT_AS);//added -1 to Ta loop as the error is state-ref not ref-state
	// _intergrator_AS = _intergrator_AS + edot_a_antwind*dt;
	// float dT = -Ki_AS*_intergrator_AS - Kp_AS*edot_a;
	// dT_AS=dT+T_Trim;//Add trim thrust
	// // std::cout<<"dT_AS : "<<dT_AS<<"\n";
	// dT_AS_lim=math::constrain(dT_AS,_min_thrust,_max_thrust);
	// return dT_AS_lim;

	//Intergrator conditional anti-windup
	if((_intergrator_AS + edot_a*dt)*-Ki_AS >= (_max_thrust-T_Trim)){
		_intergrator_AS = (_max_thrust-T_Trim)/-Ki_AS;
	}else if((_intergrator_AS + edot_a*dt)*-Ki_AS <= (_min_thrust-T_Trim)){
		_intergrator_AS = (_min_thrust-T_Trim)/-Ki_AS;
	}else{
		_intergrator_AS = _intergrator_AS + edot_a*dt;
	}

	float dT = -Ki_AS*_intergrator_AS - Kp_AS*edot_a;
	dT_AS=dT+T_Trim;//Add trim thrust
	// std::cout<<"dT_AS : "<<dT_AS<<"\n";
	dT_AS_lim=math::constrain(dT_AS,_min_thrust,_max_thrust);
	return dT_AS_lim;

}

void Controllers::NSADLC_controller(const Control_Data &state_data,float Cw_ref,const float dt,float *defl)
{
	//DLC
	float ew=-Cw_ref+state_data.Cw;
	float edot_f=NSADLC_HPF.update(ew);
	// _intergrator_DLC = _intergrator_DLC + edot_f*dt;
	// float df = -Ki_f*_intergrator_DLC;

	//Anti-Windup
	// float Ta_DLC=1,Ti_DLC=1;
	// float edot_f_antwind=edot_f*1/Ti_DLC-1/Ta_DLC*(dF_DLC_lim-dF_DLC);
	// _intergrator_DLC = _intergrator_DLC + edot_f_antwind*dt;
	// dF_DLC = -Ki_f*_intergrator_DLC;
	// dF_DLC_lim=math::constrain(dF_DLC,_min_deflection,_max_deflection);

	//Intergrator conditional anti-windup
	if((_intergrator_DLC + edot_f*dt)*-Ki_f >= (_max_deflection)){
		_intergrator_DLC = (_max_deflection)/-Ki_f;
	}else if((_intergrator_DLC + edot_f*dt)*-Ki_f <= (_min_deflection)){
		_intergrator_DLC = (_min_deflection)/-Ki_f;
	}else{
		_intergrator_DLC = _intergrator_DLC + edot_f*dt;
	}
	dF_DLC = -Ki_f*_intergrator_DLC;
	dF_DLC_lim=math::constrain(dF_DLC,_min_deflection,_max_deflection);

	//NSA
	float edot_c=-Cw_ref+state_data.Cw;
	// _intergrator_NSA=_intergrator_NSA+edot_c*dt;
	// float de = Cw_ref*Nc - _intergrator_NSA * Ki_e - state_data.Cw * Kc - state_data.q * Kq + Km * df;

	//Anti-windup
	// float Ta_NSA=1,Ti_NSA=1;
	// float edot_c_antwind=edot_c*1/Ti_NSA-1/Ta_NSA*(dE_NSA_lim-dE_NSA);
	// _intergrator_NSA=_intergrator_NSA+edot_c_antwind*dt;
	// float de = Cw_ref*Nc - _intergrator_NSA * Ki_e - state_data.Cw * Kc - state_data.q * Kq + Km * dF_DLC_lim;
	// dE_NSA=de+dE_Trim;//Add trim elevator
	// dE_NSA_lim=math::constrain(dE_NSA,_min_deflection,_max_deflection);


	//Intergrator conditional anti-windup
	if((_intergrator_NSA + edot_c*dt)*-Ki_e >= (_max_deflection-dE_Trim)){
		_intergrator_NSA = (_max_deflection-dE_Trim)/-Ki_e;
	}else if((_intergrator_NSA + edot_c*dt)*-Ki_e <= (_min_deflection-dE_Trim)){
		_intergrator_NSA = (_min_deflection-dE_Trim)/-Ki_e;
	}else{
		_intergrator_NSA = _intergrator_NSA + edot_c*dt;
	}
	float de = Cw_ref*Nc - _intergrator_NSA * Ki_e - state_data.Cw * Kc - state_data.q * Kq + Km * dF_DLC_lim;
	dE_NSA=de+dE_Trim;//Add trim elevator
	dE_NSA_lim=math::constrain(dE_NSA,_min_deflection,_max_deflection);

	//copy to defl array
	// *defl=de;
	// defl++;
	// *defl=df;
	*defl=dE_NSA_lim;
	defl++;
	*defl=dF_DLC_lim;
}

float Controllers::climb_rate_controller(const Control_Data &state_data,float hdot_ref,const float dt)
{
	float edot_r=-hdot_ref+state_data.h_dot;
	// _intergrator_CR=_intergrator_CR+edot_r*dt;
	// float Cw_ref=-Ki_CR*_intergrator_CR-Kp_CR*edot_r;
	// return Cw_ref;

	//Intergrator conditional anti-windup
	if((_intergrator_CR+edot_r*dt)*-Ki_CR >= (-_Cw_min)){
		_intergrator_CR = (-_Cw_min)/-Ki_CR;
	}else if((_intergrator_CR+edot_r*dt)*-Ki_CR <= (-_Cw_max)){
		_intergrator_CR = (-_Cw_max)/-Ki_CR;
	}else{
		_intergrator_CR=_intergrator_CR+edot_r*dt;
	}
	float Cw_ref=-Ki_CR*_intergrator_CR-Kp_CR*edot_r;
	return Cw_ref;
}

float Controllers::altitude_controller(const Control_Data &state_data,float h_ref,const float dt)
{
	float hdot_ref=-Kp_alt*(-1*state_data.posz-h_ref)+state_data.hdot_bar_ref; //!!!Check posz positive direction!!!
	return hdot_ref;
}

float Controllers::altitude_limit_intergrator_mpc(const Control_Data &state_data,float h_ref,const float dt)
{
	float herr=(-1*state_data.posz-h_ref);
	//Anti-windup
	// float Ti=1,Ta=1;
	// float herr_ant_wind=1/Ti*herr+1/Ta*(hdot_ref_int_lim_mpc-hdot_ref_int_mpc);
	// _intergrator_alt_lim_mpc=_intergrator_alt_lim_mpc+herr_ant_wind*dt;
	// hdot_ref_int_mpc=Ki_alt_lim_mpc*_intergrator_alt_lim_mpc; //!!!Check posz positive direction!!!
	// hdot_ref_int_lim_mpc=math::constrain(hdot_ref_int_mpc,_min_alt_lim_mpc,_max_alt_lim_mpc);
	// return hdot_ref_int_lim_mpc;

	//Intergrator conditional anti-windup
	if((_intergrator_alt_lim_mpc + herr*dt)*-Ki_alt_lim_mpc >= (_max_alt_lim_mpc)){
		_intergrator_alt_lim_mpc = (_max_alt_lim_mpc)/-Ki_alt_lim_mpc;
	}else if((_intergrator_alt_lim_mpc + herr*dt)*-Ki_alt_lim_mpc <= (_min_alt_lim_mpc)){
		_intergrator_alt_lim_mpc = (_min_alt_lim_mpc)/-Ki_alt_lim_mpc;
	}else{
		_intergrator_alt_lim_mpc = _intergrator_alt_lim_mpc + herr*dt;
	}
	hdot_ref_int_mpc=-Ki_alt_lim_mpc*_intergrator_alt_lim_mpc; //!!!Check posz positive direction!!!
	hdot_ref_int_lim_mpc=math::constrain(hdot_ref_int_mpc,_min_alt_lim_mpc,_max_alt_lim_mpc);
	return -hdot_ref_int_lim_mpc;

}

float Controllers::altitude_limit_intergrator_normal(const Control_Data &state_data,float h_ref,const float dt)
{
	float herr=(-1*state_data.posz-h_ref);
	//Anti-windup
	// float Ti=1,Ta=1;
	// float herr_ant_wind=1/Ti*herr+1/Ta*(hdot_ref_int_lim_normal-hdot_ref_int_normal);
	// _intergrator_alt_lim_normal=_intergrator_alt_lim_normal+herr_ant_wind*dt;
	// hdot_ref_int_normal=Ki_alt_lim_normal*_intergrator_alt_lim_normal; //!!!Check posz positive direction!!!
	// hdot_ref_int_lim_normal=math::constrain(hdot_ref_int_normal,_min_alt_lim_normal,_max_alt_lim_normal);
	// return hdot_ref_int_lim_normal;

	//Intergrator conditional anti-windup
	if((_intergrator_alt_lim_normal + herr*dt)*-Ki_alt_lim_normal >= (_max_alt_lim_normal)){
		_intergrator_alt_lim_normal = (_max_alt_lim_normal)/-Ki_alt_lim_normal;
	}else if((_intergrator_alt_lim_normal + herr*dt)*-Ki_alt_lim_normal <= (_min_alt_lim_normal)){
		_intergrator_alt_lim_normal = (_min_alt_lim_normal)/-Ki_alt_lim_normal;
	}else{
		_intergrator_alt_lim_normal = _intergrator_alt_lim_normal + herr*dt;
	}
	hdot_ref_int_normal=-Ki_alt_lim_normal*_intergrator_alt_lim_normal; //!!!Check posz positive direction!!!
	hdot_ref_int_lim_normal=math::constrain(hdot_ref_int_normal,_min_alt_lim_normal,_max_alt_lim_normal);
	return -hdot_ref_int_lim_normal;
}

float Controllers::LSA(const Control_Data &state_data,float Bw_ref,const float dt)
{
	float edot_b=state_data.Bw-Bw_ref;
	// _intergrator_LSA=_intergrator_LSA+edot_b*dt;
	// float dr_r=-Ki_LSA*_intergrator_LSA;
	// float dr=dr_r-state_data.Bw*K_B-state_data.r*K_r;
	// return dr;

	//Anti-windup
	// float Ti=1,Ta=1;
	// float edot_b_ant_wind=1/Ti*edot_b-1/Ta*(dR_LSA_lim-dR_LSA);
	// _intergrator_LSA=_intergrator_LSA+edot_b_ant_wind*dt;
	// float dr_r=-Ki_LSA*_intergrator_LSA;
	// float dr=dr_r-state_data.Bw*K_B-state_data.r*K_r;
	// dR_LSA=dr;
	// dR_LSA_lim=math::constrain(dR_LSA,_min_deflection,_max_deflection);
	// return dR_LSA_lim;

	//Intergrator conditional anti-windup
	if((_intergrator_LSA + edot_b*dt)*-Ki_LSA >= (_max_deflection)){
		_intergrator_LSA = (_max_deflection)/-Ki_LSA;
	}else if((_intergrator_LSA + edot_b*dt)*-Ki_LSA <= (_min_deflection)){
		_intergrator_LSA = (_min_deflection)/-Ki_LSA;
	}else{
		_intergrator_LSA = _intergrator_LSA + edot_b*dt;
	}
	float dr_r=-Ki_LSA*_intergrator_LSA;
	float dr=dr_r-state_data.Bw*K_B-state_data.r*K_r;
	dR_LSA=dr;
	dR_LSA_lim=math::constrain(dR_LSA,_min_deflection,_max_deflection);
	return dR_LSA_lim;
}

float Controllers::roll_rate_controller(const Control_Data &state_data,float p_ref,const float dt)
{
	float edot_p=state_data.p-p_ref;
	// _intergrator_RR=_intergrator_RR+edot_p*dt;
	// float da=-_intergrator_RR*Ki_RR-edot_p*Kp_RR;
	// return da;

	//Anti-windup
	// float Ti=1,Ta=1;
	// float edot_p_ant_wind=edot_p*1/Ti-1/Ta*(dA_RR_lim-dA_RR);
	// _intergrator_RR=_intergrator_RR+edot_p_ant_wind*dt;
	// float da=-_intergrator_RR*Ki_RR-edot_p*Kp_RR;
	// dA_RR=da;
	// dA_RR_lim=math::constrain(dA_RR,_min_deflection,_max_deflection);
	// return dA_RR_lim;

	if((_intergrator_RR + edot_p*dt)*-Ki_RR >= (_max_deflection)){
		_intergrator_RR = (_max_deflection)/-Ki_RR;
	}else if((_intergrator_RR + edot_p*dt)*-Ki_RR <= (_min_deflection)){
		_intergrator_RR = (_min_deflection)/-Ki_RR;
	}else{
		_intergrator_RR = _intergrator_RR + edot_p*dt;
	}
	float da=-_intergrator_RR*Ki_RR-edot_p*Kp_RR;
	dA_RR=da;
	dA_RR_lim=math::constrain(dA_RR,_min_deflection,_max_deflection);
	return dA_RR_lim;

}

float Controllers::roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt)
{
	float p_ref=-Kp_RA*(state_data.phi-phi_ref);
	return p_ref;
}

float Controllers::guidance_controller_1(const Control_Data &state_data,float phibar_ref,float y_ref,const float dt)
{
	float phi_ref=phibar_ref-Kp_G*(state_data.y-y_ref)-state_data.ydot*Kd_G;
	return phi_ref;
}

float Controllers::guidance_1_limited_intergrator(const Control_Data &state_data,float y_ref,const float dt)
{
	float yerr=(state_data.y-y_ref);
	//Anti-windup
	// float Ti=100.0,Ta=0.01;
	// float yerr_ant_wind=(float)1.0/Ti*yerr+(float)1.0/Ta*(ydot_ref_int_lim-ydot_ref_int);
	// _intergrator_guid1_lim=_intergrator_guid1_lim+yerr_ant_wind*dt;
	// ydot_ref_int=Ki_y_lim*_intergrator_guid1_lim; //!!!Check posz positive direction!!!
	// ydot_ref_int_lim=math::constrain(ydot_ref_int,_min_guid1_lim,_max_guid1_lim);
	// return ydot_ref_int_lim;

	if((_intergrator_guid1_lim + yerr*dt)*-Ki_y_lim >= (_max_guid1_lim)){
		_intergrator_guid1_lim = (_max_guid1_lim)/-Ki_y_lim;
	}else if((_intergrator_guid1_lim + yerr*dt)*-Ki_y_lim <= (_min_guid1_lim)){
		_intergrator_guid1_lim = (_min_guid1_lim)/-Ki_y_lim;
	}else{
		_intergrator_guid1_lim = _intergrator_guid1_lim + yerr*dt;
	}
	ydot_ref_int=-Ki_y_lim*_intergrator_guid1_lim; //!!!Check posz positive direction!!!
	ydot_ref_int_lim=math::constrain(ydot_ref_int,_min_guid1_lim,_max_guid1_lim);
	return -ydot_ref_int_lim;
}


float Controllers::yaw_controller(const Control_Data &state_data,float psi_ref,float psi_crab_error,const float dt)
{
	float edot_psi=psi_crab_error-psi_ref;
	// _intergrator_yaw=_intergrator_yaw+edot_psi*dt;
	// float Bw_ref=-Ki_yaw*_intergrator_yaw-Kp_yaw*edot_psi;
	// return Bw_ref;

	// Conditional Intergrator Windup
	if((_intergrator_yaw+edot_psi*dt)*-Ki_yaw >= (_Bw_max)){
		_intergrator_yaw = (_Bw_max)/-Ki_yaw;
	}else if((_intergrator_yaw+edot_psi*dt)*-Ki_yaw <= (_Bw_min)){
		_intergrator_yaw = (_Bw_min)/-Ki_yaw;
	}else{
		_intergrator_yaw=_intergrator_yaw+edot_psi*dt;
	}
	float Bw_ref=-Ki_yaw*_intergrator_yaw-Kp_yaw*edot_psi;
	Bw_ref=math::constrain(Bw_ref,_Bw_min,_Bw_max);
	return Bw_ref;
}

float Controllers::heading_controller(const Control_Data &state_data,float psi_ref)
{
	float psi_error,u_ref[3]={(float)cos(psi_ref),(float)sin(psi_ref),0},u[3]={(float)cos(state_data.psi),(float)sin(state_data.psi),0},u_D[3]={0,0,1};
	float p_mat[3];
	crossProduct(u,u_ref,p_mat);
	float x=dotProduct(p_mat,u_D);
	float sign=((x > 0) - (x < 0));
	psi_error= sign*(float)acos(dotProduct(u,u_ref));
	float phi_ref= psi_error*K_psi;
	return phi_ref;
}

void Controllers::guide_axis_alg(float S[2],float D[2],const Control_Data &state_data,float out[4])
{
	//track angle
	float psi_track=atan2((D[0]-S[0]),(D[1]-S[1]));

	//x and 	y calculation
	//conversion from inertial axis to track axis via matrix multiplication
	// float res[10][10],m1[10][10],m2[10][10];//result,matrix1,matrix2
	// int r1=2,c1=2,r2=2,c2=1;//row and column of matrix 1, row and column of matrix 2
	// m1[0][0]=cos(psi_track);m1[0][1]=sin(psi_track);m1[1][0]=-sin(psi_track);m1[1][1]=cos(psi_track);m2[0][0]=(state_data.posx-S[1]);m2[1][0]=(state_data.posy-S[0]);
	// matrix_mult(m1, m2,res,r1,c1,r2,c2);
	// out[0]=res[0][0];out[1]=res[1][0];out[2]=psi_track;

	//Alternative to guidance calculation
	matrix::Matrix<float,2,2> Guide_DCM;
	Guide_DCM(0,0)=cos(psi_track);Guide_DCM(0,1)=sin(psi_track);Guide_DCM(1,0)=-sin(psi_track);Guide_DCM(1,1)=cos(psi_track);
	matrix::Matrix<float,2,1> Guide_in_pose;
	Guide_in_pose(0,0)=(state_data.posx-S[1]);Guide_in_pose(1,0)=(state_data.posy-S[0]);
	matrix::Matrix<float,2,1> Guide_out_pose = Guide_DCM.operator*(Guide_in_pose);
	out[0]=Guide_out_pose(0,0);out[1]=Guide_out_pose(1,0);out[2]=psi_track;

	//ydot calculation
	// for(int i=0;i<10;i++){
	// 	for(int j=0;j<10;j++){
	// 		res[i][j]=0;
	// 		m1[i][j]=0;
	// 		m2[i][j]=0;
	// 	}
	// }
	// r1=2,c1=2,r2=2,c2=1;//row and column of matrix 1, row and column of matrix 2
	// m1[0][0]=cos(psi_track);m1[0][1]=sin(psi_track);m1[1][0]=-sin(psi_track);m1[1][1]=cos(psi_track);m2[0][0]=(state_data.velx_I);m2[1][0]=(state_data.vely_I);
	// matrix_mult(m1,m2,res,r1,c1,r2,c2);
	// out[3]=res[1][0];

	//Alternative to guidance ydot calculation
	matrix::Matrix<float,2,1> Guide_in_vel;
	Guide_in_vel(0,0)=(state_data.velx_I);Guide_in_vel(1,0)=(state_data.vely_I);
	matrix::Matrix<float,2,1> Guide_out_vel = Guide_DCM.operator*(Guide_in_vel);
	out[3]=Guide_out_vel(1,0);
}

void Controllers::waypoint_scheduler(float Destinations[][2],int Destination_size,const Control_Data &state_data,float out[2][2]) //!!!COMPLETE!!!
{
	if(Abort && !Anti_Abort){
       		//Reset waypoints
       		_loc_guide=1;
		_x_guide=0;
       		waypoint_END=false;
		PX4_INFO("Abort");
	}
	math::constrain(_loc_guide, 1, Destination_size); // Out of bound of Destinations array protection
	L_track=sqrt(pow((Destinations[_loc_guide][1]-Destinations[_loc_guide-1][1]),2)+pow((Destinations[_loc_guide][0]-Destinations[_loc_guide-1][0]),2));//Track length
	if(_x_guide>(L_track-waypoint_early_switching_point) && _loc_guide<Destination_size){
        	if(_loc_guide!=(Destination_size-1))//If not final destination then update waypoints
		{
	    	// std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[0][0]);//copy starting point coordinates into out array
		out[0][0]=Destinations[_loc_guide][0];out[0][1]=Destinations[_loc_guide][1];
		// std::copy(&Destinations[_loc_guide+1][0],&Destinations[_loc_guide+1][0]+1*2,&out[1][0]);//copy destination point coordinates into out array
            	out[1][0]=Destinations[_loc_guide+1][0];out[1][1]=Destinations[_loc_guide+1][1];
            	_loc_guide=_loc_guide+1;
		waypoint_END=false;
		}else
		{
		if(land_init){
			waypoint_END=true;//End waypoint navigation
		}
		// PX4_INFO("waypoint_END");
		// std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[0][0]);//Update last starting point
		out[0][0]=Destinations[_loc_guide][0];out[0][1]=Destinations[_loc_guide][1];
		// std::copy(&Destinations[_loc_guide+1][0],&Destinations[_loc_guide+1][0]+1*2,&out[1][0]);//keep desitination the same
		out[1][0]=Destinations[_loc_guide+1][0];out[1][1]=Destinations[_loc_guide+1][1];
 	        _loc_guide=_loc_guide+1;
		L_track=sqrt(pow((Destinations[_loc_guide][1]-Destinations[_loc_guide-1][1]),2)+pow((Destinations[_loc_guide][0]-Destinations[_loc_guide-1][0]),2));
        	}
    	}else
    	{ //Did not reach destination point yet
      	// std::copy(&Destinations[_loc_guide-1][0],&Destinations[_loc_guide-1][0]+1*2,&out[0][0]);//keep start point the same
	// out[0][0]=Destinations[_loc_guide-1][0];out[0][1]=Destinations[_loc_guide-1][1];
	// std::copy(&Destinations[_loc_guide][0],&Destinations[_loc_guide][0]+1*2,&out[1][0]);//keep destination the same
	// out[1][0]=Destinations[_loc_guide][0];out[1][1]=Destinations[_loc_guide][1];
	if(_loc_guide==Destination_size){ //Going to land
			// waypoint_END=true;
		if(_x_guide>(L_track) && !land_init){ //For continous looping
			//Reset waypoints
       			_loc_guide=1;
			_x_guide=0;
			PX4_INFO("Reset Waypoints");
		}else if(land_init){
			waypoint_END=true;
		}else{
			waypoint_END=false;
		}
	}
	out[0][0]=Destinations[_loc_guide-1][0];out[0][1]=Destinations[_loc_guide-1][1];
	out[1][0]=Destinations[_loc_guide][0];out[1][1]=Destinations[_loc_guide][1];
	}
}

void Controllers::navigation_controller(float D[][2],int Destination_size,const Control_Data &state_data,float out[4])
{
	//add waypoint_scheduler and guide_axis_alg function call outs
	float points[2][2],guide_val[4];
	waypoint_scheduler(D,Destination_size,state_data,points);
	guide_axis_alg(points[0],points[1],state_data,guide_val);
	_x_guide=guide_val[0];//update x in memory block

	for(int i=0;i<4;i++){
		out[i]=guide_val[i];
	}
}

float Controllers::guidance_controller_2(float guide_val[4],float y_ref){
	// workout and return psi_ref
	// float psi_ref=K_y*(0-guide_val[1])+guide_val[2];
	// return psi_ref;

	// TESTING FIXING INTAIL HEADING ISSUE (WORKS FOR NOW)
	float delta_psi_ref=K_y*(y_ref-guide_val[1]);
	delta_psi_ref=math::constrain(delta_psi_ref,(float)-M_PI/4,(float)M_PI/4); //Constrain delta heading referance (!!!Confirm!!!) (45 degree limit for now)
	float psi_ref=delta_psi_ref+guide_val[2];
	return psi_ref;
}

float Controllers::transition_multiplexer(float phi_ref_HG,float phi_ref_CT, float yerr){
	float y_e=Kd_G/Kp_G*vbar_trim;
    	float b_u=y_e;
    	float b_l=b_u/2;
	float r_HG=0;
    	//Transition curve
    	if(abs(yerr)>b_u){
        	r_HG=1;
	}else if(abs(yerr)<b_l){
        	r_HG=0;
	}else{
        	r_HG=sin((float)M_PI/2*(abs(yerr)-b_l)/(b_u-b_l));
	}
    	float r_CTC=1-r_HG;
    	float phi_ref=r_HG*phi_ref_HG+r_CTC*phi_ref_CT;
    	return phi_ref;
}

void Controllers::reset_integrators()
{
	_integrator_I[0][0]=0;
	_integrator_I[1][0]=0;
	_integrator_RA=0;
	_intergrator_AS=0;
	_intergrator_DLC=0;
	_intergrator_NSA=0;
	_intergrator_CR=0;
	_intergrator_LSA=0;
	_intergrator_RR=0;
	_intergrator_yaw=0;
	_intergrator_alt_lim_mpc=0;
	_intergrator_alt_lim_normal=0;
	_intergrator_guid1_lim=0;
}

void Controllers::initialise_integrators(const Control_Data &state_data)
{
	_integrator_I[0][0]=0;
	_integrator_I[1][0]=0;
	_integrator_RA=0;
	_intergrator_AS=-dT_pub/Ki_AS;
	_intergrator_CR=0;
	_intergrator_DLC=-dF_pub/Ki_f;
	float Cw_ref_init=-1*(-Kp_CR*state_data.h_dot-Ki_CR*state_data.h_dot);
	_intergrator_NSA=1/Ki_e*(Km*dF_pub+Nc*Cw_ref_init-Kc*state_data.Cw-Kq*state_data.q-dE_pub);
	_intergrator_yaw=0;
	_intergrator_LSA=1/Ki_LSA*(-K_B*state_data. Bw-K_r*state_data.r-dR_pub);
	//Assume phi_ref=0 at manual to auto transition
	float p_ref=-Kp_RA*state_data.phi;
	//Assume p_ref=0
	p_ref=0;//Testing
	_intergrator_RR=1/Ki_RR*(-Kp_RR*(state_data.p-p_ref)-dA_pub);// !!!Check if p_ref=0 is a valid assumption!!!
	_intergrator_alt_lim_mpc=0;
	_intergrator_alt_lim_normal=0;
	_intergrator_guid1_lim=0;

}

void Controllers::state_machine(Control_Data &state_data,float ref_out[4],float mp_pos[3],const float dt){
	float V_ground=sqrt(pow(state_data.velx_I,2)+pow(state_data.vely_I,2)+pow(state_data.velz_I,2));
	float dd=V_ground*Tau_yaw;  //distance of state changes
	if(state>=3){
		dd=V_ground_check*Tau_yaw;  //distance of state changes
	}else{
		dd=V_ground*Tau_yaw;  //distance of state changes
	}
	dd= math::constrain(dd,0.0f,(dis_t-5.0f));
	//flare calculation
	float h_land=0.0;//Flare height
	float d_flare=h_land/(float)tan(gamma_land);

    	if(waypoint_END==true){
        	if(Land==true){//Decision 1
            		if(state==0){
                	state=1;//final approach
			PX4_INFO("state 1");
            		}
		}else{
           		Abort=true;
           		state=0;
			Log_data=true;
			PX4_INFO("Land Flag is Disabled");
		}
        	if((L_track-_x_guide-d_flare)<dg && (L_track-_x_guide-d_flare)>=dis_t){
            		if(state==1){
                		if(abs(h_ref_SM-(-state_data.posz))<1){//Decision 2
                    			state=2;//Glideslope tracking
					PX4_INFO("state 2");
			    	}else{
                    			Abort=true;
                    			state=0;
					Log_data=true;
					PX4_INFO("state 2 check failed");
                		}
            		}
		}
        	if((L_track-_x_guide-d_flare)<dis_t && (L_track-_x_guide-d_flare)>dd){
            		if(state==2){
				// Display states that are checked #TODO COMBINE displayed values into two/three strings as log cannot show all
				// PX4_INFO("airspeed : \t%.4f",(double)state_data.airspeed);
				// PX4_INFO("hdot : \t%.4f",(double)state_data.h_dot);
				// PX4_INFO("psi_crab_error : \t%.4f",(double)(state_data.psi_crab_error*(float)(180.0/M_PI)));
				// PX4_INFO("theta : \t%.4f",(double)(state_data.theta*(float)(180/M_PI)));
				// PX4_INFO("phi : \t%.4f",(double)(state_data.phi*(float)(180/M_PI)));
				// PX4_INFO("y :  \t%.4f",(double)state_data.y);
				// PX4_INFO("herr :  \t%.4f",(double)abs((float)h_ref_SM-(-(float)state_data.posz)));

				PX4_INFO("airspeed : \t%.4f ; hdot : \t%.4f ;  psi_crab_error : \t%.4f",(double)state_data.airspeed,(double)state_data.h_dot,(double)(state_data.psi_crab_error*(float)(180.0/M_PI)));
				PX4_INFO("theta : \t%.4f ; phi : \t%.4f ; y : \t%.4f",(double)(state_data.theta*(float)(180/M_PI)),(double)(state_data.phi*(float)(180/M_PI)),(double)state_data.y);
				PX4_INFO("herr :  \t%.4f",(double)abs((float)h_ref_SM-(-(float)state_data.posz)));
                		if(state_data.airspeed>(float)15.00 && state_data.airspeed<(float)17.00 && state_data.h_dot>(float)-1.50 && abs(state_data.psi_crab_error)<(float)(8.0/180*M_PI) && state_data.theta>(float)(-6.0/180*M_PI) && abs(state_data.phi)<(float)(8.0/180*M_PI) && abs(state_data.y)<(float)1.50 && abs((float)h_ref_SM-(-(float)state_data.posz))<(float)0.10){//Decision 3
                    			state=3;//Pre-landing
					PX4_INFO("state 3");
					V_ground_check=V_ground;
				}else{
                    			Abort=true;
                    			state=0;
					Log_data=true;
					PX4_INFO("state 3 check failed");
                		}
            		}
		}
        	if((L_track-_x_guide-d_flare)<=dd){
            		if(state==3){
                		state=4;//decrab (cannot abort)
				PX4_INFO("state 4");
            		}
	    	}
		if((L_track-_x_guide)<=d_flare && d_flare>0){
            		if(state==4){
                		state=5;//flare (cannot abort)
				PX4_INFO("state 5");
            		}
	    	}
        	if(-state_data.posz<(float)0.5 && state_data.Cw<-(float)19){//TODO: CHOOSE ACCELARATION TRIGGER VALUE(20?)
			if(state==5 || state!=6){
            			state=6;//Landed
            			END=true;
				PX4_INFO("state 6");
				Log_data=true;
				// Display if hit target or not and miss distance
				float fw_pos_land=sqrt(pow(state_data.posx,2.0)+pow(state_data.posy,2.0));
				float mp_pos_land=sqrt(pow(mp_pos[0],2.0)+pow(mp_pos[1],2.0));
				// std::cout<<"Miss Distance : "<<(fw_pos_land-mp_pos_land)<<"\n";
				PX4_INFO("Miss Distance : \t%.4f",(double)(fw_pos_land-mp_pos_land));
				// std::cout<<"fw_x : "<<state_data.posx<<" fw_y : "<<state_data.posy<<" fw_z : "<<state_data.posz<<"\n";
				PX4_INFO("fw_x : \t%.4f ; fw_y : \t%.4f ; fw_z : \t%.4f",(double)state_data.posx,(double)state_data.posy,(double)state_data.posz);
				// std::cout<<"mp_x : "<<mp_pos[0]<<" mp_y : "<<mp_pos[1]<<" mp_z : "<<mp_pos[2]<<"\n";
				PX4_INFO("mp_x : \t%.4f ; mp_y : \t%.4f ; mp_z : \t%.4f",(double)mp_pos[0],(double)mp_pos[1],(double)mp_pos[2]);
				if(abs(mp_pos_land-fw_pos_land)<=(float)1.5){ //Depends on size of platform(for now it is 3m long therefore +-1.5m is range)
					// std::cout<<"Platform Hit\n";
					PX4_INFO("Platform Hit");
				}else{
					// std::cout<<"Ground Hit\n";
					PX4_INFO("Ground Hit");
				}
				// std::cout<<"TD_X : "<<TD_position[0]<<" TD_Y : "<<TD_position[1]<<" TD_Z : "<<TD_position[2]<<"\n";//Touch Down point
				// PX4_INFO("TD_x : \t%.4f TD_y : \t%.4f TD_z : \t%.4f",(double)TD_position[0],(double)TD_position[1],(double)TD_position[2]);
			}
        	}
	}else{
        	state=0;
        	Abort=false;Anti_Abort=false;
	}
    	switch(state){
        	case 0 : // Waypoint state
            		ref_out[0]=(float)17.4817+(-TD_position[2]);ref_out[1]=0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=false;//conventional flying
	    		//ref_out order: h_ref, vbar_ref, y_ref, yaw_ref
			break;
        	case 1 : //Final Approach
            		ref_out[0]=(float)17.4817+(-TD_position[2]);ref_out[1]=-2.0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=false;
			break;
        	case 2 : //Glideslope Tracking
            		ref_out[0]=(L_track-_x_guide-d_flare)*(float)tan(gamma)-(V_ground*(float)cos(gamma)*dt*(float)tan(gamma))+h_land+(-TD_position[2]);ref_out[1]=-2.0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=false;
			break;
        	case 3 : //Pre-Landing
            		ref_out[0]=(L_track-_x_guide-d_flare)*(float)tan(gamma)-(V_ground*(float)cos(gamma)*dt*(float)tan(gamma))+h_land+(-TD_position[2]);ref_out[1]=-2.0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=false;
			break;
        	case 4 : //Decrab
            		Anti_Abort=true;
            		ref_out[0]=(L_track-_x_guide-d_flare)*(float)tan(gamma)-(V_ground*(float)cos(gamma)*dt*(float)tan(gamma))+h_land+(-TD_position[2]);ref_out[1]=-2.0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=true;
			break;
		case 5 : //Flare
            		Anti_Abort=true;
            		ref_out[0]=(L_track-_x_guide)*(float)tan(gamma_land)-(V_ground*(float)cos(gamma_land)*dt*(float)tan(gamma_land))+(-TD_position[2]);ref_out[1]=-2.0;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=true;
			break;
        	case 6 : //Landed
            		ref_out[0]=0+(-TD_position[2]);ref_out[1]=-vbar_trim;ref_out[2]=0;ref_out[3]=0;
			yaw_ctrl=false;
			break;
	    }

//      if(state==3){ //Testing Abort
//         Abort=true;
// 	Log_data=true;
// 	state=0;
//      }

	h_ref_SM=ref_out[0];
	if(state<2){
		state_data.hdot_bar_ref=0;
	}else if(state>=2 && state<5){
		state_data.hdot_bar_ref=-V_ground*(float)sin(gamma);
	}else{
		//state_data.hdot_bar_ref=-V_ground*sin(gamma_land);// For flaring
		state_data.hdot_bar_ref=-V_ground*(float)sin(gamma);
	}
}

void Controllers::landing_point(Control_Data &state_data,float mp_pose[3],float mp_vel[3]){
	if(state>=1){
		//Get Moving Platform position(Defined in UAV direction NED)
		float d_ap = sqrt(pow((mp_pose[0]-state_data.posx),2)+pow((mp_pose[1]-state_data.posy),2));
		// float FW_UAV_vel_I=sqrt(pow(state_data.velx_I,2)+pow(state_data.vely_I,2)+pow(state_data.velz_I,2));
		float MP_vel_I=sqrt(pow(mp_vel[0],2)+pow(mp_vel[1],2)+pow(mp_vel[2],2));
		float land_time=d_ap/(FW_UAV_ideal_vel_I_land*(float)cos(gamma)-MP_vel_I); //Choose wether you want to apply ideal fw velocity or not
		// float MP_psi=atan2(mp_vel[1],mp_vel[0]); //moving platform heading
		// float da=FW_UAV_ideal_vel_I_land*cos(gamma)*land_time;

		//Testing TD generation values
		// std::cout<<"MP distance_x : "<<mp_pose[0]<<" MP distance_y : "<<mp_pose[1]<<"\n";
		// std::cout<<"MP_psi : "<<MP_psi*(float)(180.00/M_PI)<<"\n";
		// std::cout<<"FW UAV Vel I : "<<FW_UAV_vel_I<<"\n";

		if(MP_vel_I>(float)1){ //Removes the ambiguity when the platform is stationary
			// TD_position[0]=(state_data.posx+da*cos(MP_psi));
			// TD_position[1]=(state_data.posy+da*sin(MP_psi));
			TD_position[0] = mp_pose[0] + mp_vel[0] * land_time;
			TD_position[1] = mp_pose[1] + mp_vel[1] * land_time;
			TD_position[2]=	mp_pose[2];
		}else{
			TD_position[0]=mp_pose[0];
			TD_position[1]=mp_pose[1];
			TD_position[2]=mp_pose[2];
		}
	}else{
		TD_position[0]=mp_pose[0];
		TD_position[1]=mp_pose[1];
		TD_position[2]=mp_pose[2];
	}

}

void Controllers::mpc_referance_generator(float h_ref,float v_ref,float mpc_ref[]){
	for(int i=0;i<MPC_P;i++){
		if(state>=2){
			// if(i<MPC_P){
			// 	mpc_ref[i]=h_ref+(i-1)*tan(-gamma)*MPC_Ts;
			// }else{
			// 	mpc_ref[i]=v_ref;
			// }
			mpc_ref[i*2]=h_ref+(i)*(float)tan(-gamma)*MPC_Ts; //!!! FIX: ADD vehicle ground velocity component!!!
			mpc_ref[i*2+1]=v_ref;
		}else{
			// if(i<MPC_P){
			// 	mpc_ref[i]=h_ref;
			// }else{
			// 	mpc_ref[i]=v_ref;
			// }
			mpc_ref[i*2]=h_ref;
			mpc_ref[i*2+1]=v_ref;
		}
	}
}

void Controllers::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);
}

int parameters_init(struct param_handles *handles)
{
	/* PID parameters */
	handles->hdng_p 	=	param_find("EXFW_HDNG_P");
	handles->roll_p 	=	param_find("EXFW_ROLL_P");
	handles->pitch_p 	=	param_find("EXFW_PITCH_P");

	return 0;
}

int parameters_update(const struct param_handles *handles, struct params *parameters)
{
	param_get(handles->hdng_p, &(parameters->hdng_p));
	param_get(handles->roll_p, &(parameters->roll_p));
	param_get(handles->pitch_p, &(parameters->pitch_p));

	return 0;
}


/* Main Thread */
//int fixedwing_custom_control_thread_main(int argc, char *argv[])

void Controllers::Run()
{

	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (should_exit()) {
		airspeed_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}



	perf_begin(_loop_perf);

	vehicle_attitude_s att;

	if (_att_sub.update(&att)) {

	vehicle_control_mode_poll();

	/*Get commander state*/
	commander_state_s com_state{};
	commander_state_sub.copy(&com_state);

	vehicle_status_s vstatus{};
	vstatus_sub.copy(&vstatus);

	/*Set EKF Origin when armed*/
	bool armed = (vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	if(armed && !EKF_Origin_set){
			vehicle_gps_position_s gps_pos{};
			gps_pos_sub.copy(&gps_pos);
			vehicle_command_s veh_com{};
			veh_com.command = vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
			veh_com.target_system = vstatus.system_id;
			veh_com.target_component = vstatus.component_id;
			// veh_com.param1 = (float) 0.0;
			veh_com.param5 = (double)(gps_pos.lat * 1.e-7);
			veh_com.param6 = (double)(gps_pos.lon * 1.e-7);
			veh_com.param7 = (float)(gps_pos.alt * 1.e-3f);
			vehicle_command_pub.publish(veh_com);
			EKF_Origin_set=true;
			PX4_INFO("Set New EKF Origin When Armed");
	}

	if(!armed){
		EKF_Origin_set=false;
	}

	// if(_vcontrol_mode.flag_control_position_enabled || _vcontrol_mode.flag_control_offboard_enabled){ //FIX CONTROL MODE

	if(!_vcontrol_mode.flag_control_manual_enabled || (com_state.main_state == com_state.MAIN_STATE_POSCTL || com_state.main_state == com_state.MAIN_STATE_OFFBOARD || com_state.main_state == com_state.MAIN_STATE_ALTCTL || com_state.main_state == com_state.MAIN_STATE_ACRO)){ //FIX CONTROL MODE

	if(!_vcontrol_mode.flag_control_manual_enabled && !mcl_disp){
		// std::cout<<"Manual Control Signal Lost\n";
		PX4_INFO("Manual Control Signal Lost");
		mcl_disp=true;
	}

	// airspeed_validated_s airspeed;
	// airspeed_s airspeed;
	// if(airspeed_sub.update(&airspeed)){
	vehicle_local_position_s local_position;
	if(_local_pos_sub.update(&local_position)){
		// check for parameter updates
		if (parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			parameter_update_sub.copy(&pupdate);

			// if a param update occured, re-read our parameters
			parameters_update(&ph, &p);
		}

		bool global_sp_updated=global_sp_sub.updated();
		//bool manual_control_setpoint_updated=manual_control_setpoint_sub.updated();

		// const float dt = math::constrain((airspeed.timestamp - _last_run) * 1e-6f, 0.002f, 0.4f);
		// _last_run = airspeed.timestamp;

		const float dt = math::constrain((local_position.timestamp - _last_run) * 1e-6f, 0.002f, 0.4f);
		_last_run = local_position.timestamp;
		// std::cout<<"local position dt : "<<dt<<"\n";
		// std::cout<<"local position time : "<<local_position.timestamp<<"\n";

		/*calculate rotation matix(DCM) and euler angles(CHECK IF CORRECT AS IT IS DIRECTLY FROM FixedwingAttitudeControl.cpp)*/
		matrix::Dcmf R = matrix::Quatf(att.q);
		matrix::Dcmf DCM =R.transpose();
		const matrix::Eulerf euler_angles(R);

		/*get local copy of estimator states*/
				//orb_copy(ORB_ID(estimator_states), estim_sub, &estim);
		estimator_status_s estim{};
		estim_sub.copy(&estim);

		/*get local copy of angular velocity states*/
				// orb_copy(ORB_ID(vehicle_angular_velocity), ang_vel_sub, &ang_vel);
		vehicle_angular_velocity_s ang_vel{};
		ang_vel_sub.copy(&ang_vel);

		/*get local copy of accelaration states*/
		vehicle_acceleration_s accel{};
		accel_sub.copy(&accel);

		/*get local copy of airspeed state*/
				// orb_copy(ORB_ID(airspeed_validated), airspeed_sub, &airspeed);
		airspeed_validated_s airspeed{};
		airspeed_sub.copy(&airspeed);

		// airspeed_s airspeed;
		// airspeed_sub.copy(&airspeed);

		airspeed_s airspeed_sens;
		airspeed_sens_sub.copy(&airspeed_sens);

		moving_platform_s moving_platform{};
		_mov_plat_sub.copy(&moving_platform);

		/*get local copy of mpc out states*/
		mpc_outputs_s mpc_out{};
		mpc_out_sub.copy(&mpc_out);

		fw_custom_control_testing_s fw_custom_control_testing_setpoints{};
		_fw_custom_control_testing_sub.copy(&fw_custom_control_testing_setpoints);

		fw_custom_control_testing_mode_s fw_custom_control_testing_modes{};
		_fw_custom_control_testing_modes_sub.copy(&fw_custom_control_testing_modes);

		fw_custom_control_testing_lateral_s fw_custom_control_testing_lateral{};
		_fw_custom_control_testing_lateral_sub.copy(&fw_custom_control_testing_lateral);

		vehicle_status_flags_s veh_status_flgs{};
		veh_status_flgs_sub.copy(&veh_status_flgs);

		// Setup Home Postion Manually
		// if(!Home_Pose_set){
		// 	vehicle_command_s veh_com{};
		// 	veh_com.command = (uint16_t) 179; //VEHICLE_CMD_DO_SET_HOME
		// 	veh_com.target_system = vstatus.system_id;
		// 	veh_com.target_component = vstatus.component_id;
		// 	veh_com.param1 = (float) 0.0;
		// 	veh_com.param5 = (double) -34.0469250;
		// 	veh_com.param6 = (double) 18.7403705;
		// 	veh_com.param7 = (float) 9.194;
		// 	vehicle_command_pub.publish(veh_com);
		// 	Home_Pose_set=true;
		// 	PX4_INFO("Set Manual Home Postion");
		// }

		// if(!EKF_Origin_set){
		// 	vehicle_command_s veh_com{};
		// 	veh_com.command = (uint16_t) 100000; //VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
		// 	veh_com.target_system = vstatus.system_id;
		// 	veh_com.target_component = vstatus.component_id;
		// 	// veh_com.param1 = (float) 0.0;
		// 	veh_com.param5 = (double) -34.0473810;
		// 	veh_com.param6 = (double) 18.7405320;
		// 	veh_com.param7 = (float) 10.3 +0.1718f;
		// 	vehicle_command_pub.publish(veh_com);
		// 	EKF_Origin_set=true;
		// }



		/*get alpha and beta*/
		//get body velocity
		// const matrix::Vector3f vel_inertia=matrix::Vector3f(estim.states[4],estim.states[5],estim.states[6]);
		const matrix::Vector3f vel_inertia=matrix::Vector3f(local_position.vx,local_position.vy,local_position.vz);
		const matrix::Vector3f vel_b=DCM.operator*(vel_inertia);


		//work out polar coordinates
		float vbar=sqrt(pow(vel_b(0),2)+pow(vel_b(1),2)+pow(vel_b(2),2));
		float alpha=atan2(vel_b(2),vel_b(0));
		float beta=asin(vel_b(1)/vbar);

		// Workout offset between local postion referance point and home set point
		// home_position_s home_pose{};
		// home_position_sub.copy(&home_pose);
		// struct map_projection_reference_s ref_pos;
		// map_projection_init(&ref_pos, home_pose.lat, home_pose.lon);
		// float x_loc_ref_diff, y_loc_ref_diff;
		// map_projection_project(&ref_pos, local_position.ref_lat, local_position.ref_lon, &x_loc_ref_diff, &y_loc_ref_diff);
		// float z_loc_ref_diff = -(local_position.ref_alt-home_pose.alt);

		/*Define Control Data*/
		Control_Data control_input{};
		control_input.phi=euler_angles.phi();
		control_input.theta=euler_angles.theta();
		control_input.psi=euler_angles.psi();
		control_input.p=ang_vel.xyz[0];
		control_input.q=ang_vel.xyz[1];
		control_input.r=ang_vel.xyz[2];
		// control_input.airspeed=airspeed.equivalent_airspeed_m_s;//check
		control_input.airspeed=airspeed.calibrated_airspeed_m_s;
		// control_input.airspeed=airspeed.true_airspeed_m_s;//check
		control_input.alpha=alpha;
		control_input.beta=beta;
		// control_input.posx=estim.states[7];
		// control_input.posy=estim.states[8];
		// control_input.posz=estim.states[9];
		control_input.posx=local_position.x;
		control_input.posy=local_position.y;
		control_input.posz=local_position.z;
		// control_input.posx=local_position.x - (-x_loc_ref_diff); //compensate for local position referance point difference with home point
		// control_input.posy=local_position.y - (-y_loc_ref_diff);
		// control_input.posz=local_position.z - (-z_loc_ref_diff);
		// control_input.h_dot=-estim.states[6];
		control_input.h_dot=-local_position.vz;
		control_input.Bw=accel.xyz[1];//!!!check!!!
		control_input.Cw=accel.xyz[2];//!!!check!!!
		// control_input.velx_I=estim.states[4];
		// control_input.vely_I=estim.states[5];
		// control_input.velz_I=estim.states[6];
		control_input.velx_I=local_position.vx;
		control_input.vely_I=local_position.vy;
		control_input.velz_I=local_position.vz;
		control_input.y=y_old;
		control_input.ydot=ydot_old;
		control_input.psi_crab_error=psi_crab_error_old;

		//Test Psi(heading) intilisation
		// printf("Controllers \n");
		// printf("Controllers Psi :    %f\n",(double)control_input.psi*180/M_PI);
		// printf("\n");

		// if(disp_count%1==0 && state!=6){
			// std::cout<<"fw_px4_x : "<<control_input.posx<<" fw_px4_y : "<<control_input.posy<<" fw_px4_z : "<<-control_input.posz<<"\n";
			// std::cout<<"Moving Platform posex : "<<moving_platform.mp_pose[0]<<" Moving Platform posey : "<<moving_platform.mp_pose[1]<<" Moving Platform posez : "<<-moving_platform.mp_pose[2]<<"\n";
			// std::cout<<"FW_px4_X : "<<local_position.x<<"FW_px4_Y"<<local_position.y<<"\n";
			// std::cout<<"ECL Status : "<<std::bitset<24>(estim.control_mode_flags)<<"\n";
			// std::cout<<"Wind_North : "<<wind.windspeed_north<<" Wind_East : "<<wind.windspeed_east<<"\n";
			// std::cout<<"Airspeed : "<<airspeed_sens.true_airspeed_m_s<<"\n";
		// }

				if (global_sp_updated) {
					// struct position_setpoint_triplet_s triplet;
					// orb_copy(ORB_ID(position_setpoint_triplet), global_sp_sub, &triplet);
					// memcpy(&global_sp, &triplet.current, sizeof(global_sp));
					position_setpoint_triplet_s triplet{};
					global_sp_sub.copy(&triplet);
				}


				/* get the system status and the flight mode we're in */
				//orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				float SM_ref[4];
				if(fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode){
					/*Run State Machine*/
					float touch_down_point[3] = {0.0f,0.0f,0.0f};
					float mp_velocity[3] = {0.0f,0.0f,0.0f};
					state_machine(control_input,SM_ref,touch_down_point,dt);
					/*Run Landing point algorithm*/
					landing_point(control_input,touch_down_point,mp_velocity);
				}

				// Publish time
				// float publish_time = hrt_absolute_time()*1e-6f;

				/*Run Landing point algorithm*/
				// landing_point(control_input,moving_platform.mp_pose,moving_platform.mp_vel);

				//!!!TESTING!!!
				// std::cout<<"Moving Platform posez : "<<moving_platform.mp_pose[2]<<"\n";


				/*Run Controllers*/
				float dA=0,dE=0,dF=0,dR=0,dT=0,hdot_ref=0,guide_val[4],href=0,vbar_ref=0,phi_ref=0,y_ref=0.0f,yaw_ref=0.0f,psi_crab_error=0.0f,Cw_ref=0.0f,Bw_ref=0.0f,p_ref=0.0f;
				// float dA=0,dE=0,dF=0,dR=0,dT=0,vbar_ref=0,hdot_ref=0,href=0;
				// if((control_input.posz<-4 && (double)control_input.airspeed>5) || controllers_activate){
				if(1){
				//if(1){
					if(!controllers_activate){
						// printf("!!!Activated Controllers!!!\n");
						PX4_INFO("!!!Activated Controllers!!!");
						//_fw_cust_ctrls.reset_integrators();
						//reset_integrators();
						if(!manual_mode){//started in automode
							reset_integrators();
							// std::cout<<"Reset Intergrators\n";
							PX4_INFO("Reset Intergrators");
						}else{ //transition from manual to auto mode
							// std::cout<<"Initialise Intergrators\n";
							PX4_INFO("Initialise Intergrators");
							initialise_integrators(control_input);
						}
						// printf("\n");
						alt_cap = -control_input.posz;

					}
					controllers_activate=true;
					manual_mode=false;

					if(fw_custom_control_testing_modes.land_mode){
						href=SM_ref[0];//altitude (Make a way to trigger )
						vbar_ref=SM_ref[1];
					}else if(fw_custom_control_testing_modes.altitude_mode){
						href=alt_cap + math::constrain(fw_custom_control_testing_setpoints.altitude_step, -15.0f, 15.0f);//Altitude setpoint with step
						vbar_ref=0.0f;//Airspeed setpoint (with respect to trim speed(18m/s))
					}else if(fw_custom_control_testing_modes.airspeed_mode){
						vbar_ref=0.0f + math::constrain(fw_custom_control_testing_setpoints.airspeed_step, -2.0f, 2.0f);//Airspeed setpoint (with respect to trim speed(18m/s)) + step
						href=alt_cap;
					}else{
						href=alt_cap;
						vbar_ref=0.0f;//Airspeed setpoint (with respect to trim speed(18m/s))
					}

					//MPC referance generator!!!
					// mpc_referance_generator(href,vbar_ref,mpc_ref_in);
					if(fw_custom_control_testing_modes.mpc_airspeed_mode){
						mpc_ref_in[0]=alt_cap;
						mpc_ref_in[1]=0.0f + math::constrain(fw_custom_control_testing_setpoints.mpc_airspeed_step, -2.0f, 2.0f);
						mpc_ramp_init = false;
					}else if(fw_custom_control_testing_modes.mpc_altitude_mode){
						mpc_ref_in[0]=alt_cap + math::constrain(fw_custom_control_testing_setpoints.mpc_altitude_step, -15.0f, 15.0f);
						mpc_ref_in[1]=0.0f;
						mpc_ramp_init = false;
					}else if(fw_custom_control_testing_modes.mpc_ramp_mode && fw_custom_control_testing_setpoints.mpc_altitude_ramp>0){
						if(!mpc_ramp_init){
							mpc_ramp_origin[0]=control_input.posx;
							mpc_ramp_origin[1]=control_input.posy;
							mpc_ramp_init = true;
							mpc_ramp_max_dist = alt_cap/((float)tan(gamma));
						}
						float mpc_ramp_dist = sqrt(pow(control_input.posx-mpc_ramp_origin[0],2)+pow(control_input.posy-mpc_ramp_origin[1],2));
						mpc_ref_in[0]=math::constrain(((mpc_ramp_max_dist - mpc_ramp_dist) * (float)tan(gamma)),15.0f, alt_cap);
						mpc_ref_in[1]=0.0f;
					}else if(fw_custom_control_testing_modes.mpc_land_mode){
						mpc_ref_in[0]=SM_ref[0];
						mpc_ref_in[1]=SM_ref[1];
						mpc_ramp_init = false;
					}else{
						mpc_ref_in[0]=alt_cap;
						mpc_ref_in[1]=0.0f;
						mpc_ramp_init = false;
					}
					mpc_h=-control_input.posz;
					mpc_vbar=control_input.airspeed-vbar_trim;

					//MPC activator
					// if(state>=1 && _vcontrol_mode.flag_control_offboard_enabled){
					if(!veh_status_flgs.offboard_control_signal_lost && (fw_custom_control_testing_modes.mpc_airspeed_mode||fw_custom_control_testing_modes.mpc_altitude_mode||fw_custom_control_testing_modes.mpc_ramp_mode||fw_custom_control_testing_modes.mpc_land_mode)){

						//MPC controller
						if(!mpc_activate){
							// std::cout<<"Activate MPC"<<"\n";
							PX4_INFO("Activate MPC");
							mpc_activate=true;
							_intergrator_alt_lim_mpc=0;//Reset altitude limit integrator
						}

						if(fw_custom_control_testing_modes.mpc_ramp_mode || fw_custom_control_testing_modes.mpc_land_mode){
							//Limit intergrator calculation
							float Lim_Int=altitude_limit_intergrator_mpc(control_input,mpc_ref_in[0],dt);
							//MPC inputs(Check if you can publish here then wait for outputs before continuing)

							//MPC outputs
							hdot_ref=mpc_out.mpc_mv_out[0]-Lim_Int;
						}else{
							hdot_ref=mpc_out.mpc_mv_out[0];
						}

						dT=mpc_out.mpc_mv_out[1]+T_Trim;
						dT=math::constrain(dT,_min_thrust,_max_thrust);

						clas_alt_activate=false;
					}else{
						//Altitude controller
						hdot_ref=altitude_controller(control_input,href,dt);

						// Testing
						if(!clas_alt_activate){
							// std::cout<<"Activate Classical Altitude Controller"<<"\n";
							PX4_INFO("Activate Classical Altitude Controller");
							clas_alt_activate=true;
							_intergrator_alt_lim_normal=0;//Reset altitude limit integrator
						}
						float Lim_Int=altitude_limit_intergrator_normal(control_input,href,dt);
						hdot_ref=hdot_ref-Lim_Int;

						//Airspeed controller
						dT=airspeed_controller(control_input,vbar_ref,dt);
						mpc_activate=false;

						// Normal climbrate controller gains
						// Ki_CR=1.0130;//Original CR gain
						// Kp_CR=2.4960;//Original CR gain
					}

					// Longitudinal Controllers Flight Test
					if(fw_custom_control_testing_modes.climbrate_mode){
						hdot_ref = 0.0f + math::constrain(fw_custom_control_testing_setpoints.climbrate_step, -3.0f, 3.0f); //Climb rate step
					}else{
						hdot_ref=math::constrain(hdot_ref, _hdot_min, _hdot_max); //Altitude controller run
					}


					// Climbrate Testing
					// hdot_ref = -1.0f ;

					//Climbrate controller
					Cw_ref=0.0;
					Cw_ref=climb_rate_controller(control_input,hdot_ref,dt);
					//NSADLC controller
					Cw_ref=-Cw_ref;
					Cw_ref=math::constrain(Cw_ref, _Cw_min, _Cw_max);
					float defl[2];
					NSADLC_controller(control_input,Cw_ref,dt,&defl[0]);
					dE=defl[0];
					dF=defl[1];


					//Lateral Controllers
					if(!fw_custom_control_testing_lateral.lateral_control_disable){
					//Waypoints
					// float destinations[][2]={{0,0},{15000,0}};//Straight flight
					// float destinations[][2]={{0,0},{1000,0},{1000,500},{-1000,500},{-1000,0},{-750,0},{TD_position[1],TD_position[0]}};//Landing testing bigger Rectangle
					// float destinations[][2]={{0,0},{200,0},{200,250},{-500,250},{-500,0},{-350,0},{TD_position[1],TD_position[0]}};//HRF Gazebo x direction Waypoints
					// float destinations[][2]={{0,0},{0,200},{-250,200},{-250,-500},{0,-500},{0,-350},{TD_position[1],TD_position[0]}};//HRF North Waypoints
					// float destinations[][2]={{0.0f,0.0f},{-54.7573,192.3581},{-295.2049,123.9115},{-103.5545,-549.3419},{136.8932,-480.8953},{95.8252,-336.6267},{0.0f,0.0f}};//HRF Runway Aligned Waypoints
					float destinations[][2]={{0.0f,0.0f},{-55.5401,192.1336},{-295.7070,122.7085},{-198.5119,-213.5252},{-101.3168,-549.7590},{138.8502,-480.3339},{97.1951,-336.2337},{0.0f,0.0f}};//HRF Runway Aligned Waypoints Actual (added extra waypoint for capture)

					int destination_size=sizeof(destinations)/sizeof(destinations[0])-1;

					// Waypoint capture
					if(!waypoint_capture){
						// Get closest waypoint
						int close_loc = 1;
						float close_dist = sqrt(pow((control_input.posx-destinations[0][1]),2)+pow((control_input.posy-destinations[0][0]),2));
						for(int des_pos = 1; des_pos<destination_size; des_pos++){
							float cur_dist = sqrt(pow((control_input.posx-destinations[des_pos][1]),2)+pow((control_input.posy-destinations[des_pos][0]),2));
							if(cur_dist<close_dist){
								close_dist = cur_dist;
								close_loc = des_pos + 1;
							}
						}
						_loc_guide = close_loc;
						_x_guide = 0;
						waypoint_capture = true;
						if((fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode) && _loc_guide>=6){ //Reset if waypoint capture to close to land point
							PX4_INFO("Waypoint \t%d is too close to land point, therefore Waypoint Choosen as Source is: 0",_loc_guide-1);
							_loc_guide = 1;
						}else{
							PX4_INFO("Waypoint Choosen as Source: \t%d",_loc_guide-1);
						}
						land_init = false;
					}

					if((fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode) && !land_init){ //Protection incase land mode is activated while autopilot is already running
						if((fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode) && _loc_guide>=6){ //Reset if waypoint capture to close to land point
							PX4_INFO("Waypoint \t%d is too close to land point, therefore Waypoint Choosen as Source is: 0",_loc_guide-1);
							_loc_guide = 1;
						}
						land_init = true;
					}

					if(!(fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode)){
						land_init = false;
					}

					//Navigation controller
					navigation_controller(destinations,destination_size,control_input,guide_val);
					//Guidance and Heading controllers
					// float y_ref=SM_ref[2];
					y_ref=0.0f;
					if(fw_custom_control_testing_modes.ct_mode){
						y_ref=0.0f + math::constrain(fw_custom_control_testing_setpoints.ct_err_step,-20.0f,20.0f);
					}else if(fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode){
						y_ref=SM_ref[2];
					}else{
						y_ref=0.0f;
					}

					psi_ref_out=guidance_controller_2(guide_val,y_ref);
					float phi_ref_HG=heading_controller(control_input,psi_ref_out);
					//Cross-Track controller
					float phibar_ref=0;
					control_input.y=guide_val[1];//check
					control_input.ydot=guide_val[3];//check
					y_old=control_input.y;
					ydot_old=control_input.ydot;
					// float phi_ref_CT=guidance_controller_1(control_input,phibar_ref,y_ref,dt) - guidance_1_limited_intergrator(control_input, y_ref, dt);
					float phi_ref_CT=guidance_controller_1(control_input,phibar_ref,y_ref,dt);
					//Transition Multiplexer
					phi_ref=transition_multiplexer(phi_ref_HG,phi_ref_CT, (control_input.y-y_ref));
					// phi_ref = math::constrain(phi_ref, _phi_min, _phi_max);

					if(fw_custom_control_testing_modes.roll_angle_mode){
						phi_ref=0.0f + math::constrain(fw_custom_control_testing_setpoints.roll_angle_step,-30.0f,30.0f) * (float)(M_PI/180.0);
					}else{
						phi_ref = math::constrain(phi_ref, _phi_min, _phi_max);
					}

					//Roll Angle controller
					p_ref=roll_angle_controller(control_input,phi_ref,dt);
					//Roll Rate controller
					dA=roll_rate_controller(control_input,p_ref,dt);
					//Yaw controller
					// float yaw_ref=SM_ref[3];
					yaw_ref=0.0f;
					// Work out crab error
					psi_crab_error=0.0f;
					float psi_track=guide_val[2];
					float u_ref[3]={(float)cos(psi_track),(float)sin(psi_track),0},u[3]={(float)cos(control_input.psi),(float)sin(control_input.psi),0},u_D[3]={0,0,1};
					float p_mat[3];
					crossProduct(u,u_ref,p_mat);
					float x=dotProduct(p_mat,u_D);
					float sign=((x > 0) - (x < 0));
					psi_crab_error= sign*(float)acos(dotProduct(u,u_ref));
					control_input.psi_crab_error=psi_crab_error;
					psi_crab_error_old=psi_crab_error;
					if(fw_custom_control_testing_modes.sideslip_mode){
						yaw_ref=0.0f + math::constrain(fw_custom_control_testing_setpoints.sideslip_step,-15.0f,15.0f) * (float)(M_PI/180.0);
						// Only activate yaw control on straights
						if(abs(psi_crab_error)<=(float)(15.0/180.0*M_PI)){
							yaw_ctrl = true;
						}else{
							yaw_ctrl = false;
						}
					}else if(fw_custom_control_testing_modes.land_mode || fw_custom_control_testing_modes.mpc_land_mode){
						yaw_ref=SM_ref[3];
					}else{
						yaw_ref=0.0f;
						yaw_ctrl = false;
					}

					//conventional flight Bw_ref=0
					Bw_ref=0.0;
					if(yaw_ctrl){
						Bw_ref=yaw_controller(control_input,yaw_ref,psi_crab_error,dt);
					}else{
						Bw_ref=0.0;
					}

					//LSA controller
					dR=LSA(control_input,Bw_ref,dt);
					//Testing rudder command
					// std::cout<<"Controllers dR : "<<dR<<"\n";
					}else{
						dA = 0.0f;
						dR = 0.0f;
						waypoint_capture = false;
					}

				// //Auto Airspeed Control
				// dT=airspeed_controller(control_input,vbar_ref,dt);


					// disp_count_ctrl++;

					//Controller actuator command save
					dA_ctrl=dA;
					dE_ctrl=dE;
					dR_ctrl=dR;
					dT_ctrl=dT;
					dF_ctrl=dF;
				}


				/*Intial set up to get to equilabruim position*/
				//Problem with deflections not being able to be detected when small.
				//Fix the deflection normilisation
				// Normalised between -1 and 1 for dA,dE and dR
				// Normalesed between -1 and 1 on input but converts it to -3500 to 3500 on output.
				//climb
			/*	if(control_input.posz>-7 && !controllers_activate){
					// if((double)euler_angles.phi()<15*M_PI/180){
					// 	dA=-0.015;
					// }else{
					// 	dA=0;
					// }
					dA=0;
					if((double)euler_angles.theta()<30*M_PI/180){
						dE=-0.0767-0.1*M_PI/180;
					}else{
						dE=0;
					}

					dR=0;
					dT=6.6102+33;
					dF=0;
					//dT=0.03;
					//dT=((350.00)/3500.00);//temporary compensation
				}

				if(control_input.posz<-7 && !controllers_activate){
					dA=0;
					dR=0;
					dF=0;
					if((double)euler_angles.theta()>0.0355){
						dE=+0.0767+0.1*M_PI/180;
					}else if((double)euler_angles.theta()<-0.0355){
						dE=-0.0767-0.1*M_PI/180;
					}else{
						dE=-0.0767-0.0001;
					}
					if((double)control_input.airspeed>35){
						dT=6.6102;
					}else if((double)control_input.airspeed<35){
						dT=6.6102;
					}
				}
			*/

				// }





				//Normalisation of deflections to -1 and 1 for mixer
				// dA=dA;
				// dE=dE;
				// dR=dR;
				dT=dT/_max_thrust;



				//!!!CHECK BELOW!!!

				/* Gazebo axes compensation for SIMULATION*/
				dA=-dA;
				dE=-dE;
				dR=-dR;
				dF=-dF;

				/* Compensating for phyical control surface deformation for PIXHAWK 4 CODE*/
				// dA=-dA * (float) 2.223525;
				// dE=-dE * (float)1.8193;// Scaler is for controller only
				// // dT=dT;
				// dR=-dR * (float)1.60315;
				// dF=-dF * (float) 2.5331025;

				//If Landed
				// if(state==6 || control_input.posz>=(float)0.1){
				// 	dA=(float)0.0;
				// 	dE=(float)0.0;
				// 	dR=(float)0.0;
				// 	dF=(float)0.0;
				// 	dT=(float)0.0;
				// }

				// if(state==2 || state==3 || state==4 || state==5){
				// 	std::cout<<"Cw : "<<control_input.Cw<<"\n";
				// 	std::cout<<"hdot : "<<control_input.h_dot<<"\n";
				// 	std::cout<<"theta : "<<control_input.theta<<"\n";
				// 	std::cout<<"airspeed : "<<control_input.airspeed<<"\n";
				// }

				// Store data for plotting
				// if(state>0){
				// 	x_log.push_back(_x_guide-L_track);
				// 	z_log.push_back(-control_input.posz);//Remove offset when done testing
				// 	state_log.push_back(state);
				// 	hdot_log.push_back(control_input.h_dot);
				// 	hdot_bar_ref_log.push_back(control_input.hdot_bar_ref);
				// 	airspeed_log.push_back(control_input.airspeed-vbar_trim);
				// 	h_ref_log.push_back(href); //Remove offset when done testing
				// 	vbar_ref_log.push_back(vbar_ref);
				// 	hdot_ref_log.push_back(hdot_ref);
				// 	dTc_log.push_back(dT*_max_thrust-T_Trim);
				// 	publish_time_log.push_back(publish_time);
				// 	mp_pose_intra_log.push_back(moving_platform.mp_pose[0]);//Change based on direction of mp motion
				//	mp_pose_alt_log.push_back(-moving_platform.mp_pose[2]);
				//	fw_intra_log.push_back(control_input.posx);//Change based on direction landing
				// }

				// if(Log_data==true){
				// 	myfile.open("/home/mohzap/Documents/PX4_python_logger/Landing_Procedure_Data.txt");
				// 	int sizex =x_log.size();
				// 	int sizez=z_log.size();
				// 	int sizestate=state_log.size();
				// 	int sizehdot=hdot_log.size();
				// 	for(int i=0;i<sizex;i++){
				// 		myfile<<x_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizez;i++){
				// 		myfile<<z_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	myfile<<sizex;
				// 	myfile<<"\n";
				// 	myfile<<sizez;
				// 	myfile<<"\n";
				// 	myfile<<L_track;
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizestate;i++){
				// 		myfile<<state_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<hdot_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<hdot_bar_ref_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<airspeed_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<h_ref_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<vbar_ref_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<hdot_ref_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<dTc_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<publish_time_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<mp_pose_intra_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<mp_pose_alt_log[i]<<",";
				// 	}
				// 	myfile<<"\n";
				// 	for(int i=0;i<sizehdot;i++){
				// 		myfile<<fw_intra_log[i]<<",";
				// 	}
				// 	myfile.close();
				// 	Log_data=false;
				// 	// Clear logging arrays incase they are reused
				// 	x_log.clear();z_log.clear();state_log.clear();hdot_log.clear();hdot_bar_ref_log.clear();airspeed_log.clear();
				// 	h_ref_log.clear();vbar_ref_log.clear();hdot_ref_log.clear();dTc_log.clear();publish_time_log.clear();
				// }




				/*Set variables for publishing*/
				// dA_pub=dA;
				// dE_pub=dE;
				// dR_pub=dR;
				// dT_pub=dT;
				// dF_pub=dF;


				dE_pub=dE;
				dT_pub=dT;
				dF_pub=dF;
				dA_pub=0.0f;
				dR_pub=0.0f;

				if(!fw_custom_control_testing_lateral.lateral_control_disable){ //Enable lateral control
					dA_pub=dA;
					dR_pub=dR;
				}else{
					manual_control_setpoint_s manual_control_setpoint; //Disable lateral control(manual)
					if(manual_control_setpoint_sub.copy(&manual_control_setpoint)){
						dA_pub=manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
						dR_pub=manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					}
				}



				// manual_control_setpoint_s manual_control_setpoint;
				// if (manual_control_setpoint_sub.copy(&manual_control_setpoint))
				// {

				// dA_pub=manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
				// // dE_pub=-manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
				// dR_pub=manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
				// // dT_pub=math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);
				// // dF_pub=0;//!!!ADD MANUAL CONTROL FOR FLAPS!!!
				// // dF_pub = 0.5f * (-manual_control_setpoint.flaps - 1.0f) * _param_fw_flaps_scl.get();
				// }


				publish_roll=control_input.phi;
				publish_pitch=control_input.theta;
				publish_yaw=control_input.psi;


				//print deflection commands
				// disp_count++;
				// if(disp_count%100==0)
				// {
				// 	printf("Controller Deflections\n");
				// 	printf("dA : %.5f ; dE : %.5f ; dR : %.5f ; dT : %.5f\n",(double)dA,(double)dE,(double)dR,(double)dT);

				// }
				// printf("Controllers\n");
				// printf("dA : %.5f ; dE : %.5f ; dR : %.5f ; dT : %.5f\n",(double)dA,(double)dE,(double)dR,(double)dT);
				// printf("\n");

				/*Publish MPC inputs*/
				// std::copy(mpc_ref_in,mpc_ref_in+38,mpc_ins.mpc_ref_in);
				mpc_ins.mpc_ref_in[0]=mpc_ref_in[0];
				mpc_ins.mpc_ref_in[1]=mpc_ref_in[1];
				mpc_ins.mpc_mo_in[0]=mpc_h;
				mpc_ins.mpc_mo_in[1]=mpc_vbar;
				mpc_ins.state=state;
				mpc_ins.grd_vel=sqrt(pow(control_input.velx_I,2)+pow(control_input.vely_I,2));
				if(mpc_ramp_init){
					mpc_ins.ramp_trajectory = (int8_t)1;
				}else{
					mpc_ins.ramp_trajectory = (int8_t)0;
				}
				if(mpc_activate){
					mpc_ins.mpc_activate = (int8_t)1;
				}else{
					mpc_ins.mpc_activate = (int8_t)0;
				}
				if(fw_custom_control_testing_modes.mpc_land_mode){
					mpc_ins.mpc_land = (int8_t)1;
				}else{
					mpc_ins.mpc_land = (int8_t)0;
				}

				// TESTING
				// std::cout<<"PX4 MPC in ref "<<mpc_ins.mpc_ref_in[0]<<" ; "<<mpc_ins.mpc_ref_in[37]<<"\n";
				// std::cout<<"PX4 MPC in mo "<<mpc_ins.mpc_mo_in[0]<<" ; "<<mpc_ins.mpc_mo_in[1]<<"\n";
				// std::cout<<"MPC_in publish time : "<<publish_time<<"\n";

				mpc_in_pub.publish(mpc_ins);

				/*publish state machine state value to topic for gazebo*/
				sm_state.state=state;
				_sm_state_pub.publish(sm_state);

				fw_custom_control_testing_states.timestamp = hrt_absolute_time();
				fw_custom_control_testing_states.yaw = psi_crab_error;
				fw_custom_control_testing_states.y_ct = control_input.y;
				fw_custom_control_testing_states.href = href;
				fw_custom_control_testing_states.vref = vbar_ref;
				fw_custom_control_testing_states.hdotref = hdot_ref;
				fw_custom_control_testing_states.phiref = phi_ref;
				fw_custom_control_testing_states.yref = y_ref;
				fw_custom_control_testing_states.yawref = yaw_ref;
				fw_custom_control_testing_states.phi = control_input.phi;
				fw_custom_control_testing_states.h = -control_input.posz;
				fw_custom_control_testing_states.hdot = control_input.h_dot;
				fw_custom_control_testing_states.v = control_input.airspeed - vbar_trim;
				fw_custom_control_testing_states.da = dA_ctrl;
				fw_custom_control_testing_states.de = dE_ctrl;
				fw_custom_control_testing_states.dr = dR_ctrl;
				fw_custom_control_testing_states.df = dF_ctrl;
				fw_custom_control_testing_states.dt = dT_ctrl;
				fw_custom_control_testing_states.cw_ref = Cw_ref;
				fw_custom_control_testing_states.bw_ref = Bw_ref;
				fw_custom_control_testing_states.p_ref = p_ref;
				fw_custom_control_testing_states.mpc_href = mpc_ref_in[0];
				fw_custom_control_testing_states.mpc_vref = mpc_ref_in[1];
				fw_custom_control_testing_states.sm_state = state;
				fw_custom_control_testing_states.x_it = _x_guide;
				fw_custom_control_testing_states.l_t = L_track;
				fw_custom_control_testing_states.theta = control_input.theta;
				_fw_custom_control_testing_states_pub.publish(fw_custom_control_testing_states);

	}

	}else{
		mcl_disp=false;
		/*In Manual Mode*/
		if(!manual_mode){
			// printf("!!!Manual Mode!!!\n");
			// printf("\n");
			PX4_INFO("!!!Manual Mode!!!");
			manual_count=0;
			//Protection from going from auto to manual then back to auto(reintialise values to default)
			waypoint_END=false;
			state=0;Abort=false;Anti_Abort=false;
			waypoint_capture = false;
			land_init = false;
			//TODO: ADD commander mode and steps reset here
			//Reset Modes
			vehicle_command_s veh_com_res_mode{};
			veh_com_res_mode.command = vehicle_command_s::VEHICLE_CMD_PUB_FW_CUST_SET;
			veh_com_res_mode.target_system = vstatus.system_id;
			veh_com_res_mode.target_component = vstatus.component_id;
			veh_com_res_mode.param1 = 15.0f;
			veh_com_res_mode.param2 = 0.0f;
			vehicle_command_pub.publish(veh_com_res_mode);
			PX4_INFO("Reset FW_Cust Modes");
			//Reset Steps
			vehicle_command_s veh_com_res_step{};
			veh_com_res_step.command = vehicle_command_s::VEHICLE_CMD_PUB_FW_CUST_SET;
			veh_com_res_step.target_system = vstatus.system_id;
			veh_com_res_step.target_component = vstatus.component_id;
			veh_com_res_step.param1 = 7.0f;
			veh_com_res_step.param2 = 0.0f;
			vehicle_command_pub.publish(veh_com_res_step);
			PX4_INFO("Reset FW_Cust Setpoints");
		}
		manual_mode=true;
		controllers_activate=false;
		manual_control_setpoint_s manual_control_setpoint;
		if (manual_control_setpoint_sub.copy(&manual_control_setpoint))
		{
					//orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint);
					//manual_control_setpoint_sub.copy(&manual_control_setpoint);
					//manual_control_setpoint_sub.update(&manual_control_setpoint);
		//Smooth Transition
		if(manual_count<25){//Set smooth transition time to 100ms(4ms*25)
			//dA
			float m_dA=((manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get())-dA_pub)/(25-manual_count);
			float c_dA=(manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get())-m_dA*25;
			dA_pub=m_dA*manual_count+c_dA;
			//dE
			float m_dE=((-manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get())-dE_pub)/(25-manual_count);
			float c_dE=(-manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get())-m_dE*25;
			dE_pub=m_dE*manual_count+c_dE;
			//dR
			float m_dR=((manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get())-dR_pub)/(25-manual_count);
			float c_dR=(manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get())-m_dR*25;
			dR_pub=m_dR*manual_count+c_dR;
			//dT
			float m_dT=((math::constrain(manual_control_setpoint.z, 0.0f, 1.0f))-dR_pub)/(25-manual_count);
			float c_dT=(math::constrain(manual_control_setpoint.z, 0.0f, 1.0f))-m_dR*25;
			dT_pub=m_dT*manual_count+c_dT;
			//dF
			// dF_pub=0;
			dF_pub = 0.5f * (-manual_control_setpoint.flaps - 1.0f) * _param_fw_flaps_scl.get();
			manual_count++;
		}else{
		dA_pub=manual_control_setpoint.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
		dE_pub=-manual_control_setpoint.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
		dR_pub=manual_control_setpoint.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
		dT_pub=math::constrain(manual_control_setpoint.z, 0.0f, 1.0f);
		dF_pub=0;//!!!ADD MANUAL CONTROL FOR FLAPS!!!
		// dF_pub = 0.5f * (-manual_control_setpoint.flaps - 1.0f) * _param_fw_flaps_scl.get();

		}

		}

	}

		/*Testing Control Surface Deflection*/
		// dA_pub=-25.0/180.0*M_PI;
		// dE_pub=10.0/180.0*M_PI;
		// dT_pub=0.0;
		// dR_pub=-20.0/180.0*M_PI;
		// dF_pub=-22.5/180.0*M_PI;

		// Compensating for controller direction change
		// dA_pub=-dA_pub * (float) 2.223525;
		// dE_pub=-dE_pub * (float)1.8193;//Scaller is for controller only
		// dT_pub=dT_pub;
		// dR_pub=-dR_pub * (float)1.60315;
		// dF_pub=-dF_pub * (float) 2.5331025;


		/*Assign actuators*/
		actuators.timestamp= hrt_absolute_time();
		actuators.control[0]=dA_pub;actuators.control[1]=dE_pub;actuators.control[2]=dR_pub;actuators.control[3]=dT_pub;actuators.control[4]=dF_pub;

		/*set setpoint rates*/
		//Check!!!
		// rates_sp.roll=publish_roll;
		// rates_sp.pitch=publish_pitch;
		// rates_sp.yaw=publish_yaw;

		/* publish rates */
		//orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);
		// _rate_sp_pub.publish(rates_sp);


		/* sanity check and publish actuator outputs */
		if (PX4_ISFINITE(actuators.control[0]) &&
			PX4_ISFINITE(actuators.control[1]) &&
			PX4_ISFINITE(actuators.control[2]) &&
			PX4_ISFINITE(actuators.control[3]) &&
			PX4_ISFINITE(actuators.control[4])) {
			//orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
			_actuators_0_pub.publish(actuators);
		}
	}
	perf_end(_loop_perf);
}



/* Startup Functions */

int Controllers::task_spawn(int argc, char *argv[])
{
	// bool vtol = false;

	// if (argc > 1) {
	// 	if (strcmp(argv[1], "vtol") == 0) {
	// 		vtol = true;
	// 	}
	// }

	Controllers *instance = new Controllers();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Controllers::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Controllers::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
extern "C" __EXPORT int ex_fixedwing_actual_custom_control_mod_main(int argc, char *argv[])
{
	// printf("Running custom fixed wing controller\n");
	return Controllers::main(argc, argv);
}

/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ecl_roll_controller.h
 * Definition of a simple orthogonal roll PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <drivers/drv_hrt.h>
#include "ecl_controller.h"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Subscription.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
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
#include "params.h"
#include <lib/ecl/geo/geo.h>
#include <uORB/uORB.h>
#include <float.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_accel.h>
#include "BlockHighPass.cpp"
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/moving_platform.h>
#include <uORB/topics/fw_controllers_sm.h>
#include <uORB/topics/mpc_inputs.h>
#include <uORB/topics/mpc_outputs.h>
// #include <uORB/topics/wind_estimate.h>
#include <uORB/topics/commander_state.h>

//Graph plotting
#include <iostream>
#include <fstream>
#include "cmath"
#include <vector>
using namespace std;

using uORB::SubscriptionData;

using namespace time_literals;

class Controllers:  public ModuleBase<Controllers>,public ModuleParams,public ECL_Controller, public px4::WorkItem{ //Figure out how to import BlockHighPass class
public:
	// Controllers() = default;
	// ~Controllers() = default;
	Controllers();
	~Controllers() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	//float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	//float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) override;
	//float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;

	void init_ECL_variables();
	// float pitch_rate_damper_controller(const Control_Data &state_data,float dE);
	// void airspeed_climb_rate_controller(const Control_Data &state_data,float vbar_ref,float hdot_ref,const float dt,float *defl);
	// float altitude_controller(const Control_Data &state_data,float h_ref);
	void initialise_NSADLC_HPF();
	// float dutch_roll_damper_controller(const Control_Data &state_data,float dt);
	// float roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt);
	float heading_controller(const Control_Data &state_data,float psi_ref);
	void guide_axis_alg(float S[2],float D[2],const Control_Data &state_data,float out[4]);
	void waypoint_scheduler(float Destinations[][2],int Destination_size,const Control_Data &state_data,float out[2][2]);
	void navigation_controller(float D[][2],int Destination_size,const Control_Data &state_data,float out[4]);
	float guidance_controller_2(float guide_val[4]);
	float transition_multiplexer(float phi_ref_HG,float phi_ref_CT, float yerr);

	//Actual Controllers
	float airspeed_controller(const Control_Data &state_data,float vbar_ref,const float dt);
	void NSADLC_controller(const Control_Data &state_data,float Cw_ref,const float dt,float *defl);
	float climb_rate_controller(const Control_Data &state_data,float hdot_ref,const float dt);
	float altitude_controller(const Control_Data &state_data,float h_ref,const float dt);
	float LSA(const Control_Data &state_data,float Bw_ref,const float dt);
	float roll_rate_controller(const Control_Data &state_data,float p_ref,const float dt);
	float roll_angle_controller(const Control_Data &state_data,float phi_ref,const float dt);
	float guidance_controller_1(const Control_Data &state_data,float phibar_ref,float y_ref,const float dt);
	float yaw_controller(const Control_Data &state_data,float psi_ref,const float dt);

	void reset_integrators();
	void vehicle_control_mode_poll();
	void initialise_integrators(const Control_Data &state_data);
	void state_machine(Control_Data &state_data,float ref_out[4],float mp_pos[3]);
	void landing_point(Control_Data &state_data,float mp_pose[3],float mp_vel[3]);
	float altitude_limit_intergrator(const Control_Data &state_data,float h_ref,const float dt);
	void mpc_referance_generator(float h_ref,float v_ref,float mpc_ref[]);

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude)};//EKF Output
	// uORB::SubscriptionCallbackWorkItem _att_sub{this, ORB_ID(vehicle_attitude_groundtruth)};//Simulator output
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};//EKF Output
	// uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position_groundtruth)};//Simulator output
	uORB::SubscriptionCallbackWorkItem airspeed_sub{this, ORB_ID(airspeed_validated)};


	uORB::SubscriptionInterval parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	//uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription vstatus_sub{ORB_ID(vehicle_status)};
	uORB::Subscription global_sp_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription estim_sub{ORB_ID(estimator_status)};
	uORB::Subscription ang_vel_sub{ORB_ID(vehicle_angular_velocity)};//EKF Output
	// uORB::Subscription ang_vel_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};//Simulator output
	uORB::Subscription accel_sub{ORB_ID(vehicle_acceleration)};
	uORB::Subscription _vcontrol_mode_sub{ORB_ID(vehicle_control_mode)};
	//uORB::Subscription airspeed_sub{ORB_ID(airspeed_validated)};
	//uORB::SubscriptionData<airspeed_validated_s> _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	//uORB::Subscription airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription airspeed_sens_sub{ORB_ID(airspeed)};
	uORB::Subscription _mov_plat_sub{ORB_ID(moving_platform)};
	uORB::Subscription mpc_out_sub{ORB_ID(mpc_outputs)};
	// uORB::Subscription wind_sub{ORB_ID(wind_estimate)};
	uORB::Subscription commander_state_sub{ORB_ID(commander_state)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub;
	//uORB::Publication<vehicle_attitude_setpoint_s>	_attitude_sp_pub;
	//uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s>	_rate_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<fw_controllers_sm_s>	_sm_state_pub{ORB_ID(fw_controllers_sm)};
	uORB::Publication<mpc_inputs_s>	mpc_in_pub{ORB_ID(mpc_inputs)};

	actuator_controls_s			actuators {};
	vehicle_rates_setpoint_s		rates_sp {};
	vehicle_local_position_s	_local_pos {};
	vehicle_control_mode_s			_vcontrol_mode {};	/**< vehicle control mode */
	fw_controllers_sm_s			sm_state{};
	mpc_inputs_s				mpc_ins{};

	perf_counter_t	_loop_perf;

	//Control Data memory
	float y_old=0,ydot_old=0;

	int disp_count=0;
	int disp_count_ctrl=0;
	bool controllers_activate=false;
	float dA_pub=0,dE_pub=0,dR_pub=0,dT_pub=0,dF_pub=0;
	float publish_roll=0,publish_pitch=0,publish_yaw=0;
	bool manual_mode=false;
	int manual_count=0;
	//Anti-windup
	float dT_AS=0,dT_AS_lim=0,dR_LSA=0,dR_LSA_lim=0,dA_RR=0,dA_RR_lim=0,dF_DLC=0,dF_DLC_lim=0,dE_NSA=0,dE_NSA_lim=0;
	float hdot_ref_int=0,hdot_ref_int_lim=0;
	//State Machine
	bool waypoint_END=false,END=false,Land=false,Abort=false,Anti_Abort=false,yaw_ctrl=false,Log_data=false;
	float Tau_yaw=0,gamma=0,gamma_land=0,h_ref_SM=0,dg=0,dis_t=0;
	int state=0;
	//Graph Plotting
	ofstream myfile;
	std::vector<float> x_log,z_log,hdot_log,hdot_bar_ref_log,airspeed_log,h_ref_log,vbar_ref_log,hdot_ref_log,dTc_log,publish_time_log,mp_pose_intra_log,mp_pose_alt_log,fw_intra_log;
	std::vector<int> state_log;

	// Moving platform
	std::vector<float> TD_position={0.00,0.00,0.00};//Touch Down point

	// MPC
	int MPC_P=30;//choose
	float MPC_Ts=0.12;//choose
	float MPC_num_inputs=2;
	float mpc_ref_in[2]={0,0};
	float mpc_h=0;
	float mpc_vbar=0;
	bool mpc_activate=false,clas_alt_activate=false;
	uint64_t prev_mpc_pub_time=0;
	float CR_start_test_time=0;
	bool test_log=false,test_write=false,end_log=false;

	//Flight mode control
	bool mcl_disp=false; //Manual control lost display


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FW_MAN_P_MAX>) _param_fw_man_p_max,
		(ParamFloat<px4::params::FW_MAN_P_SC>) _param_fw_man_p_sc,
		(ParamFloat<px4::params::FW_MAN_R_MAX>) _param_fw_man_r_max,
		(ParamFloat<px4::params::FW_MAN_R_SC>) _param_fw_man_r_sc,
		(ParamFloat<px4::params::FW_MAN_Y_SC>) _param_fw_man_y_sc,
		(ParamFloat<px4::params::TRIM_PITCH>) _param_trim_pitch,
		(ParamFloat<px4::params::TRIM_ROLL>) _param_trim_roll,
		(ParamFloat<px4::params::TRIM_YAW>) _param_trim_yaw
	)



};

#endif // ECL_ROLL_CONTROLLER_H

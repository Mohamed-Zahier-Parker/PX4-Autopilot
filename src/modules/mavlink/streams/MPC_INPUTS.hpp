
#ifndef MPC_INPUTS_HPP
#define MPC_INPUTS_HPP

#include <uORB/topics/mpc_inputs.h>
// #include <iostream>

class MavlinkStreamMPCInputs : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "MPC_INPUTS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MPC_INPUTS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamMPCInputs(mavlink);
	}

	unsigned get_size() override
	{
		return _mpc_in_sub.advertised() ? MAVLINK_MSG_ID_MPC_INPUTS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:

	explicit MavlinkStreamMPCInputs(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	/* do not allow top copying this class */
	// MavlinkStreamMPCInputs(MavlinkStreamMPCInputs &) = delete;
	// MavlinkStreamMPCInputs &operator = (const MavlinkStreamMPCInputs &) = delete;


	uORB::Subscription _mpc_in_sub{ORB_ID(mpc_inputs)};

	bool send() override
	{
		mavlink_mpc_inputs_t msg{};

		// for(int i=0;i<38;i++){
		// 	msg.mpc_ref_in[i] = NAN;
		// }
		msg.mpc_ref_in[0] = NAN;
		msg.mpc_ref_in[1] = NAN;
		msg.mpc_mo_in[0] = NAN;
		msg.mpc_mo_in[1] = NAN;

		bool mpc_inputs_updated = false;
		mpc_inputs_s mpc_inputs_data{};

		if (_mpc_in_sub.update(&mpc_inputs_data)) {
			// for(int i=0;i<38;i++){
			// 	msg.mpc_ref_in[i] = mpc_inputs_data.mpc_ref_in[i];
			// }
			msg.mpc_ref_in[0] = mpc_inputs_data.mpc_ref_in[0];
			msg.mpc_ref_in[1] = mpc_inputs_data.mpc_ref_in[1];
			msg.mpc_mo_in[0] = mpc_inputs_data.mpc_mo_in[0];
			msg.mpc_mo_in[1] = mpc_inputs_data.mpc_mo_in[1];
			msg.state = mpc_inputs_data.state;
			mpc_inputs_updated = true;
		}

		if (mpc_inputs_updated) {
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_mpc_inputs_send_struct(_mavlink->get_channel(), &msg);
			// std::cout<<"PX4 MPC_inputs publish: "<<msg.mpc_ref_in[0]<<" ; "<<msg.mpc_mo_in[0]<<"\n";
			// std::cout<<"MPC_in Time : "<<msg.time_usec<<"\n";
			return true;
		}

		return false;
	}
};

#endif // MPC_INPUTS_HPP

#include <cstring>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <can_plugins/Frame.h>
#include <adhoc_canplugins_onehalf/CanFrame.h>

#include <CRSLib/include/std_type.hpp>
#include <rosparam_util.hpp>

#include <XmlRpcException.h>

using namespace CRSLib::RosparamUtil;

namespace crs_lib
{
	class AdhocCanPluginsBridgeNode final : public nodelet::Nodelet
	{
		struct CanPlugins2Pub
		{
			u32 id;
			ros::Publisher pub;
		};

		struct CanPlugins2Callback
		{
			AdhocCanPluginsBridgeNode& self;
			u32 id;

			void operator()(const adhoc_canplugins_onehalf::CanFrame::ConstPtr& can_frame)
			{
				can_plugins::Frame frame{};
				frame.id = id;
				frame.dlc = can_frame->dlc;
				std::memcpy(frame.data.data(), can_frame->data.data(), can_frame->dlc);
				self.to_canplugins_pub.publish(frame);
			}
		};

		std::vector<CanPlugins2Pub> to_ros_pubs{};
		std::vector<ros::Subscriber> from_ros_subs{};
		ros::Publisher to_canplugins_pub{};
		ros::Subscriber from_canplugins_sub{};

		void onInit() override
		{
			auto nh = Nodelet::getMTNodeHandle();

			auto ros_ids = CRSLib::RosparamUtil::get_param(nh, "ros_ids");
			if(!ros_ids) return;
			for(int i = 0; ; ++i)
			{

				// エラーをデフォルトで出すのはよろしくなかろう。
				auto ros_id = CRSLib::RosparamUtil::get_param(ros_ids, i, true);
				auto id_ = CRSLib::RosparamUtil::nullable_cast<int>(ros_id);
				if(!id_) break;
				auto id = static_cast<u32>(*id_);
				to_ros_pubs.emplace_back(CanPlugins2Pub{id, nh.advertise<adhoc_canplugins_onehalf::CanFrame>("can" + std::to_string(id), 10)});
				from_ros_subs.emplace_back(nh.subscribe<adhoc_canplugins_onehalf::CanFrame>("can" + std::to_string(id), 10, CanPlugins2Callback{*this, id}));

				int tx_buffer = xml_rpc_cast<int>(CRSLib::RosparamUtil::get_param(nh, "tx_buffer"));
				int rx_buffer = xml_rpc_cast<int>(CRSLib::RosparamUtil::get_param(nh, "rx_buffer"));

				to_canplugins_pub = nh.advertise<can_plugins::Frame>("can_tx", tx_buffer);
				from_canplugins_sub = nh.subscribe<can_plugins::Frame>("can_rx", rx_buffer, &AdhocCanPluginsBridgeNode::can_plugins_callback, this);
			}
		}

		void can_plugins_callback(const can_plugins::Frame::ConstPtr& frame)
		{
			for(auto& to_ros_pub : to_ros_pubs)
			{
				if(to_ros_pub.id == frame->id)
				{
					adhoc_canplugins_onehalf::CanFrame can_frame{};
					std::memcpy(can_frame.data.data(), frame->data.data(), frame->dlc);
					to_ros_pub.pub.publish(can_frame);
					break;
				}
			}
		}
	};
}

PLUGINLIB_EXPORT_CLASS(crs_lib::AdhocCanPluginsBridgeNode, nodelet::Nodelet);
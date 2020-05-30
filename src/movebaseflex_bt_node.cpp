#include <behaviortree_ros/bt_action_node.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/RecoveryAction.h>

using namespace BT;
//Mainly copied from test_bt.cpp

//-------------------------------------------------------------
// Simple Action to print a number (PURE BT implementation)
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    geometry_msgs::PoseStamped value;
    if( getInput("message", value ) ){
      std::cout << "PrintValue: " << value << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return{ BT::InputPort<geometry_msgs::PoseStamped>("message") };
  }
};

class WaitForGoal : public BT::SyncActionNode
{
public:
  WaitForGoal(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name,config) {}

  BT::NodeStatus tick() override{
    geometry_msgs::PoseStamped goalpose;
    geometry_msgs::PoseStampedConstPtr msg = 
      ros::topic::waitForMessage<geometry_msgs::PoseStamped>(goal_topic_,ros::Duration(10));
      if (msg == NULL){
        ROS_INFO("No Goal PoseStamped Recieved");
        return NodeStatus::FAILURE;
      }
      else {
        ROS_INFO("SUCCESS");
        setOutput<geometry_msgs::PoseStamped>("goal",*msg); //TODO: Pass around ConstPtr 
        return NodeStatus::SUCCESS; 
      }
  }
  static BT::PortsList providedPorts(){
    return {BT::OutputPort<geometry_msgs::PoseStamped>("goal")};
  }
private:
  const std::string goal_topic_ = "/move_base_simple/goal";
};

//----------------------------------------------------------
  // Simple tree, used to execute once each action.
  static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
            <RetryUntilSuccesful num_attempts="4">
                <Timeout msec="10000">
                    <WaitForGoal goal="{goal_pose}" />
                </Timeout>
            </RetryUntilSuccesful>
            <PrintValue message="{goal_pose}"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "movebaseflex_bt_node");
  ros::NodeHandle nh;

  BehaviorTreeFactory factory;

  factory.registerNodeType<WaitForGoal>("WaitForGoal");
  factory.registerNodeType<PrintValue>("PrintValue");

  auto tree = factory.createTreeFromText(xml_text);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}


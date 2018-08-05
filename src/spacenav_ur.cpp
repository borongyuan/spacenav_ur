#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

class InverseDifferentialKinematics
{
public:
    InverseDifferentialKinematics(const ros::NodeHandle& nh, const KDL::Chain& chain):
    nh_(nh),
    iksolver(chain, 0.1, 30)
    {
        iksolver.setLambda(0.1);

        joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        if (joint_names.size() != chain.getNrOfJoints())
            throw std::runtime_error("joint_names vector and kdl_chain have unequal lengths");
        joint_pos.resize(chain.getNrOfJoints());
        joint_vel.resize(chain.getNrOfJoints());
        tj_msg.joint_names = joint_names;

        joint_sub = nh_.subscribe("joint_states", 10, &InverseDifferentialKinematics::jointCallback,this);
        twist_sub = nh_.subscribe("twist", 10, &InverseDifferentialKinematics::twistCallback,this);
        vel_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_speed", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub;
    ros::Subscriber twist_sub;
    ros::Publisher vel_pub;

    KDL::ChainIkSolverVel_wdls iksolver;
    std::vector<std::string> joint_names;
    KDL::JntArray joint_pos, joint_vel;
    KDL::Twist cart_vel;
    std::map<std::string, double> joint_state_map;
    trajectory_msgs::JointTrajectoryPoint tj_point;
    trajectory_msgs::JointTrajectory tj_msg;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& joint);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);
};

void InverseDifferentialKinematics::jointCallback(const sensor_msgs::JointState::ConstPtr& joint)
{
    for (size_t i = 0; i < joint->name.size(); i++)
        joint_state_map[joint->name[i]] = joint->position[i]; 
    for (size_t i = 0; i < joint_pos.rows(); i++)
        joint_pos(i) = joint_state_map[joint_names[i]];
}

void InverseDifferentialKinematics::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    tf::twistMsgToKDL(*twist, cart_vel);
    if (iksolver.CartToJnt(joint_pos, cart_vel, joint_vel) == iksolver.E_SVD_FAILED)
        throw std::runtime_error("svd solution failed, error code: " + std::to_string(iksolver.getSVDResult()));
    tj_point.velocities = {joint_vel(0), joint_vel(1), joint_vel(2), joint_vel(3), joint_vel(4), joint_vel(5)};
    tj_msg.points = {tj_point};
    vel_pub.publish(tj_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "spacenav_ur");
    ros::NodeHandle nh("~");

    try
    {
        KDL::Tree ur_tree;
        std::string robot_desc_string;
        nh.param("/robot_description", robot_desc_string, std::string());
        if (!kdl_parser::treeFromString(robot_desc_string, ur_tree))
            throw std::runtime_error("Failed to construct KDL tree from parameter server");

        KDL::Chain ur_chain;
        if (!ur_tree.getChain("world", "ee_link", ur_chain))
            throw std::runtime_error("Failed to get KDL chain from KDL tree");

        InverseDifferentialKinematics ur_world_to_ee(nh, ur_chain);
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }
    return 0;
}
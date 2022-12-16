#ifndef __RANDOM_CMD_VEL_H__
#define __RANDOM_CMD_VEL_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


class RandomCmdVel
{
    public:
        RandomCmdVel();
        void generate_cmd_vel(void);
        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg); 

    class State{
        public:
            State();

            double yaw;
            double velocity;
            double angular_velocity;
        private:
    };
    typedef std::vector<State> trajectory;
    typedef std::vector<trajectory> trajectories;

    class Window{
        public:
            Window();
            double min_velocity;
            double max_velocity;
            double min_angular_velocity;
            double max_angular_velocity;
        private:
    };

    void odom_callback(const nav_msgs::OdometryConstPtr& msg);


    Window calc_dynamic_window(const geometry_msgs::Twist&);
    trajectories calc_trajectories(const Window&);
    geometry_msgs::Twist choice_trajectory(const trajectories&, const int&);

    void process(void);
    void visualize_trajectories(const trajectories&, const int&, const ros::Publisher&);
    int generate_random_number(const trajectories&);

    private:
        double HZ;
        double CHANGE_RATE;

        double MAX_VELOCITY;
        double MIN_VELOCITY;
        double MAX_YAWRATE;

        double MAX_ACCELERATION;
        double MAX_D_YAWRATE;

        double VELOCITY_RESOLUTION;
        double YAWRATE_RESOLUTION;

        double PREDICT_TIME;

        bool is_odom_received;
        bool IS_VISUALIZE;

        std::string ROBOT_FRAME; 


        geometry_msgs::Twist current_velocity;

        ros::NodeHandle nh;

        ros::Subscriber odom_sub;

        ros::Publisher cmd_vel_pub;
        ros::Publisher trajectories_pub;

};

#endif // __RANDOM_CMD_VEL_H__

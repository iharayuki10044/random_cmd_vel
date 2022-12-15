#ifndef __RANDOM_CMD_VEL_H__
#define __RANDOM_CMD_VEL_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

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
    typedef std::vector<trajectory> trajectorys;

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
    trajectorys calc_trajectorys(const Window&);


    private:

        double Hz;
        double CHANGE_RATE;

        double MAX_VELOCITY;
        double MIN_VELOCITY;
        double MAX_YAWRATE;

        double MAX_ACCELERATION;
        double MAX_D_YAWRATE;

        geometry_msgs::Twist current_velocity;

        ros::NodeHandle nh;

        ros::Subscriber odom_sub;

        ros::Publisher cmd_vel_pub;


};

#endif // __RANDOM_CMD_VEL_H__

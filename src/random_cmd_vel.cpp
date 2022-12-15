#include "random_cmd_vel/random_cmd_vel.h"

RandomCmdVel::RandomCmdVel()
    :nh("~")
{
    odom_sub = nh.subscribe("/odom", 1, &RandomCmdVel::odom_callback, this);    
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void RandomCmdVel::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
}

RandomCmdVel::State::State()
{
    yaw = 0.0;
    velocity = 0.0;
    angular_velocity = 0.0;
}

RandomCmdVel::Window::Window()
{
    min_velocity = 0.0;
    max_velocity = 0.0;
    min_angular_velocity = 0.0;
    max_angular_velocity = 0.0;
}

RandomCmdVel::Window RandomCmdVel::calc_dynamic_window(const geometry_msgs::Twist& current_velocity)
{
    Window window;
    window.min_velocity = current_velocity.linear.x - MAX_ACCELERATION;
    window.max_velocity = current_velocity.linear.x + MAX_ACCELERATION;
    window.min_angular_velocity = current_velocity.angular.z - MAX_D_YAWRATE;
    window.max_angular_velocity = current_velocity.angular.z + MAX_D_YAWRATE;

    if(window.min_velocity < MIN_VELOCITY)
        window.min_velocity = MIN_VELOCITY;
    if(window.max_velocity > MAX_VELOCITY)
        window.max_velocity = MAX_VELOCITY;
    if(window.min_angular_velocity < -MAX_YAWRATE)
        window.min_angular_velocity = -MAX_YAWRATE;
    if(window.max_angular_velocity > MAX_YAWRATE)
        window.max_angular_velocity = MAX_YAWRATE;

    return window;
}

RandomCmdVel::trajectorys RandomCmdVel::calc_trajectorys(const Window& window)
{
    trajectorys trajectorys;
    for(float v = window.min_velocity; v <= window.max_velocity; v += CHANGE_RATE)
    {
        for(float w = window.min_angular_velocity; w <= window.max_angular_velocity; w += CHANGE_RATE)
        {
            trajectory trajectory;
            for(float t = 0.0; t <= 1.0; t += 1.0/Hz)
            {
                State state;
                state.velocity = v;
                state.angular_velocity = w;
                state.yaw += state.angular_velocity * t;
                trajectory.push_back(state);
            }
            trajectorys.push_back(trajectory);
        }
    }

    return trajectorys;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "random_cmd_vel");
    RandomCmdVel random_cmd_vel;
    ros::spin();
    return 0;
}
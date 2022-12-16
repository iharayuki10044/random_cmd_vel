#include "random_cmd_vel/random_cmd_vel.h"

RandomCmdVel::RandomCmdVel()
    :nh("~")
{
    nh.param("HZ", HZ, 10.0);
    nh.param("CHANGE_RATE", CHANGE_RATE, 0.1);
    nh.param("MAX_VELOCITY", MAX_VELOCITY, 0.5);
    nh.param("MIN_VELOCITY", MIN_VELOCITY, -0.5);
    nh.param("MAX_YAWRATE", MAX_YAWRATE, 1.0);
    nh.param("MAX_ACCELERATION", MAX_ACCELERATION, 0.5);
    nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, 1.0);
    nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, 0.1);
    nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, 0.1);

    ROS_INFO_STREAM("HZ: " << HZ);
    ROS_INFO_STREAM("CHANGE_RATE: " << CHANGE_RATE);
    ROS_INFO_STREAM("MAX_VELOCITY: " << MAX_VELOCITY);
    ROS_INFO_STREAM("MIN_VELOCITY: " << MIN_VELOCITY);
    ROS_INFO_STREAM("MAX_YAWRATE: " << MAX_YAWRATE);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
    ROS_INFO_STREAM("MAX_D_YAWRATE: " << MAX_D_YAWRATE);
    ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
    ROS_INFO_STREAM("YAWRATE_RESOLUTION: " << YAWRATE_RESOLUTION);

    odom_sub = nh.subscribe("/odom", 1, &RandomCmdVel::odom_callback, this);    
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

void RandomCmdVel::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    is_odom_received = true;
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
    window.min_velocity = std::min(current_velocity.linear.x - MAX_ACCELERATION, MIN_VELOCITY);
    window.max_velocity = std::max(current_velocity.linear.x + MAX_ACCELERATION, MAX_VELOCITY);
    window.min_angular_velocity = std::min(current_velocity.angular.z - MAX_D_YAWRATE, -MAX_YAWRATE);
    window.max_angular_velocity = std::max(current_velocity.angular.z + MAX_D_YAWRATE, MAX_YAWRATE);

    return window;
}

RandomCmdVel::trajectorys RandomCmdVel::calc_trajectorys(const Window& window)
{
    trajectorys trajectorys;
    for(float v = window.min_velocity; v <= window.max_velocity; v += VELOCITY_RESOLUTION){
        for(float w = window.min_angular_velocity; w <= window.max_angular_velocity; w += YAWRATE_RESOLUTION){
            trajectory trajectory;
            for(float t = 0.0; t <= 1.0; t += 1.0/HZ){
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

geometry_msgs::Twist RandomCmdVel::choice_trajectry(const trajectorys& trajectorys)
{
    geometry_msgs::Twist cmd_vel;

    int max = trajectorys.size();
    int id = rand() %max;

    cmd_vel.linear.x = trajectorys[id][0].velocity;
    cmd_vel.angular.z = trajectorys[id][0].angular_velocity;
    return cmd_vel;
}

void RandomCmdVel::process(void)
{
    ros::Rate rate(HZ);
    while(ros::ok())
    {
        if(is_odom_received){
            Window window;
            trajectorys trajectorys;
            window = calc_dynamic_window(current_velocity);
            trajectorys = calc_trajectorys(now_window);
            geometry_msgs::Twist cmd_vel = choice_trajectry(trajectorys);
            cmd_vel_pub.publish(cmd_vel);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "random_cmd_vel");
    RandomCmdVel random_cmd_vel;
    random_cmd_vel.process();
    return 0;
}
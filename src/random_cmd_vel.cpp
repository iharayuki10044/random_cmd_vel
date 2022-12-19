#include "random_cmd_vel/random_cmd_vel.h"

RandomCmdVel::RandomCmdVel()
    :nh("~")
{
    nh.param("HZ", HZ, 10.0);
    nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    nh.param("CHANGE_RATE", CHANGE_RATE, 0.1);
    nh.param("MAX_VELOCITY", MAX_VELOCITY, 0.5);
    nh.param("MIN_VELOCITY", MIN_VELOCITY, -0.5);
    nh.param("MAX_YAWRATE", MAX_YAWRATE, 1.0);
    nh.param("MAX_ACCELERATION", MAX_ACCELERATION, 0.5);
    nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, 1.0);
    nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, 0.1);
    nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, 0.1);
    nh.param("PREDICT_TIME", PREDICT_TIME, 3.0);
    nh.param("IS_VISUALIZE", IS_VISUALIZE, false);

    ROS_INFO_STREAM("HZ: " << HZ);
    ROS_INFO_STREAM("ROBOT_FRAME: " << ROBOT_FRAME);
    ROS_INFO_STREAM("CHANGE_RATE: " << CHANGE_RATE);
    ROS_INFO_STREAM("MAX_VELOCITY: " << MAX_VELOCITY);
    ROS_INFO_STREAM("MIN_VELOCITY: " << MIN_VELOCITY);
    ROS_INFO_STREAM("MAX_YAWRATE: " << MAX_YAWRATE);
    ROS_INFO_STREAM("MAX_ACCELERATION: " << MAX_ACCELERATION);
    ROS_INFO_STREAM("MAX_D_YAWRATE: " << MAX_D_YAWRATE);
    ROS_INFO_STREAM("VELOCITY_RESOLUTION: " << VELOCITY_RESOLUTION);
    ROS_INFO_STREAM("YAWRATE_RESOLUTION: " << YAWRATE_RESOLUTION);
    ROS_INFO_STREAM("PREDICT_TIME: " << PREDICT_TIME);
    ROS_INFO_STREAM("IS_VISUALIZE: " << IS_VISUALIZE);

    odom_sub = nh.subscribe("/odom", 1, &RandomCmdVel::odom_callback, this);
    trajectories_pub = nh.advertise<visualization_msgs::MarkerArray>("/random_trajectories", 1);
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
    window.min_velocity = std::max(current_velocity.linear.x - MAX_ACCELERATION, MIN_VELOCITY);
    window.max_velocity = std::min(current_velocity.linear.x + MAX_ACCELERATION, MAX_VELOCITY);
    window.min_angular_velocity = std::max(current_velocity.angular.z - MAX_D_YAWRATE, -MAX_YAWRATE);
    window.max_angular_velocity = std::min(current_velocity.angular.z + MAX_D_YAWRATE, MAX_YAWRATE);

    return window;
}

RandomCmdVel::trajectories RandomCmdVel::calc_trajectories(const Window& window)
{
    // std::cout << "==calc_trajectories==" << std::endl;
    // std::cout << "window.min_velocity: " << window.min_velocity << std::endl;
    // std::cout << "window.max_velocity: " << window.max_velocity << std::endl;
    // std::cout << "window.min_angular_velocity: " << window.min_angular_velocity << std::endl;
    // std::cout << "window.max_angular_velocity: " << window.max_angular_velocity << std::endl;

    trajectories trajectories;
    int counter = 0;
    for(float v = window.min_velocity; v <= window.max_velocity; v += VELOCITY_RESOLUTION){
        // std::cout << "v: " << v << std::endl;
        // std::cout << "counter: " << counter++ << std::endl; 
        for(float w = window.min_angular_velocity; w <= window.max_angular_velocity; w += YAWRATE_RESOLUTION){
            trajectory trajectory;
            for(float t = 0.0; t <= PREDICT_TIME; t += 1.0/HZ){
                State state;
                state.velocity = v;
                state.angular_velocity = w;
                state.yaw += state.angular_velocity * t;
                trajectory.push_back(state);
            }
            trajectories.push_back(trajectory);
        }
        counter++;
    }
    return trajectories;
}

std::vector<geometry_msgs::Twist> RandomCmdVel::choice_trajectory(const trajectories& trajectories, const int& id)
{
    std::vector<geometry_msgs::Twist> cmd_vel;

    for(int i = 0; i < trajectories[id].size(); i++){
        geometry_msgs::Twist twist;
        twist.linear.x = trajectories[id][i].velocity;
        twist.angular.z = trajectories[id][i].angular_velocity;
        cmd_vel.push_back(twist);
    }
    return cmd_vel;
}

int RandomCmdVel::generate_random_number(const trajectories& trajectories) 
{
    int max = trajectories.size();
    int id = rand() %max;
    return id;
}

void RandomCmdVel::visualize_trajectories(const trajectories& trajectories, const int& id, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray marker_array;
    for(int i = 0; i < trajectories.size(); i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = ROBOT_FRAME;
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectories";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        geometry_msgs::Point point;
        point.x = 0.0;
        point.y = 0.0;

        for(int j = 0; j < trajectories[i].size(); j++){
            if(i != id){
                point.x += trajectories[i][j].velocity /HZ * cos(trajectories[i][j].yaw);
                point.y += trajectories[i][j].velocity /HZ * sin(trajectories[i][j].yaw);
                marker.points.push_back(point);
            }
            else{
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                point.x += trajectories[i][j].velocity /HZ * cos(trajectories[i][j].yaw);
                point.y += trajectories[i][j].velocity /HZ * sin(trajectories[i][j].yaw);
                marker.points.push_back(point);
            }
        }
        marker_array.markers.push_back(marker);
    }
    pub.publish(marker_array);

}


void RandomCmdVel::process(void)
{
    ros::Rate rate(HZ);
    while(ros::ok())
    {
        if(is_odom_received){
            Window window;
            trajectories trajectories;
            window = calc_dynamic_window(current_velocity);
            trajectories = calc_trajectories(window);
            int id = generate_random_number(trajectories);

            geometry_msgs::Twist cmd_vel;

            std::cout << "id: " << id << std::endl;
            std::cout << "trajectory size : " << trajectories[id].size() << std::endl;

            for(int i=0; i<trajectories[id].size(); i++){
                cmd_vel.linear.x = trajectories[id][i].velocity;
                cmd_vel.angular.z = trajectories[id][i].angular_velocity;
                cmd_vel_pub.publish(cmd_vel);
                if(IS_VISUALIZE){
                    visualize_trajectories(trajectories, id, trajectories_pub);
                }
                rate.sleep();
            }
            // std::cout << "current_velocity.linear.x: " << current_velocity.linear.x << std::endl;
            // std::cout << "current_velocity.angular.z: " << current_velocity.angular.z << std::endl;

            // std::cout << "window.min_velocity: " << window.min_velocity << std::endl;
            // std::cout << "window.max_velocity: " << window.max_velocity << std::endl;
            // std::cout << "window.min_angular_velocity: " << window.min_angular_velocity << std::endl;
            // std::cout << "window.max_angular_velocity: " << window.max_angular_velocity << std::endl;

            // std::cout << id << " / " << trajectories.size() << std::endl;
            // std::cout << "cmd_vel.linear.x: " << cmd_vel.linear.x << std::endl;
            // std::cout << "cmd_vel.angular.z: " << cmd_vel.angular.z << std::endl;
            

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
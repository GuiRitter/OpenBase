#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <open_base/Velocity.h>

long double duration;

int i;

open_base::Velocity message;

ros::Publisher publisher;

long double theta_left_current;
long double theta_back_current;
long double theta_right_current;
long double theta_left_previous;
long double theta_back_previous;
long double theta_right_previous;

ros::Time timeCurrent;
ros::Time timePrevious;

long double v_left;
long double v_back;
long double v_right;

void onJointStateMessage(const sensor_msgs::JointState::ConstPtr& input){
    timeCurrent = ros::Time::now();
    for (i = 0; i < input->name.size(); i++) {
        if (input->name[i].c_str()[0] == 'l') {
            theta_left_current = input->position[i];
        } else if (input->name[i].c_str()[0] == 'b') {
            theta_back_current = input->position[i];
        } else if (input->name[i].c_str()[0] == 'r') {
            theta_right_current = input->position[i];
        }
    }
    duration = (timeCurrent - timePrevious).toSec();
    v_left  = (theta_left_current  - theta_left_previous ) / duration;
    v_back  = (theta_back_current  - theta_back_previous ) / duration;
    v_right = (theta_right_current - theta_right_previous) / duration;
    message.v_left  = v_left ;
    message.v_back  = v_back ;
    message.v_right = v_right;
    publisher.publish(message);
    timePrevious = timeCurrent;
    theta_left_previous  = theta_left_current ;
    theta_back_previous  = theta_back_current ;
    theta_right_previous = theta_right_current;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sensor/encoder");
    ros::NodeHandle node;
    while (!ros::Time::waitForValid()) {}
    ros::Subscriber subscriber = node.subscribe("/open_base/joint_states", 1, onJointStateMessage);
    publisher = node.advertise<open_base::Velocity>("wheel_velocity", 1);
    ros::spin();
    return 0;
}

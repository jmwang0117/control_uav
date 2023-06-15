#include <iostream>
#include <ros/ros.h>
#include <prometheus_msgs/CustomControl.h>

using namespace std;

class UAVController
{
private:
    ros::Publisher pub;
    float last_x, last_y, last_z, last_yaw;

public:
    UAVController(ros::NodeHandle& nh) : last_x(0), last_y(0), last_z(0), last_yaw(0)
    {
        pub = nh.advertise<prometheus_msgs::CustomControl>("/uav_control", 10);
    }

    void move(int control_mode, float x = 0, float y = 0, float z = 0, float yaw = 0, int move_mode = 0, int move_frame = 0)
    {
        prometheus_msgs::CustomControl cmd;
        cmd.Control_Mode = control_mode;

        if (control_mode == 1 || control_mode == 3)
        {
            // 如果Control_Mode是1或3,只发送Control_Mode
            pub.publish(cmd);
            if (control_mode == 1)
            {
                last_z = 3.0;
            }
        }
        else
        {
            // 更新位置
            last_x += x;
            last_y += y;
            last_z += z;
            last_yaw += yaw;

            // 发送全部数据
            cmd.x = last_x;
            cmd.y = last_y;
            cmd.z = last_z;
            cmd.yaw = last_yaw;
            cmd.Move_mode = move_mode;
            cmd.Move_frame = move_frame;

            pub.publish(cmd);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_controller");
    ros::NodeHandle nh;

    UAVController uav_controller(nh);
    ros::Rate rate(10);

    int user_input;

    while(ros::ok())
    {
        cout << "Please choose a command:" << endl;
        cout << "1: Takeoff, 2: Land" << endl;
        cout << "3: Forward, 4: Backward, 5: Left, 6: Right" << endl;
        cout << "7: Yaw left, 8: Yaw right" << endl;
        cout << "Enter the command number: ";
        cin >> user_input;
        cout << endl;

        switch(user_input)
        {
            
            case 1:
                uav_controller.move(1);
                break;
            case 2:
                uav_controller.move(3);
                break;
            case 3:
                uav_controller.move(4, 3);
                break;
            case 4:
                uav_controller.move(4, -3);
                break;
            case 5:
                uav_controller.move(4, 0, 3);
                break;
            case 6:
                uav_controller.move(4, 0, -3);
                break;
            case 7:
                uav_controller.move(4, 0, 0, 2);
                break;
            case 8:
                uav_controller.move(4, 0, 0, -2);
                break;
            default:
                cout << "Invalid command, please try again." << endl;
        }

        rate.sleep();
    }

    return 0;
}

#include <string>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class PS4_TELEOP
{
public:
    PS4_TELEOP(ros::NodeHandle &n)
    {
        ros::NodeHandle private_nh("~");

        if (!getRequiredParam(n, "max_teleop_forward_speed", this->max_forward_speed))
            return;
        if (!getRequiredParam(n, "max_teleop_reverse_speed", this->max_reverse_speed))
            return;

        // Subscribe Joystick
        this->joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 10, &PS4_TELEOP::subscribeJoystick, this);

        // Publish Ackermann
		private_nh.param<std::string>("pub_motor", this->pubAckermann, "/vesc/low_level/ackermann_cmd_mux/input/teleop");
        this->motor_chat = n.advertise<ackermann_msgs::AckermannDriveStamped>(this->pubAckermann, 10);

        // Publish teleop_enable
        this->enable_chat = n.advertise<std_msgs::Bool>("/teleop_enable", 10);

        ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        ackermann_msg.header.frame_id = "base_link";
    }

    ~PS4_TELEOP()
    {
        //
    }

    void subscribeJoystick(const sensor_msgs::Joy::ConstPtr &joy)
    {
        this->buttonSq = joy->buttons[0];
        this->buttonX = joy->buttons[1];
        this->buttonO = joy->buttons[2];
        this->buttonTr = joy->buttons[3];

        this->l1 = joy->buttons[4];
        this->r1 = joy->buttons[5];

        this->buttonShare = joy->buttons[8];
        this->buttonOptions = joy->buttons[9];
        this->buttonTouch = joy->buttons[13];

        this->leftStickX = joy->axes[0];
        this->leftStickY = joy->axes[1];

        this->l2 = joy->axes[3];
        this->r2 = joy->axes[4];

        this->rightStickX = joy->axes[2];
        this->rightStickY = joy->axes[5];

        this->arrowsX = joy->axes[9];
        this->arrowsY = joy->axes[10];
    }

    void run()
    {
        this->ackermann_msg.header.stamp = ros::Time::now();
        if (!this->enabled)
        {
            if (this->buttonShare == 1) this->enabled = true;
            //publishZero();
        }
        else
        {
            if (this->buttonOptions == 1)   this->enabled = false;
            this->publishControl();
        }
        publishEnable();
    }

    void publishEnable()
    {
        bool_msg.data = (unsigned char) this->enabled;
        enable_chat.publish(this->bool_msg);
    }

    void publishZero()
    {
        this->ackermann_msg.drive.steering_angle = 0;
        this->ackermann_msg.drive.speed = 0;
        motor_chat.publish(this->ackermann_msg);
    }

    void publishControl()
    {
        if (this->leftStickX >= -1 && this->leftStickX <= 1)
        {
            this->ackermann_msg.drive.steering_angle = this->leftStickX * 0.3F;    //M_PI
        }

        this->unit_forward = ((this->r2 * -1) + 1) / 2;
        this->unit_reverse = ((this->l2 * -1) + 1) / 2;

        if (unit_forward > 0 && unit_reverse == 0 && unit_forward <= 1)
        {
            this->ackermann_msg.drive.speed = (float) (this->unit_forward * this->max_forward_speed);
        }
        else if (unit_reverse > 0 && unit_forward == 0 && unit_reverse <= 1)
        {
            this->ackermann_msg.drive.speed = (float) (-1 * this->unit_reverse * this->max_reverse_speed);
        }
        else
        {
            this->ackermann_msg.drive.speed = 0;
        }
        motor_chat.publish(ackermann_msg);
    }

    template<typename T>
    inline bool getRequiredParam(const ros::NodeHandle &nh, std::string name, T &value) {
        if (nh.getParam(name, value))
            return true;

        ROS_FATAL("AckermannToVesc: Parameter %s is required.", name.c_str());
        return false;
    }

private:
    //Topics
    ros::Publisher motor_chat, enable_chat;
    ros::Subscriber joy_sub;

    std::string pubAckermann;
    ackermann_msgs::AckermannDriveStamped ackermann_msg;
    std_msgs::Bool bool_msg;

    bool enabled = false;
    double max_forward_speed;        // max forward speed (m/s)
    double max_reverse_speed;        // max reverse speed (m/s)
    double unit_forward = 0;         // forward throttle (0 to 1)
    double unit_reverse = 0;         // reverse throttle (0 to 1)

    float leftStickY = 0, leftStickX = 0,
        rightStickY = 0, rightStickX = 0,
        l2 = 1, r2 = 1,
        arrowsX = 0, arrowsY = 0;
    int buttonSq = 0, buttonX = 0, buttonO = 0, buttonTr = 0,
        buttonShare = 0, buttonOptions = 0,  buttonTouch = 0,
        l1 = 0, r1 = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ps4_teleop_node");
    ros::NodeHandle n;

    //Removed initialization code (Must push R2 and L2 in before pressing Share to enable!)
    PS4_TELEOP *joy_teleop = new PS4_TELEOP(n);
    ros::Rate loop_rate(20);    //20 Hz

    while (ros::ok())
    {
        joy_teleop->run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete joy_teleop;
    return 0;
}

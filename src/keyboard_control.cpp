#include <cstdio>
#include <sstream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>


//====================
// State Machine
//====================

typedef void (*transitionFunction) (int);

struct transition
{
    int newState;
    transitionFunction function;
};

void GoToStatePosition(int newState)
{
    float step = 0.5;

    ROS_INFO("\n\n-- Going to state %d\n", newState + 1);

    geometry_msgs::Pose move_target;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    move_target.position.x = 2;
    move_target.position.y = step * (newState % 3 - 1);
    move_target.position.z = (newState / 3) * step + 0.5;
    tf::Quaternion q;
    q.setRPY(0, 1.57, 0);
    move_target.orientation.x = q.x();
    move_target.orientation.y = q.y();
    move_target.orientation.z = q.z();
    move_target.orientation.w = q.w();

    move_group.setPoseTarget(move_target);
    move_group.move();

    ROS_INFO("-- done\n\n");
}

void DoNothing(int newState)
{
    ROS_INFO("\n\n-- Staing on state %d \n\n", newState + 1);
}

transition table[9][4] =
{
    { // 1
        { 1 - 1, DoNothing },
        { 4 - 1, GoToStatePosition },
        { 2 - 1, GoToStatePosition },
        { 1 - 1, DoNothing }
    },
    { // 2
        { 1 - 1, GoToStatePosition },
        { 5 - 1, GoToStatePosition },
        { 3 - 1, GoToStatePosition },
        { 2 - 1, DoNothing }
    },
    { // 3
        { 2 - 1, GoToStatePosition },
        { 6 - 1, GoToStatePosition },
        { 3 - 1, DoNothing },
        { 3 - 1, DoNothing }
    },
    { // 4
        { 4 - 1, DoNothing },
        { 7 - 1, GoToStatePosition },
        { 5 - 1, GoToStatePosition },
        { 1 - 1, GoToStatePosition }
    },
    { // 5
        { 4 - 1, GoToStatePosition },
        { 8 - 1, GoToStatePosition },
        { 6 - 1, GoToStatePosition },
        { 2 - 1, GoToStatePosition }
    },
    { // 6
        { 5 - 1, GoToStatePosition },
        { 9 - 1, GoToStatePosition },
        { 6 - 1, DoNothing },
        { 3 - 1, GoToStatePosition }
    },
    { // 7
        { 7 - 1, DoNothing },
        { 7 - 1, DoNothing },
        { 8 - 1, GoToStatePosition },
        { 4 - 1, GoToStatePosition }
    },
    { // 8
        { 7 - 1, GoToStatePosition },
        { 8 - 1, DoNothing },
        { 9 - 1, GoToStatePosition },
        { 5 - 1, GoToStatePosition }
    },
    { // 9
        { 8 - 1, GoToStatePosition },
        { 9 - 1, DoNothing },
        { 9 - 1, DoNothing },
        { 6 - 1, GoToStatePosition }
    }
};

class StateMachine
{
private:
    int currentState;

    int KeycodeToSignal(int keycode)
    {
        int signal = -1;

        switch (keycode)
        {
            case 87: case 119: signal = 1; break; // W
            case 65: case 97:  signal = 0; break; // A
            case 83: case 115: signal = 3; break; // S
            case 68: case 100: signal = 2; break; // D
        }
        return signal;
    }
public:
    StateMachine()
    {
        currentState = 5 - 1;
        GoToStatePosition(currentState);
    }

    void SignalInput(int keycode)
    {
        int signal = this->KeycodeToSignal(keycode);

        if (signal == -1)
            return;

        transitionFunction function = table[currentState][signal].function;
        currentState = table[currentState][signal].newState;

        if (function != NULL)
            function(currentState);
    }
};

//====================

int main (int argc, char **argv)
{
    ros::init(argc, argv, "cs_key_control");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    StateMachine sm;
    int keycode;

    system("/bin/stty raw");

    while ((keycode = getchar()) != 27)
    {
        system("/bin/stty cooked");

        ROS_INFO("\n-- %d\n", keycode);

        sm.SignalInput(keycode);

        system("/bin/stty raw");
    }

    system("/bin/stty cooked");

    return 0;
}

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include <sstream>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>


/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

int main(int argc, char **argv)
{

  int arraySize = 6;
  int joy_val[arraySize];

  const char *device;
  int js;
  struct js_event event;
  struct axis_state axes[3] = {0};
  size_t axis;

  if (argc > 1)
      device = argv[1];
  else
      device = "/dev/input/js0";

  js = open(device, O_RDONLY);

  if (js == -1){
    perror("Could not open joystick");
    return 0;  
  }

  ros::init(argc, argv, "joy_reader");
  ros::NodeHandle n;
  ros::Publisher joy_pub = n.advertise<std_msgs::Int32MultiArray>("joy_value", 1);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {

    std_msgs::Int32MultiArray msg;

    if(read_event(js, &event) == 0){
        axis = get_axis_state(&event,axes);
        if(axis < 3){
          joy_val[2*axis] = axes[axis].x;
          joy_val[2*axis + 1] = axes[axis].y; 
          //x0 = axes[0].x;
          //y0 = -axes[0].y;
        }
    }

    if (js == -1){
      perror("Joystick is closed");
      return 0;
    }
    
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = arraySize;
    msg.layout.dim[0].stride = 1;

    msg.data.assign(joy_val, joy_val + arraySize);

    joy_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}

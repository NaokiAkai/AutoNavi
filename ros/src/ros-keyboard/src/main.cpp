#include <ros/ros.h>
#include <iostream>
#include "keyboard.h"
using namespace std;

int main(int argc, char** argv)
{  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher pub_down = n.advertise<keyboard::Key>("keydown", 10);
  ros::Publisher pub_up = n.advertise<keyboard::Key>("keyup", 10);

  bool allow_repeat=false;
  int repeat_delay, repeat_interval;
  
  n.param<bool>( "allow_repeat", allow_repeat, false ); // disable by default
  n.param<int>( "repeat_delay", repeat_delay, SDL_DEFAULT_REPEAT_DELAY );
  n.param<int>( "repeat_interval", repeat_interval, SDL_DEFAULT_REPEAT_INTERVAL );
  
  if ( !allow_repeat ) repeat_delay=0; // disable 
  keyboard::Keyboard kbd( repeat_delay, repeat_interval );
  
  ros::Rate r(50);
  
  keyboard::Key k;
  bool pressed, new_event;
  while (ros::ok() && kbd.get_key(new_event, pressed, k.code, k.modifiers)) {
    if (new_event) {
      k.header.stamp = ros::Time::now();
      if (pressed) pub_down.publish(k);
      else pub_up.publish(k);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  ros::waitForShutdown();
}

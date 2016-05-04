#ifndef _VB_MAIN_
#define _VB_MAIN_

//#include <vb_util_lib/bag_logger.h>

//BagLogger *BagLogger::s_instance_ = 0;

void changemode(int dir)
{
	static struct termios oldt, newt;
	if ( dir == 1 )
	{
		tcgetattr( STDIN_FILENO, &oldt);
		newt = oldt;
		newt.c_lflag &= ~( ICANON | ECHO );
		tcsetattr( STDIN_FILENO, TCSANOW, &newt);
	}
	else
		tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}


int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}

bool userInputEnabled(ros::NodeHandle n)
{
	bool user_input = true;

	if (n.hasParam("/DisableUserInput"))
	{
	    user_input = false;
	}
	else
	{
	    n.getParam("UserInput", user_input);
	}
	ROS_INFO("USER INPUT %s", user_input ? "ENABLED" : "DISABLED");

	return user_input;
}

#endif



#include "Aria.h"
#include "math.h"

int main(int argc,char **argv)
{
	ArRobot robot;
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);
	Aria::init();
	ArSimpleConnector connector(&argc,argv);

	if (!connector.connectRobot(&robot))
	{
		Aria::shutdown();
		Aria::exit(1);
	}
	
	robot.comInt(ArCommands::ENABLE,1);
	robot.runAsync(false);

	// get goal location
	double x_goal,y_goal,theta_goal;
	printf("Enter the location of goal that about x(m),y(m),theta :");
	scanf("%lf%lf%lf",&x_goal,&y_goal,&theta_goal);


	robot.lock();
	robot.setVel(100);
	robot.setRotVel(20);
	robot.unlock();

	bool success = false;
	// test
	// the distance of origin and later
	double max_dis = 200;
	double dis = 0.0;
	while(!success)
	{
		if (robot.isMoveDone())
		{
			robot.stop();
			double x_now_dis = robot.getX();
			double y_now_dis = robot.getY();
			double x_diff = x_goal * 1000.0 - x_now_dis;
			double y_diff = y_goal * 1000.0 - y_now_dis;
			
			dis = sqrt(x_diff * x_diff + y_diff * y_diff);
			if (dis <= max_dis)
			{
				success = true;
				printf("Good!");
			}

			else
			{
				double direction = atan(y_diff - x_diff) * 180.0 / 3.14;
				if (y_diff < 0 && direction >0)
					direction -= 180.0;
				else if(y_diff >0 && direction <0)
					direction += 180.0;
				robot.setHeading(direction);
				
				bool rotate = false;
				 
				while(!rotate)
				{
					if(robot.isHeadingDone())
					{
						rotate = true;
					}
					else
					{	
						ArUtil::sleep(10);
					}
					
				}
		
				if (dis > 3000)
				{
					robot.move(3000);
				}
				else
				{
					if (dis < 50)
						 success = true;        			                 	printf("Good!");
				
					robot.move(50);
				}
			}
		}
		else
			ArUtil::sleep(500);
	}

	Aria::shutdown();
	Aria::exit(0);
}

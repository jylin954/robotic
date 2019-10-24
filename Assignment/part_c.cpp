#include "Aria.h"

 double safety_dis = 500.0;
 double value = 100.0;
 double value_vel = 50.0;
 double value_Rotvel = 10.0; 
 double max_vel = 800.0;
 double max_Rotvel = 50.0;  






void turn_up(ArRobot *robot,ArRangeDevice *sonar)
{

 double now_vel = robot->getVel();
 double dis = sonar->currentReadingPolar(-45.0,45.0);
 if (dis < safety_dis && now_vel > 0)
 {
 	if (now_vel > value)
		robot->setVel(now_vel - value);
	else 
		robot->setVel(0.0);
 }
 else
 {
	if (now_vel + value_vel <= max_vel)
		robot->setVel(now_vel + value_vel);
	else
		robot->setVel(max_vel);
 }
 printf("\n");
}


void turn_down(ArRobot *robot,ArRangeDevice *sonar)
{

 double now_vel = robot->getVel();
 double dis = sonar->currentReadingPolar(135.0,-135.0);
 if (dis < safety_dis && now_vel < 0)
 {
        if (now_vel < value * -1.0)
                robot->setVel(now_vel + value);
        else
                robot->setVel(0.0);
 }
 else
 {
        if (now_vel - value_vel >= max_vel * -1.0)
                robot->setVel(now_vel - value_vel);
        else
                robot->setVel(max_vel * -1.0);
 }
 printf("\n");
}



void turn_right(ArRobot *robot)
{
 double now_Rotvel = robot->getRotVel();
 if (now_Rotvel - value_Rotvel >= max_Rotvel * -1.0)
{
	robot->setRotVel(now_Rotvel - value_Rotvel);
}
 else
	robot->setRotVel(max_Rotvel * -1.0);
 printf("\n");
}

void turn_left(ArRobot *robot)
{
 double now_Rotvel = robot->getRotVel();
 if (now_Rotvel - value_Rotvel >= max_Rotvel)
{
        robot->setRotVel(now_Rotvel + value_Rotvel);
}
 else
        robot->setRotVel(max_Rotvel);
 printf("\n");
}





int main(int argc,char **argv)
{
 ArRobot robot; 
 // sonar
 ArSonarDevice sonar;
 robot.addRangeDevice(&sonar);
 
 // Initialize some global data
 Aria::init();
 // set up our simple connector
 ArSimpleConnector connector(&argc,argv);

 if (!connector.connectRobot(&robot)){
        printf("Could not connect to robot... exiting\n");
        Aria::shutdown();
        Aria::exit(1);
 }


 // turn on the motors
 robot.comInt(ArCommands::ENABLE, 1);
 // start the robot running
 robot.runAsync(false);

 // Used to perform actions when keyboard keys are pressed
 ArKeyHandler keyHandler; // a key handler so we can do our key handling
 Aria::setKeyHandler(&keyHandler); // let the global aria stuff know i


 // ArRobot contains an exit action for the Escape key. It also 
 // stores a pointer to the keyhandler so that other parts of the program can
 // use the same keyhandler.
 robot.attachKeyHandler(&keyHandler);
 printf("You may press escape to exit\n");

 // TODO: control the robot

 // Start of controling

 // 1. Lock the robot
 robot.lock();

 // 2. Write your control code here, 
 //    e.g. robot.setVel(150);  
 

 keyHandler.addKeyHandler(ArKeyHandler::UP,new ArGlobalFunctor2<ArRobot *,ArRangeDevice *>(&turn_up,&robot,&sonar));
 keyHandler.addKeyHandler(ArKeyHandler::DOWN,new ArGlobalFunctor2<ArRobot *,ArRangeDevice *>(&turn_down,&robot,&sonar));
 keyHandler.addKeyHandler(ArKeyHandler::RIGHT,new ArGlobalFunctor1<ArRobot *>(&turn_right,&robot));
 keyHandler.addKeyHandler(ArKeyHandler::LEFT,new ArGlobalFunctor1<ArRobot *>(&turn_left,&robot));


 // 3. Unlock the robot
 
 robot.setVel(0);
 robot.unlock();

 while(true){
 double now_vel = robot.getVel();
 if (now_vel > 0)
 {
 	double dis = sonar.currentReadingPolar(-45.0,45.0);
 	if (dis < safety_dis)
 	{
		printf("obstacle in %f \n",dis);
        	if (now_vel > value)
                	robot.setVel(now_vel - value);
        	else
        		robot.setVel(0.0);
 	}
 }
 else
 {	
	double dis = sonar.currentReadingPolar(135.0,-135.0);
        if (dis < safety_dis)
	{
		printf("obstacle in %f \n",dis);
		if (now_vel < value_vel * -1.0)
		{
                	robot.setVel(now_vel + value_vel);
        	}
		else
			robot.setVel(0.0);
	}
 }
 printf("%f %f %f\n", robot.getX(), robot.getY(), robot.getTh());
 ArUtil::sleep(300);
}
	
 // End of controling


 Aria::shutdown();

 Aria::exit(0);
}


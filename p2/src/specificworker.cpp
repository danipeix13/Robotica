#include "specificworker.h"
#include <chrono>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
    std::srand(std::time(nullptr));
}

void SpecificWorker::compute()
{
    float stop_threshold = 650, residue = 200, trim = 3, adv = 1000, rot = 1.3;

    if( auto ldata = laser_proxy->getLaserData(); !ldata.empty()) {
        //Using only distance values
        std::vector<float> distances;
        for(auto &point : ldata)
            distances.push_back(point.dist);

        //Sort and take the lower distance value
        int limit = distances.size()/trim;
        std::sort(distances.begin() + limit, distances.end() - limit, [](float a, float b){return a < b;});
        float minValue = distances[limit];

        //Set the speeds depending on minValue SWITCH
        if(minValue < stop_threshold) { //
            differentialrobot_proxy->setSpeedBase(0, (std::rand()%2) ? rot : -rot);
            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 2250));//random
        } else
            differentialrobot_proxy->setSpeedBase(adv, 0);
    }
}



int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


//int stop_threshold = 650, slow_threshold = 1250, min_speed = 50, max_speed = 700, residue = 200, trim = 3; //COPPELIA VALUES
////    float stop_threshold = 650.0, slow_threshold = 1250.0;
////    float min_speed = 0.0, max_speed = 0.6;
////    float residue = 0.1;
////    float trim = 3.0;   // GIRAFF  VALUES
//float adv, rot = 0;
//
////We have to control the robot' speed depending on the distance for keeping its balance
////Speed-Distance graphics
//float m = (max_speed - min_speed) * 1. / (slow_threshold - stop_threshold);
//float n = min_speed - m * stop_threshold + residue;
//
//if( auto ldata = laser_proxy->getLaserData(); !ldata.empty())
//{
////Using only distance values
//std::vector<float> distances;
//for(auto &point : ldata)
//distances.push_back(point.dist);
//
////Sort and take the lower distance value
//int limit = distances.size()/trim;
//std::sort(distances.begin() + limit, distances.end() - limit, [](float a, float b){return a < b;});
//float minValue = distances[limit];
//
////Set the speeds depending on minValue
//if(minValue < stop_threshold) { //
//rot = 0.8;
//adv = min_speed;
//}else if(minValue < slow_threshold)
//adv = m * minValue + n;
//else
//adv = max_speed;
//
//try
//{
//cout << "adv: " << adv << "     rot: " << rot << endl;
//differentialrobot_proxy->setSpeedBase(adv, rot);
//}
//catch(const Ice::Exception &e)
//{ std::cout << e.what() << std::endl;}
//}
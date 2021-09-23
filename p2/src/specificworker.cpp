#include "specificworker.h"
#include "cppitertools/sliding_window.hpp"

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
}

void SpecificWorker::compute()
{
    //dist < x1 --> dist = y1
    //dist > x2 --> dist = y2
    //else      --> dist = m * dist + n
    //int x1 = 600, x2 = 1000, y1 = 0, y2 = 600, residue = 300, trim = 3; //COPPELIA VALUES
    float x1 = 650.0, x2 = 1250.0, y1 = 0.0, y2 = 0.6, residue = 0.1, trim = 3.0;   //GIRAFF ROBO VALUES
    float adv, rot = 0.8, m = (y2 - y1) * 1. / (x2 - x1) , n = y1 - m * x1 + residue;

    if(auto ldata = laser_proxy->getLaserData(); !ldata.empty()) {
        //Using only distance values
        std::vector<float> distances;
        for(auto point : ldata)
            distances.push_back(point.dist);

        //Filter, try to fix laser measure errors
        if(distances[0] < 200)
            distances[0] = 200;
        for(auto &&window : iter::sliding_window(distances, 2)){
            if(window[1] < 200)
                window[1] = window[0];
        }

        //Sort and take the lower distance value
        int limit = distances.size()/trim;
        std::sort(distances.begin() + limit, distances.end() - limit, [=](float a, float b){return a < b;});
        float minValue = distances[limit];

        //Set the speeds depending on minValue, x1, x2, y1, y2
        bool stop = minValue < x1, slow = minValue < x2;
        adv = (stop) ? y1 : (slow) ? m * minValue + n : y2;
        try{
            std::cout << adv << std::endl;
            differentialrobot_proxy->setSpeedBase(adv, (stop) ? rot : 0);
        }catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

#include "specificworker.h"

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
    cout << "EMPIEZA" << endl;
    int stop_threshold = 700, trim = 5;
    float adv, rot;
    bool s[trim];//sector
    cout << "FIN DLECARACIONES" << endl;

    if (auto laser = laser_proxy->getLaserData(); !laser.empty()) {
        cout << "Entra en el if" << endl;
        int limit = laser.size() / trim;
        cout << "LIMIT" << endl;
        for(int i = 0; i < trim; i++){
            cout << i << endl;
            std::sort(laser.begin() + limit * i, laser.end() + limit * (i + 1), [](auto &a, auto &b) { return a.dist < b.dist; });
            s[i] = laser[limit * i].dist < stop_threshold;
        }

        if(s[0] && s[1]) //[0 && 1]  ^
        {
            cout << "A" << endl;
            adv = 1000;
            rot = 0;
        }
        else if (s[0]) // [0] <
        {
            cout << "AB" << endl;
            rot = -2;
            adv = 600;
        }else{ //else >
            cout << "ELSE" << endl;
            rot = 2;
            adv = 600;
        }
        cout << "SEND SPEEDS" << endl;
        differentialrobot_proxy->setSpeedBase(adv, rot);
    }
    else
    {
        cout << "######### NO LASER DATA ###########" << endl;
    }
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

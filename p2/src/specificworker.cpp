#include "specificworker.h"
#include <chrono>
#define GIRAR 0
#define AVANZAR 1
#define ESPIRAL 2

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
    static int estado;
    float stop_threshold = 600, trim = 3, adv, rot;
    int sleep;

    if( auto ldata = laser_proxy->getLaserData(); !ldata.empty())
    {
        int limit = ldata.size()/trim;
        std::sort(ldata.begin() + limit, ldata.end() - limit, [](auto &a , auto  &b){return a.dist < b.dist;});
        float minValue = ldata[limit].dist;
        if(minValue < stop_threshold)
            estado = GIRAR;
    }

    switch (estado)
    {
        case GIRAR:
            estado = girar(adv, rot, sleep);
            break;
        case AVANZAR:
            estado = avanzar(adv, rot, sleep);
            break;
        case ESPIRAL:
            estado = espiral(adv, rot, sleep);
            break;
        default:
            cout << "ERES TONTO X NO CONTROLAR LOS ESTADOS XD" << endl;
    }

    differentialrobot_proxy->setSpeedBase(adv, rot);
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));


//
//    float stop_threshold = 600, residue = 200, adv = 1000, rot = 1.3;
//
//    if( auto ldata = laser_proxy->getLaserData(); !ldata.empty()) {
//        //Using only distance values
//        std::vector<float> distances;
//        for(auto &point : ldata)
//            distances.push_back(point.dist);
//
//        //Sort and take the lower distance value
//        int limit = distances.size()/trim;
//        std::sort(distances.begin() + limit, distances.end() - limit, [](float a, float b){return a < b;});
//        float minValue = distances[limit];
//
//        //Set the speeds depending on minValue SWITCH
//        if(minValue < stop_threshold) { //
//            differentialrobot_proxy->setSpeedBase(0, /*(std::rand()%2) ? rot : */-rot);
//            std::this_thread::sleep_for(std::chrono::milliseconds(std::rand() % 2250));//random
//        } else
//            differentialrobot_proxy->setSpeedBase(adv, 0);
//    }
}

int SpecificWorker::girar(float &adv, float &rot, int &sleep)
{
    rot = 0.8;
    adv = 0;
    sleep = 1000;
    return AVANZAR;
}

int SpecificWorker::avanzar(float &adv, float &rot, int &sleep){
    rot = 0;
    adv = 1000;
    sleep = 1000;
    return ESPIRAL;

}

int SpecificWorker::espiral(float &adv, float &rot, int &sleep)
{
    rot = 0.3;
    adv = 1000;
    sleep = 0;
    return ESPIRAL;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

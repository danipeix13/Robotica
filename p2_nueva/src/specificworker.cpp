#include "specificworker.h"
#include <eigen3/Eigen/Dense>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    threshold = 620.0;
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
    float adv, rot;
    if (auto laser = laser_proxy->getLaserData(); !laser.empty()) {
        switch(sectores(laser)){
            case 0: case 5:
                cout << "ADV" << endl; adv = 1000; rot = 0;                break;
            case 1:
                cout << "PEG" << endl; adv = 1000; rot = seguirPegado();   break;
            default:
                cout << "ROT" << endl; adv = 50;    rot = 0.4;
        }
        cout << adv << " - " << rot << endl;
        differentialrobot_proxy->setSpeedBase(adv, rot);
    } else
        cout << "######### NO LASER DATA ###########" << endl;
}

int SpecificWorker::sectores(RoboCompLaser::TLaserData laser){
    threshold += 1.1;
    RoboCompLaser::TLaserData sectA (laser.begin(), laser.begin()+laser.size()/3);
    RoboCompLaser::TLaserData sectB (laser.begin()+laser.size()/3, laser.end()-laser.size()/3);
    RoboCompLaser::TLaserData sectC (laser.end()-laser.size()/3, laser.end());
    std::sort(sectA.begin(), sectA.end(), [](auto &a, auto &b){return a.dist<b.dist;});
    std::sort(sectB.begin(), sectB.end(), [](auto &a, auto &b){return a.dist<b.dist;});
    std::sort(sectC.begin(), sectC.end(), [](auto &a, auto &b){return a.dist<b.dist;});
    int x = 0;
    if(sectA[0].dist < threshold)
        x += 1;
    if(sectB[0].dist < threshold)
        x += 2;
    if(sectB[0].dist < threshold)
        x += 4;
    return x;
}

float SpecificWorker::seguirPegado(){
    float rot = 0.0, x = 0.5;
    if (auto laser = laser_proxy->getLaserData(); !laser.empty()) {
        RoboCompLaser::TLaserData sectA (laser.begin(), laser.begin()+laser.size()/3);
        float ref1 = sectA[40].dist, ref2 = sectA[55].dist;
        if(abs(ref1 - ref2) > 5)
            rot = (2*x) / (1 + pow(M_E, (ref2 - ref1)/5)) - x;
    }else
        cout << "NO HAY LASER" << endl;
    return rot;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

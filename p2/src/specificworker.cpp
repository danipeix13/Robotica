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

void SpecificWorker::compute() {
    static int estado = ESPIRAL;
    int stop_threshold = 700, trim = 3;

    if (auto ldata = laser_proxy->getLaserData(); !ldata.empty()) {
        int limit = ldata.size() / trim;
        std::sort(ldata.begin() + limit, ldata.end() - limit, [](auto &a, auto &b) { return a.dist < b.dist; });
        float minValue = ldata[limit].dist;
        if (minValue < stop_threshold)
            estado = GIRAR;
    }

    switch (estado) {
        case GIRAR:
            estado = girar();
            break;
        case AVANZAR:
            estado = avanzar();
            advEspiral = 0;
            rotEspiral = 2;
            break;
        case ESPIRAL:
            estado = espiral();
            break;
        default:
            cout << "ERES TONTO X NO CONTROLAR LOS ESTADOS XD" << endl;
    }
}

int SpecificWorker::girar()
{

    differentialrobot_proxy->setSpeedBase(100, -2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return AVANZAR;
}

int SpecificWorker::avanzar()
{
    differentialrobot_proxy->setSpeedBase(1000, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    return ESPIRAL;
}

int SpecificWorker::espiral() {

    // adv = x; rot = y(x);
    if (advEspiral < 500)
        advEspiral += 10;
    else if(advEspiral < 800)
        advEspiral += 8;
    else if (advEspiral < 1000)
    {
        advEspiral += 6;
        rotEspiral -= 0.0025;
    }
    else
        rotEspiral -= 0.002;

    differentialrobot_proxy->setSpeedBase(advEspiral, rotEspiral);
    return ESPIRAL;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

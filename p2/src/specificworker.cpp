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
    rotEspiral = 1.75;
    advEspiral = 0;
}

void SpecificWorker::compute() {

    static int estado = ESPIRAL;
    int stop_threshold = 700, continue_threshold = 1000, trim = 4;

    if (auto ldata = laser_proxy->getLaserData(); !ldata.empty()) {

        int limit = ldata.size() / trim;
        std::sort(ldata.begin() + limit, ldata.end() - limit, [](auto &a, auto &b) { return a.dist < b.dist; });
        float minValue_front = ldata[limit].dist;

        if (minValue_front < stop_threshold)
            estado = GIRAR;

        switch (estado) {
            case GIRAR:
                estado = girar();
                break;
            case AVANZAR:
                estado = avanzar();
                break;
            case ESPIRAL:
                estado = espiral();
                break;
        }
    }
}

int SpecificWorker::girar()
{
    differentialrobot_proxy->setSpeedBase(100, -2);
    cout << "GIRAR" << endl;
    return AVANZAR;
}

int SpecificWorker::avanzar()
{
    differentialrobot_proxy->setSpeedBase(1000, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    advEspiral = 0;
    rotEspiral = 1.75;
    cout << "AVANZAR" << endl;
    return ESPIRAL;
}

int SpecificWorker::espiral() {

    if (advEspiral < 500){
        advEspiral += 10;
        cout << "ESPIRAL 1" << endl;
    } else if(advEspiral < 800) {
        advEspiral += 8;
        cout << "ESPIRAL 2" << endl;
    } else if (advEspiral < 1000) {
        advEspiral += 6;
        rotEspiral -= 0.0005;
        cout << "ESPIRAL 3" << endl;
    } else {
        rotEspiral -= 0.004;
        cout << "ESPIRAL 4" << endl;
    }
    cout << "ADV: " << advEspiral << "  ROT:    " << rotEspiral << endl;
    differentialrobot_proxy->setSpeedBase(advEspiral, rotEspiral);
    return ESPIRAL;
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

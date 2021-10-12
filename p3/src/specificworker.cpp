/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
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
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}
    QRect dimensions(-5000, -2500, 10000, 5000);
    viewer = new AbstractGraphicViewer(this, dimensions);
    this->resize(900,450);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract
    try
    {
        RoboCompGenericBase::TBaseState bState;
        differentialrobot_proxy->getBaseState(bState);
        last_point = QPointF(bState.x, bState.z);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    robot_polygon->setRotation(bState.alpha*180/M_PI);
    robot_polygon->setPos(bState.x, bState.z);
//    if(auto ldata = laser_proxy->getLaserData(); !ldata.empty()){
//        draw_laser(ldata);
//    }
    if(!target.isNull()){
        auto point_from_robot = world2robot(bState);
        auto values = cartesians2polars(point_from_robot);
        if(get<1>(values) > M_PI/90){
            differentialrobot_proxy->setSpeedBase(0, 0.2);
            cout << "GIRANDO" << endl;
        } else if(get<0>(values) > 100) {
            differentialrobot_proxy->setSpeedBase(500, 0);
            cout << "RECTO" << endl;
        } else {
            differentialrobot_proxy->setSpeedBase(0, 0);
        }
    }


}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    target = p;
    qInfo() << p << endl;
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    viewer->scene.addLine(bState.x, bState.z, p.x(), p.y());
}

void SpecificWorker::draw_laser(const RoboCompLaser::TLaserData &ldata) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    delete laser_polygon;

    QPolygonF poly;//vector
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's reference system
    for(auto &point : ldata)
    {
        auto coords = polars2cartesians(make_tuple(point.dist, point.angle), M_PI_2);
        QPointF new_point(get<0>(coords), get<1>(coords));
        poly.push_back(new_point);
    }

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

std::tuple<float, float> SpecificWorker::polars2cartesians(std::tuple<float, float> pols, float correction)
{
    float x = get<0>(pols) * cos(get<1>(pols) + correction), y = get<0>(pols) * sin(get<1>(pols) + correction);
    return make_tuple(x, y);
}

std::tuple<float, float> SpecificWorker::cartesians2polars(std::tuple<float, float> cart)
{
    float dist = sqrt(pow(get<0>(cart), 2) + pow(get<1>(cart), 2)), angle = atan2(get<1>(cart), get<0>(cart));
    return make_tuple(dist, angle);
}

std::tuple<float, float> SpecificWorker::world2robot(RoboCompGenericBase::TBaseState bState)
{   //TODO: transpose
    float angle = bState.alpha;
    Eigen::Vector2f T(bState.x, bState.z), point_in_world(target.x(),target.y());
    Eigen::Matrix2f R;
    R << cos(angle), -sin(angle), sin(angle), cos(angle);
    Eigen::Vector2f point_in_robot = R * point_in_world - T;
    return make_tuple(point_in_robot[0], point_in_robot[1]);//return target from robot's pov
}




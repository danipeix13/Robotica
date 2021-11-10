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
#include <cppitertools/range.hpp>
#define MAX_ADV_SPEED 1000
#define umbral 400

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destruct                       or
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
    QRect dimensions(-10000, -5000, 20000, 10000);
    viewer = new AbstractGraphicViewer(this->frame, dimensions);
    this->resize(1200,600);
    robot_polygon = viewer->add_robot(ROBOT_LENGTH);
    laser_in_robot_polygon = new QGraphicsRectItem(-10, 10, 20, 20, robot_polygon);
    laser_in_robot_polygon->setPos(0, 190);     // move this to abstract

    RoboCompFullPoseEstimation::FullPoseEuler r_state;

    // timeseries
    customPlot = new QCustomPlot(this->frame_2);
    customPlot->resize(frame_2->size());
    customPlot->addGraph(); // blue line
    customPlot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    customPlot->xAxis->setRange(0, 9999999);
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->yAxis->setRange(-1, 2);
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
    customPlot->show();

    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
    state = State::IDLE;
    grid.initialize(dimensions, 200, &viewer->scene);
}

void SpecificWorker::compute()
{
    RoboCompFullPoseEstimation::FullPoseEuler r_state;
    RoboCompLaser::TLaserData ldata;
    try
    {
        r_state = fullposeestimation_proxy->getFullPoseEuler();
        robot_polygon->setRotation(r_state.rz*180/M_PI);
        robot_polygon->setPos(r_state.x, r_state.y);
        ldata = laser_proxy->getLaserData();
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    update_grid(ldata, r_state);

    switch (state)
    {
        case State::EXPLORING:
            // turn until zero derivative
            break;
        case State::SEARCHING_DOOR:
            //
            break;
        case State::TO_DOOR:
            // goto
            break;
        case State::TO_MID_ROOM:
            //
            break;
    }

//    auto &[adv, rot] = speeds;
//    try
//    {
//        differentialrobot_proxy->setSpeedBase(adv, rot);
//    }
//    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}

}

void SpecificWorker::update_grid(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state)
{
    static float ant=0;
    //vector
    // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's reference system
    QPolygonF poly;
    poly << QPointF(0, 0);

    for (auto &p: ldata)
    {
        auto x = p.dist * sin(p.angle), y = p.dist * cos(p.angle);
        poly << QPointF(x, y);

        Eigen::Vector2f finRayo(x, y);
        int num_steps = std::ceil(p.dist * 2.0 / grid.TILE_SIZE);
        for (auto &&step: iter::range(0.0, 1.0-(1./num_steps), 1. / num_steps))
            grid.add_miss(robot2world(r_state, step * finRayo));

        if (p.dist <= 4000)
            grid.add_hit(robot2world(r_state, finRayo));
        else
            grid.add_miss(robot2world(r_state, finRayo));
    }

    auto change = grid.percentage_changed();
    realtime_data_slot((change-ant)*200000);


    qInfo() << __FUNCTION__ << (change-ant)*100000;
    ant = change;

    draw_laser(poly, r_state);
}
////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::realtime_data_slot(double value)
{
    static double index = 0;
    customPlot->graph(0)->addData(index++, value);
    customPlot->xAxis->setRange(index, 1000, Qt::AlignRight);
    customPlot->replot();
}

void SpecificWorker::new_target_slot(QPointF p)
{
//    target.pos = p;
//    qInfo() << p << endl;
//    RoboCompGenericBase::TBaseState bState;
//    differentialrobot_proxy->getBaseState(bState);
//    target.a = bState.x - target.pos.x();
//    target.b = target.pos.y() - bState.z;
//    target.c = (-target.b)*bState.z + (-target.a)*bState.x;
//    target.activo = true;
}

void SpecificWorker::draw_laser(QPolygonF poly, RoboCompFullPoseEstimation::FullPoseEuler bState) // robot coordinates
{
    static QGraphicsItem *laser_polygon = nullptr;
    // code to delete any existing laser graphic element
    if(laser_polygon != nullptr)
        viewer->scene.removeItem(laser_polygon);
    //robot base Qpointf

    QColor color("Red");
    color.setAlpha(40);
    laser_polygon = viewer->scene.addPolygon(laser_in_robot_polygon->mapToScene(poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}

std::tuple<float, float> SpecificWorker::world2robot(RoboCompGenericBase::TBaseState bState)
{   //TODO: transpose
//    float angle = bState.alpha;
//    Eigen::Vector2f T(bState.x, bState.z), point_in_world(target.pos.x(),target.pos.y());
//    Eigen::Matrix2f R;
//    R <<  cos(angle), sin(angle),
//         -sin(angle), cos(angle);
//    Eigen::Vector2f point_in_robot = R * (point_in_world - T);
//    return make_tuple(point_in_robot[0], point_in_robot[1]);//return target from robot's pov
}

Eigen::Vector2f SpecificWorker::robot2world(const RoboCompFullPoseEstimation::FullPoseEuler &bState, const Eigen::Vector2f &punto)
{   //TODO: transpose
    float angle = bState.rz;
    Eigen::Vector2f robot(bState.x, bState.y);
    Eigen::Matrix2f R;
    R <<  cos(angle), -sin(angle),
            sin(angle), cos(angle);
    Eigen::Vector2f point_in_world = R * punto + robot;
    return point_in_world;
}




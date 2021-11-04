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
    state = Estado::IDLE;
    target.activo = false;
    min_distance = 9999999999;
}

void SpecificWorker::compute()
{
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    robot_polygon->setRotation(bState.alpha*180/M_PI);
    robot_polygon->setPos(bState.x, bState.z);
    RoboCompLaser::TLaserData ldata;
    std::tuple<float, float> speeds;

    QPolygonF poly;

    if(ldata = laser_proxy->getLaserData(); not ldata.empty()) {
        //vector
        // code to fill poly with the laser polar coordinates (angle, dist) transformed to cartesian coordinates (x,y), all in the robot's reference system
        poly << QPointF(0, 0);
        for (auto &p: ldata)
            poly << QPointF(p.dist * sin(p.angle), p.dist * cos(p.angle));
        draw_laser(poly, bState);
    }


        switch(state)
        {
            case Estado::IDLE:
                if(target.activo)
                    state = Estado::AVANZAR;
                break;
            case Estado::AVANZAR:
                cout << "avanzar" << endl;
                std::sort(ldata.begin() + ldata.size()/3, ldata.end() - ldata.size()/3,
                          [](auto &a, auto &b){ return a.dist < b.dist;});
                if(ldata[ldata.size()/3].dist < umbral)
                    state = Estado::BORDEAR;
                else if(check_target(poly,bState))
                    state = Estado::VEO_TARGET;
                else
                    speeds = avanzar(bState);
                break;
            case Estado::BORDEAR:
                cout << "bordear ";
                if(check_target(poly,bState))
                {
                    check_free_path_to_target(ldata,bState);
                    state = Estado::VEO_TARGET;
                }
                else if(check_line(bState))
                    state = Estado::AVANZAR;
                else
                    speeds = bordear(bState,ldata);
                break;
            case Estado::VEO_TARGET:

                    speeds = avanzar(bState);
                    QPointF bsRobot(bState.x,bState.z);
                    QPointF distVector = target.pos - bsRobot;

                    if(sqrt(pow(distVector.x(),2) + pow(distVector.y(),2)) < umbral)
                    {
                        cout << "Hemos llegado ! :)" << endl;
                        target.activo = false;
                        state = Estado::IDLE;
                    }
                break;
        }
    auto &[adv, rot] = speeds;
    try
    {
        differentialrobot_proxy->setSpeedBase(adv, rot);
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;};

}

////////////////////////////////////////////////////////////////////////////////
std::tuple<float, float> SpecificWorker::avanzar(RoboCompGenericBase::TBaseState bState)
{
    float rot = 0, adv = 0;
    auto[x, y] = world2robot(bState);
    float dist = sqrt(pow(x, 2) + pow(y, 2));
    if(dist > 300)
    {
        rot = atan2(-y, x) + M_PI_2;
        float reduc_distance = (dist < 1000) ? dist / 1000.0 : 1.0, reduc_angle = pow(M_E, -pow(rot, 2));
        adv = MAX_ADV_SPEED * reduc_distance * reduc_angle/2;
    }
    return make_tuple(adv, rot);
}

std::tuple<float, float> SpecificWorker::bordear(RoboCompGenericBase::TBaseState bState, RoboCompLaser::TLaserData ldata)
{
    float rot = 0, adv = 0;
    std::sort(ldata.begin() + ldata.size()/3, ldata.end() - ldata.size()/3,
              [](auto &a, auto &b){ return a.dist < b.dist;});

    if(ldata[ldata.size()/3].dist < 500) {
        rot = 1;
        cout << "--> girar" << endl;
    }else {
        rot = seguirPegado(ldata);
        adv = MAX_ADV_SPEED / 4;
        cout << "--> pegao" << endl;
    }
    return make_tuple(adv, rot);

}

float SpecificWorker::seguirPegado(RoboCompLaser::TLaserData ldata){
    float rot = 0.0, x = 0.5;
    RoboCompLaser::TLaserData sectA (ldata.begin(), ldata.begin() + ldata.size() / 3);
    float ref1 = sectA[40].dist, ref2 = sectA[55].dist;
    if(ref1 > 1000 or ref2 > 1000)
        rot = -1;
    if(abs(ref1 - ref2) > 5)
        rot = (2*x) / (1 + pow(M_E, (ref2 - ref1)/5)) - x;
    return rot;
}

bool SpecificWorker::check_target(QPolygonF poly,RoboCompGenericBase::TBaseState bState)
{
    auto [a, b] = world2robot(bState);
    cout << a << " " << b << endl;
    return poly.containsPoint(QPointF(a,b),Qt::OddEvenFill);
}

bool SpecificWorker::check_line(RoboCompGenericBase::TBaseState bState)
{

    auto [a, b] = world2robot(bState);
    float distance = (abs(target.a*a + target.b*b + target.c)) / sqrt(pow(target.a, 2) + pow(target.b, 2));
    bool estoy = distance < 50;

    if(estoy)
    {
        auto distance2point = sqrt(pow(a-target.pos.x(),2) + pow(b-target.pos.y(),2));
        if (distance2point < min_distance)
            min_distance = distance2point;
        else
            estoy = false;
    }
    return estoy;
}


////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::new_target_slot(QPointF p)
{
    target.pos = p;
    qInfo() << p << endl;
    RoboCompGenericBase::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);
    target.a = bState.x - target.pos.x();
    target.b = target.pos.y() - bState.z;
    target.c = (-target.b)*bState.z + (-target.a)*bState.x;
    target.activo = true;
}

////////////////////////////////////////////////////////////////////

void SpecificWorker::draw_laser(QPolygonF poly, RoboCompGenericBase::TBaseState bState) // robot coordinates
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
    float angle = bState.alpha;
    Eigen::Vector2f T(bState.x, bState.z), point_in_world(target.pos.x(),target.pos.y());
    Eigen::Matrix2f R;
    R <<  cos(angle), sin(angle),
         -sin(angle), cos(angle);
    Eigen::Vector2f point_in_robot = R * (point_in_world - T);
    return make_tuple(point_in_robot[0], point_in_robot[1]);//return target from robot's pov
}

Eigen::Vector2f SpecificWorker::from_robot_to_world(Eigen::Vector2f p,RoboCompGenericBase::TBaseState bState)
{
    float angle = bState.alpha;
    Eigen::Vector2f T(bState.x, bState.z);
    Eigen::Matrix2f R;
    R <<  cos(angle), sin(angle),
            -sin(angle), cos(angle);
    return R.inverse() * (p - T);
}

void SpecificWorker::check_free_path_to_target( const RoboCompLaser::TLaserData &ldata, RoboCompGenericBase::TBaseState bState)
{
    // lambda to convert from Eigen to QPointF
    auto toQPointF = [](const Eigen::Vector2f &p){ return QPointF(p.x(),p.y());};
    auto tuple2Eigen = [](std::tuple<float, float> point){ auto [x, y] = point;  return Eigen::Vector2f(x, y); };

    // create polyggon
    QPolygonF pol;
    pol << QPointF(0,0);
    for(const auto &l: ldata)
        pol << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));

    // create tube lines
    auto goal_r = tuple2Eigen(world2robot(bState));
    Eigen::Vector2f robot(0.0,0.0);
    // number of parts the target vector is divided into
    float parts = (goal_r).norm()/(ROBOT_LENGTH/4);
    Eigen::Vector2f rside(220, 200);
    Eigen::Vector2f lside(-220, 200);
    if(parts < 1) return;

    QPointF p,q, r;
    for(auto l: iter::range(0.0, 1.0, 1.0/parts))
    {
        p = toQPointF(robot*(1-l) + goal_r*l);
        q = toQPointF((robot+rside)*(1-l) + (goal_r+rside)*l);
        r = toQPointF((robot+lside)*(1-l) + (goal_r+lside)*l);
        if( not pol.containsPoint(p, Qt::OddEvenFill) or
            not pol.containsPoint(q, Qt::OddEvenFill) or
            not pol.containsPoint(r, Qt::OddEvenFill))
            break;
    }

    // draw
    QLineF line_center(toQPointF(from_robot_to_world(robot,bState)), toQPointF(from_robot_to_world(Eigen::Vector2f(p.x(),p.y()),bState)));
    QLineF line_right(toQPointF(from_robot_to_world(robot+rside,bState)), toQPointF(from_robot_to_world(Eigen::Vector2f(q.x(),q.y()),bState)));
    QLineF line_left(toQPointF(from_robot_to_world(robot+lside,bState)), toQPointF(from_robot_to_world(Eigen::Vector2f(r.x(),q.y()),bState)));
    static QGraphicsItem *graphics_line_center = nullptr;
    static QGraphicsItem *graphics_line_right = nullptr;
    static QGraphicsItem *graphics_line_left = nullptr;
    static QGraphicsItem *graphics_target = nullptr;
    if (graphics_line_center != nullptr)
        viewer->scene.removeItem(graphics_line_center);
    if (graphics_line_right != nullptr)
        viewer->scene.removeItem(graphics_line_right);
    if (graphics_line_left != nullptr)
        viewer->scene.removeItem(graphics_line_left);
    if (graphics_target != nullptr)
        viewer->scene.removeItem(graphics_target);
    graphics_line_center = viewer->scene.addLine(line_center, QPen(QColor("Blue"), 30));
    graphics_line_right = viewer->scene.addLine(line_right, QPen(QColor("Orange"), 30));
    graphics_line_left = viewer->scene.addLine(line_left, QPen(QColor("Magenta"), 30));
    graphics_target = viewer->scene.addEllipse(-100, -100, 200, 200, QPen(QColor("Blue")), QBrush(QColor("Blue")));
    graphics_target->setPos(target.pos.x(), target.pos.y());
}





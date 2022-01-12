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
    QRect dimensions(-5100, -2600, 10200, 5200);
    viewer = new AbstractGraphicViewer(this->frame_grid, dimensions);
    viewer_graph = new AbstractGraphicViewer(this->frame_graph, dimensions);

    this->resize(900,450); //1200, 600
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
    grid.initialize(dimensions, 100, &viewer->scene);
}

void SpecificWorker::compute()
{
    static vector<QRectF> puntitos;
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

    float adv = 0, rot = 0;
    string state_str = "";
    float initial_angle;
    int num_puertas = 0;
    bool insert = true;
    static int current_room = 0, n_rooms;
    static Eigen::Vector2f caminito;
    static Dynamic_Window dw;

    switch (state) {
        case State::IDLE:
            state_str = "IDLE";
            state = State::INIT_TURN;
            break;
        case State::INIT_TURN:
            initial_angle = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            num_puertas = puertas.size();
            state = State::EXPLORING;
            break;
        case State::EXPLORING:
        {

            state_str = "EXPLORING";
            // turn until zero derivative
            //cout << r_state.rz + initial_angle << endl;
            float current = (r_state.rz < 0) ? (2 * M_PI + r_state.rz) : r_state.rz;
            float stop = grid.percentage_changed()*1000;
            cout << "STOP VALUE: " << stop << endl;
            if (fabs(current - initial_angle) < (M_PI + 0.1) and fabs(current - initial_angle) > (M_PI - 0.1))
            {
                    // {
                QRectF rect(r_state.x,r_state.y, 500 , 500);
                puntitos.push_back(rect);
                state = State::SEARCHING_DOOR;
                cout << "TERMINA DE GIRAR PORFAVOR";
            }
            else
            {
                rot = 0.5;
                state_str = "ELSE EXPLORING";
                std::vector<float> derivatives(ldata.size());
                derivatives[0] = 0;
                for (auto &&[k, p]: iter::sliding_window(ldata, 2) | iter::enumerate)
                {
                    derivatives[k + 1] = p[1].dist - p[0].dist;
                    //qInfo() << "Resta puntos: " << p[1].dist - p[0].dist;
                }

                std::vector<Eigen::Vector2f> peaks;
                for (const auto &&[k, der]: iter::enumerate(derivatives))
                {
                    if (der > 700)
                    {
                        //Guarda el punto de la derivada anterior
                        const auto &l = ldata.at(k - 1);
                        peaks.push_back(robot2world(r_state, Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                    }
                    else if (der < -700)
                    {
                        //Guarda el punto de esta derivada
                        const auto &l = ldata.at(k);
                        peaks.push_back(
                                robot2world(r_state, Eigen::Vector2f(l.dist * sin(l.angle), l.dist * cos(l.angle))));
                    }
                }
                //Tenemos todos los puntos de las puertas, ahora se busca la relacion entre ellas
                for (auto &&c: iter::combinations_with_replacement(peaks, 2))
                {
                    if ((c[0] - c[1]).norm() < 1100 and (c[0] - c[1]).norm() > 600)
                    {
                        Door d{c[0], c[1]};
                        d.pasada = false;
                        d.rooms.insert(current_room);
                        if (auto r = std::find_if(puertas.begin(), puertas.end(),
                                                  [d](auto a) { return d == a; }); r == puertas.end())
                            puertas.emplace_back(d);
                    }
                }

                //cout << "Puertas: " << puertas.size() << endl;

                static std::vector<QGraphicsItem *> door_lines;
                for (auto dp: door_lines) viewer->scene.removeItem(dp);
                door_lines.clear();

                for (const auto r: puertas)
                {
                    door_lines.push_back(viewer->scene.addLine(r.dPoint1[0], r.dPoint1[1], r.dPoint2[0], r.dPoint2[1],
                                                               QPen(QColor("Blue"), 100)));
                    door_lines.back()->setZValue(200);
                }
//                break;
            }
            break;
        }
        case State::SEARCHING_DOOR:
        {

            if(puertas.size() > 0 )
                selectedDoor = &puertas[puertas.size()-1]; //chooseDor();

            caminito = selectedDoor->get_external_midpoint();
            current_room += 1;

            selectedDoor->rooms.insert(current_room);
            pintarGrafo(puntitos);
            cout << "Puntitos Size: " << puntitos.size() << endl;
            state = State::TO_DOOR1;
        }
            break;
        case State::TO_DOOR1:
        {
            state_str = "TO_DOOR";
            // gotoxy
            //auto punto_medio = caminito.front();
            auto[x, y] = world2robot(r_state, caminito);

            // goto p0int
            Eigen::Vector2f tr = {x,y};
            float dist = tr.norm();
            if(dist < 300)  // at target
            {
                selectedDoor->pasada = true;
                if (puertas.size() == n_rooms)
                {
                    cout << "C A C A D E B A C A C A C A D E B A C A C A C A D E B A C A" << endl;
                }
                else
                {
                    n_rooms = puertas.size();
                }
                cout << "He llegado" << endl;
                state = State::EXPLORING;
            }
            else  // continue to room
            {
                // call dynamic window
                QPolygonF laser_poly;
                for(auto &&l : ldata)
                    laser_poly << QPointF(l.dist*sin(l.angle), l.dist*cos(l.angle));
                auto [_, __, advance, rotation, ___] = dw.compute(tr, laser_poly,
                                                         Eigen::Vector3f(r_state.x, r_state.y, r_state.rz),
                                                         Eigen::Vector3f(r_state.vx, r_state.vy, r_state.vrz),
                                                         nullptr /*&viewer_robot->scene*/);

                //cout << "Rotation: " << rotation << " Advance: " << advance << endl;
                const float rgain = 0.6;
                float reduc_distance = (dist < 1000) ? dist / 1000.0 : 1.0;
                rot = rgain*rotation;
                adv = MAX_ADV_SPEED * reduc_distance * pow(M_E, -pow(rot*6, 2));
            }
        break;
        }
        case State::TO_MID_ROOM:
            cout << "CACADEBACA" << endl;
            state = State::EXPLORING;

            break;
    }
    cout << state_str << endl;

    /*CACADEBACA*/


    try
    {
        differentialrobot_proxy->setSpeedBase(adv, rot);
        //cout << "Velocidades: adv->" << adv << " rot->" << rot << endl;
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}
}

void SpecificWorker::pintarGrafo(std::vector<QRectF> p)
{
    if (p.size() > 1)
    {
        viewer_graph->scene.clear();

//        std::vector<QString> c;
//        c.push_back("Black");
//        c.push_back("Red");
//        c.push_back("Blue");
//    int i = 0;

        for (int i = 0; i < p.size(); i++)
        {
//            auto color = c[++i % c.size()];
            auto punto1 = p[i], punto2 = p[(i+1) % p.size()];
            QLineF linea(punto1.center().x(), punto1.center().y(), punto2.center().x(), punto2.center().y());
            viewer_graph->scene.addLine(linea, QPen(QColor("Red"),100));
        }
//    QRectF rectangulito();
//    viewer_graph->scene.addRect(rectangulito, QPen(QColor("Yellow")), QBrush(QColor("Yellow")));
        for (auto &&punto: p)
        {
            viewer_graph->scene.addEllipse(punto, QPen(QColor("Yellow")), QBrush(QColor("Yellow")));

        }

//
//    for (auto &&puerta: puertas)
//    {
//        auto color = c[++i % c.size()];
//        if(puerta.rooms.size() == 2)
//        {
//            int room1 = *puerta.rooms.begin();
//            int room2 = *(std::next(puerta.rooms.begin()));
//            //(puerta.rooms.size() == 2) ? *(++iter) : room1;
//
//            cout << "room1: " << room1 << " room2: " << room2 << endl;
//            auto punto1 = p.at(room1), punto2 = p.at(room2);
//            cout << "PUNTO 1: (" << punto1.center().x() << ", " << punto1.center().y() << ")" << endl;
//            cout << "PUNTO 2: (" << punto2.center().x() << ", " << punto2.center().y() << ")" << endl;
//            QLineF linea(punto1.center().x(), punto1.center().y(), punto2.center().x()-300, punto2.center().y()-300);
//            viewer_graph->scene.addLine(linea, QPen(QColor(color),100));
//        }
//    }
    }

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
    realtime_data_slot((change-ant)*20000);
//    qInfo() << __FUNCTION__ << (change-ant)*100000;
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

std::tuple<float, float> SpecificWorker::world2robot(const RoboCompFullPoseEstimation::FullPoseEuler &r_state, const Eigen::Vector2f punto)
{   //TODO: transpose
    float angle = r_state.rz;
    Eigen::Vector2f T(r_state.x, r_state.y), point_in_world = punto;
    Eigen::Matrix2f R;
    R <<  cos(angle), sin(angle),
         -sin(angle), cos(angle);
    Eigen::Vector2f point_in_robot = R * (point_in_world - T);
    return make_tuple(point_in_robot[0], point_in_robot[1]);//return target from robot's pov
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





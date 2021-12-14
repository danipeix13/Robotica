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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <grid2d/grid.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
#include <cppitertools/enumerate.hpp>
#include <qcustomplot.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

public slots:
	void compute();
	int startup_check();
	void initialize(int period);
    void new_target_slot(QPointF p);
    void realtime_data_slot(double);

private:
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    AbstractGraphicViewer *viewer;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    QPointF last_point;

    struct Target
    {
        float a,b,c;
        QPointF pos;
        bool activo;
    };
    struct Door
    {
        Eigen::Vector2f dPoint1, dPoint2;
        int fromRoom, toRoom;
        bool operator==(const Door &d1)
        {
            const int EROR = 500;
            return (dPoint1 - d1.dPoint1).norm() < EROR and (dPoint2 - d1.dPoint2).norm() < EROR or
            (dPoint1 - d1.dPoint2).norm() < EROR and (dPoint2 - d1.dPoint1).norm() < EROR;
        };

        Eigen::Vector2f get_midpoint() const {return dPoint1 + ((dPoint2-dPoint1)/2.0);};
        std::vector<Eigen::Vector2f> get_caminito(RoboCompFullPoseEstimation::FullPoseEuler &bState)
        {
            Eigen::ParametrizedLine<float, 2> r1 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (dPoint1-dPoint2).unitOrthogonal());
            //Eigen::ParametrizedLine<float, 2> r2 =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (dPoint1-dPoint2).unitOrthogonal());
            //qInfo() << __FUNCTION__ << r2.pointAt(800.0).x() << r.pointAt(800.0).y();
            std::vector<Eigen::Vector2f> caminoPuerta {r1.pointAt(-1000.0), r1.pointAt(1500.0)};
            auto robot = Eigen::Vector2f(bState.x,bState.y);
            std::sort(caminoPuerta.begin(), caminoPuerta.end(), [=](auto &a, auto &b){ return (robot - a).norm() < (robot - b).norm();});
            //sort(caminoPuerta.begin(), caminoPuerta.end(),[=](auto &a, auto &b){ return })
            return caminoPuerta;
        };

    };

    Grid grid;
    void draw_laser(QPolygonF poly, RoboCompFullPoseEstimation::FullPoseEuler bState);
    std::tuple<float, float> world2robot(const RoboCompFullPoseEstimation::FullPoseEuler &r_state, const Eigen::Vector2f punto);
    Eigen::Vector2f robot2world(const RoboCompFullPoseEstimation::FullPoseEuler &bState, const Eigen::Vector2f &punto);

    // timeserues
    QCustomPlot *customPlot;
    QTimer time_series_timer;

    void update_grid(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);

    enum class State {IDLE, INIT_TURN, EXPLORING, TO_MID_ROOM, TO_DOOR1, TO_DOOR2, SEARCHING_DOOR};
    State state;
    std::vector<Door> puertas;
    Door selectedDoor;
};

#endif

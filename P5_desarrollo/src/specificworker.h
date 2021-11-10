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
    Grid grid;
    void draw_laser(QPolygonF poly, RoboCompFullPoseEstimation::FullPoseEuler bState);
    std::tuple<float, float> world2robot(RoboCompGenericBase::TBaseState bState);
    Eigen::Vector2f robot2world(const RoboCompFullPoseEstimation::FullPoseEuler &bState, const Eigen::Vector2f &punto);

    // timeserues
    QCustomPlot *customPlot;
    QTimer time_series_timer;

    void update_grid(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);

    enum class State {IDLE, EXPLORING, TO_MID_ROOM, TO_DOOR, SEARCHING_DOOR};
    State state = State::IDLE;
};

#endif

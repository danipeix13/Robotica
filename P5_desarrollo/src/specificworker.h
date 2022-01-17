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
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations_with_replacement.hpp>
#include "dynamic_window.h"

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
	bool startup_check_flag;
    AbstractGraphicViewer *viewer,*viewer_graph;
    const int ROBOT_LENGTH = 400;
    QGraphicsPolygonItem *robot_polygon;
    QGraphicsRectItem *laser_in_robot_polygon;
    Grid grid;
    QCustomPlot *customPlot;

    struct Door
    {
        Eigen::Vector2f dPoint1, dPoint2;
        std::set<int> rooms;

        bool operator==(const Door &d1)
        {
            const int ERROR = 500;
            return ((dPoint1 - d1.dPoint1).norm() < ERROR && (dPoint2 - d1.dPoint2).norm() < ERROR) ||
            ((dPoint1 - d1.dPoint2).norm() < ERROR && (dPoint2 - d1.dPoint1).norm() < ERROR);
        };

        Eigen::Vector2f get_midpoint() const
        {
            return dPoint1 + ((dPoint2-dPoint1) / 2.0);
        };

        Eigen::Vector2f get_external_midpoint() const
        {
            Eigen::ParametrizedLine<float, 2> r =  Eigen::ParametrizedLine<float, 2>(get_midpoint(), (dPoint1 - dPoint2).unitOrthogonal());
            return r.pointAt(1750.0);
        };
    };
    Door *selectedDoor;
    std::vector<Door> puertas;

    enum class State {IDLE, EXPLORING, TO_DOOR, SEARCHING_DOOR};
    State state;

    void draw_laser(QPolygonF poly, RoboCompFullPoseEstimation::FullPoseEuler bState);
    std::tuple<float, float> world2robot(const RoboCompFullPoseEstimation::FullPoseEuler &r_state, const Eigen::Vector2f punto);
    Eigen::Vector2f robot2world(const RoboCompFullPoseEstimation::FullPoseEuler &bState, const Eigen::Vector2f &punto);
    void flip_text(QGraphicsTextItem *text);
    void pintarGrafo(std::vector<QRectF> p, bool fin);
    void update_grid(const RoboCompLaser::TLaserData &ldata, const RoboCompFullPoseEstimation::FullPoseEuler &r_state);
};

#endif

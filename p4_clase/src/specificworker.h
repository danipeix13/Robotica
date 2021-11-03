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
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <eigen3/Eigen/Eigen>
#include <cppitertools/enumerate.hpp>

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


    enum class Estado{AVANZAR, IDLE, BORDEAR, VEO_TARGET};
    Estado state;
    Target target;
    float min_distance;
    void draw_laser(QPolygonF poly, RoboCompGenericBase::TBaseState bState);
    std::tuple<float, float> world2robot(RoboCompGenericBase::TBaseState bState);
    std::tuple<float, float> avanzar(RoboCompGenericBase::TBaseState bState);
    std::tuple<float, float> bordear(RoboCompGenericBase::TBaseState bState, RoboCompLaser::TLaserData ldata);
    float seguirPegado(RoboCompLaser::TLaserData ldata);
    bool check_target(QPolygonF poly, RoboCompGenericBase::TBaseState bState);
    bool check_line(RoboCompGenericBase::TBaseState bState);

};

#endif

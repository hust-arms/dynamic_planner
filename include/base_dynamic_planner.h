/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  base_dynamic_planner.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/12/25
  * - Brief:     Definition of base dynmaic planner
  *****************************************************************************
**/

#ifndef DYNAMIC_PLANNER_H_
#define DYNAMIC_PLANNER_H_

namespace dynamic_planner{
/**
 * @class BaseDynamicPlanner
 * @brief Provides an interface for local planners used in navigation. All dynamic planners written as plugins for the navigation stack must adhere to this interface.
 */
class BaseDynamicPlanner{
public:
	/**
	 * @brief Determine whether the usv should stop avoiding.
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param pos_th Orienataion of usv
	 * @return Return true if avoidance canbe stopped. Otherwise, return false.
	 */
	virtual bool isStopAvoidance(double pos_x, double pos_y, double pos_th) = 0;

	/**
	 * @brief Determine whether the following target is blocked.
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param pos_th Orientation of usv
	 * @param goal_x X coordinate of followed goal point
	 * @param goal_y Y coordinate of followed goal point
	 * @return Return true if followed point is blocked by obstacle. Otherwise, return false.
	 */
	virtual bool isFollowBlocked(double pos_x, double pos_y, double pos_th, double goal_x, double goal_y) = 0;

	/**
	 * @brief Dynamic planner provide interface to compute temperature goal point in state of avoiding
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param pos_th Orientation of usv 
	 * @param goal_x X coordinate of followed point
	 * @param goal_y Y coordinate of followed point
	 * @param avoid_goal_x X coordinate of temperature goal point for usv in state of avoiding
	 * @param avoid_goal_y Y coordinate of temperature goal point for usv in state of avoiding
	 * @return If get a temp goal point successfully, return true. Otherwise, return false
	 */
	virtual bool getAvoidGoal(double pos_x, double pos_y, double pos_th, double goal_x, double goal_y, double& avoid_goal_x, double& avoid_goal_y) = 0;

	/**
	 * @brief Virtual destructor 
	 */
	virtual ~BaseDynamicPlanner(){};
};
}; // end of namespace

#endif

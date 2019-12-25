/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tp_dynamic_planner.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/12/25
  * - Brief:     Definition of three path dynmaic planner which implements obstacle avoiding during navigation through three path logic
  *****************************************************************************
**/

#ifndef DYNAMIC_PLANNER_H_
#define DYNAMIC_PLANNER_H_

#include <vector>
#include <boost/thread/mutex.hpp>
#include "BaseDynamicPlanner"

namespace dynamic_planner{
typedef boost::unique_lock<boost::shared_mutex> WriteLock;
typedef boost::shared_lock<boost::shared_mutex> ReadLock; 

/**
 * @brief Enum class of last avoid choice
 */
enum class AvoidOrientation{
	MID,
	LEFT,
	RIGHT
};

/**
 * @class DynamicPlanner
 * @brief Get temperature goal point for usv in obstacle avoiding state.
 */
class TPDynamicPlanner : public BaseDynamicPlanner{
public:
	DynamicPlanner(){};
	~DynamicPlanner(){};

        /**
         * @brief Determine whether the usv should stop avoiding.
         * @param pos_x X coordinate of usv position
         * @param pos_y Y coordinate of usv position
         * @param pos_th Orienataion of usv
         * @return Return true if avoidance canbe stopped. Otherwise, return false.
         */
        bool isStopAvoidance(pos_x, pos_y, pos_th)override;


	/**
         * @brief Determine whether the following target is blocked.
         * @param pos_x X coordinate of usv position
         * @param pos_y Y coordinate of usv position
         * @param pos_th Orientation of usv
         * @param goal_x X coordinate of followed goal point
         * @param goal_y Y coordinate of followed goal point
         * @return Return true if followed point is blocked by obstacle. Otherwise, return false.
         */
	bool isFollowBlocked(double pos_x, double pos_y, double pos_th, double goal_x, double goal_y)override;

        /**
         * @brief Dynamic planner provide interface to compute temperature goal point in state of avoiding
         * @param pos_x X coordinate of usv position
         * @param pos_y Y coordinate of usv position
         * @param pos_th Orientation of usv 
         * @param avoid_goal_x X coordinate of temperature goal point for usv in state of avoiding
         * @param avoid_goal_y Y coordinate of temperature goal point for usv in state of avoiding
         * @return If get a temp goal point successfully, return true. Otherwise, return false
         */
        bool getAvoidGoal(double pos_x, double pos_y, double pos_th, double& avoid_goal_x, double& avoid_goal_y)override;

	/**
	 * @brief Update obstacle buffer.
	 * @param new_obs_buffer Input new observation buffer of obstacle
	 */
	void updateObsBuffer(std::vector<std::pair<double, double>>& new_obs_buffer);

private:
	/**
	 * @brief Output line cost value of three path.
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param pos_th Orientation of usv
	 * @param mid_line_cost Line cost of mid way to output
	 * @param left_line_cost Line cost of left way to output
	 * @param right_line_cost Line cost of right way to output
	 */
	void ouputLineCostOfTP(double pos_x, double pos_y, double pos_th, double& mid_line_cost, double& left_line_cost, double& right_line_cost);

	/**
	 * @brief Get cost of visual line.
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param pos_th Orientation of usv
	 * @return Returen cost of visual line
	 */
	bool isFrontPathFree(double pos_x, double pos_y, double pos_th);

	/**
	 * @brief Get cost of visual line.
	 * @param pos_x X coordinate of usv position
	 * @param pos_y Y coordinate of usv position
	 * @param detect_x X coordinate of detected point
	 * @param detect_y Y coordinate of detected point
	 * @return Returen cost of visual line
	 */
	double lineCost(double pos_x, double pos_y, double detect_x, double detect_y);


private:	
	double detect_ang_l_;
	double detect_ang_r_;
	double obs_detect_d_; // distance to detect obstacle

	AvoidOrientation last_choice_; // record last choice of avoidance

	size_t avoid_count_; // record number of avoiding

	boost::shared_mutex obs_buffer_rw_mutex_; // mutex for write and read of obstacle buffer
	std::vector<std::pair<double, double>> obs_buffer_; // position buffer of obstacle
};
}; // end of namespace

#endif





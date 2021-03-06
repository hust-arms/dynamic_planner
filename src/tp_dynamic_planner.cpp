/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tp_dynamic_planner.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/12/25
  * - Brief:     Implementation of three path dynmaic planner which implements obstacle avoiding during navigation through three path logic
  *****************************************************************************
**/

#include "tp_dynamic_planner.h"

namespace dynamic_planner{

const double PI = 3.1415926;

TPDynamicPlanner::TPDynamicPlanner(){
	detect_ang_l_ = 45; // degree
	detect_ang_r_ = 45;
	obs_detect_d_ = 5; // meter
	ship_width_ = 0.5; // meter
	check_num_ = 10;
}

bool TPDynamicPlanner::isStopAvoidance(double pos_x, double pos_y, double pos_th){
	return isFrontPathFree(pos_x, pos_y, pos_th);
}

bool TPDynamicPlanner::isFollowBlocked(double pos_x, double pos_y, double pos_th, double goal_x, double goal_y){
	double delt_x = std::fabs(goal_x - pos_x);
	double delt_y = std::fabs(goal_y - pos_y);
	double dist_to_goal = std::sqrt(delt_x * delt_y + delt_y * delt_y);
	// If goal point is closer, compute with position of goal point. Otherwise, use terminate of obstacle detection 
	if(obs_detect_d_ <= dist_to_goal){ 
		double detect_x = pos_x + obs_detect_d_ * std::cos(pos_th);
		double detect_y = pos_y + obs_detect_d_ * std::sin(pos_th);
		return (lineCost(pos_x, pos_y, detect_x, detect_y) < 0);
	}
	else{
		return (lineCost(pos_x, pos_y, goal_x, goal_y) < 0);
	}
}

bool TPDynamicPlanner::getAvoidGoal(double pos_x, double pos_y, double pos_th, double& avoid_goal_x, double& avoid_goal_y){
	double mid_line_cost, left_line_cost, right_line_cost;
	outputLineCostOfTP(pos_x, pos_y, pos_th, mid_line_cost, left_line_cost, right_line_cost);

	if(mid_line_cost == -1 && left_line_cost == -1 && right_line_cost == -1){
		return false;
	}	

	// Transform -1 to maximum value
	if(mid_line_cost < 0){
		mid_line_cost = static_cast<double>(INT_MAX);
	}
	if(left_line_cost < 0){
		left_line_cost = static_cast<double>(INT_MAX);
	}
	if(right_line_cost < 0){
		right_line_cost = static_cast<double>(INT_MAX);
	}

	// Get all possible terminal point
	double detect_x_m, detect_y_m;
	double detect_x_l, detect_y_l;
        double detect_x_r, detect_y_r;
	getObsDetectionArea(pos_x, pos_y, pos_th, detect_x_m, detect_y_m, detect_x_l, detect_y_l, detect_x_r, detect_y_r);

	// Way choose strategy
	if(left_line_cost < right_line_cost){
		if(left_line_cost < mid_line_cost){
			double left_th = pos_th - detect_ang_l_ * PI / 180;
			++avoid_count_;
			avoid_goal_x = detect_x_l;
			avoid_goal_y = detect_y_l;
			last_choice_ = AvoidOrientation::LEFT;
			return true;
		}
		else if (left_line_cost == mid_line_cost){
			if(avoid_count_){
				if(last_choice_ == AvoidOrientation::LEFT){
					double left_th = pos_th - detect_ang_l_ * PI / 180;
					avoid_goal_x = detect_x_l;
					avoid_goal_y = detect_y_l;
					++avoid_count_;
					last_choice_ = AvoidOrientation::LEFT;
					return true;
				}
				else{
					avoid_goal_x = detect_x_m;
					avoid_goal_y = detect_y_m;
					++avoid_count_;
					last_choice_ = AvoidOrientation::MID;
					return true;
				}
			}
			else{
				avoid_goal_x = detect_x_m;
				avoid_goal_y = detect_y_m;
				++avoid_count_;
				last_choice_ = AvoidOrientation::MID;
				return true;
			}
		}
		else if(left_line_cost == right_line_cost){
			if(avoid_count_){
				if(last_choice_ == AvoidOrientation::RIGHT){
					double right_th = pos_th + detect_ang_r_ * PI / 180;
					avoid_goal_x = detect_x_r;
					avoid_goal_y = detect_y_r;
					++avoid_count_;
					last_choice_ = AvoidOrientation::RIGHT;
					return true;
				}
				else{
					double left_th = pos_th - detect_ang_l_ * PI / 180;
					avoid_goal_x = detect_x_l;
					avoid_goal_y = detect_y_l;
					++avoid_count_;
					last_choice_ = AvoidOrientation::LEFT;
					return true;
				}
			}
			else{
				double left_th = pos_th - detect_ang_l_ * PI / 180;
                                avoid_goal_x = detect_x_l;
                                avoid_goal_y = detect_y_l;
                                ++avoid_count_;
                                last_choice_ = AvoidOrientation::LEFT;
                                return true;
			}
		}
	}
	else{
		if(right_line_cost < mid_line_cost){
			double right_th = pos_th + detect_ang_r_ * PI / 180;
			avoid_goal_x = detect_x_r;
			avoid_goal_y = detect_y_r;
			++avoid_count_;
			last_choice_ = AvoidOrientation::RIGHT;
			return true;
		}	
		else if(right_line_cost == mid_line_cost){
			if(avoid_count_){
				if(last_choice_ == AvoidOrientation::RIGHT){
					double right_th = pos_th + detect_ang_r_ * PI / 180;
					avoid_goal_x = detect_x_r;
					avoid_goal_y = detect_y_r;
					++avoid_count_;
					last_choice_ = AvoidOrientation::RIGHT;
					return true;
				}
				else{
					avoid_goal_x = detect_x_m;
					avoid_goal_y = detect_y_m;
					++avoid_count_;
					last_choice_ = AvoidOrientation::MID;
					return true;
				}
			}
			else{
				avoid_goal_x = detect_x_m;
				avoid_goal_y = detect_y_m;
				++avoid_count_;
				last_choice_ = AvoidOrientation::MID;
				return true;
			}
		}
	}
}

void TPDynamicPlanner::updateObsBuffer(std::vector<std::pair<double, double>>& new_obs_buffer){
	// Lock obstacle buffer during data writing
	{
		WriteLock wlock(obs_buffer_rw_mutex_);
		obs_buffer_.clear(); // clear buffer
		obs_buffer_.insert(obs_buffer_.end(), new_obs_buffer.begin(), new_obs_buffer.end()); // update
	} 	
}

void TPDynamicPlanner::outputLineCostOfTP(double pos_x, double pos_y, double pos_th, double& mid_line_cost, double& left_line_cost, double& right_line_cost){
	// Calculate line cost of mid path
	double detect_x_mid = pos_x + obs_detect_d_ * std::cos(pos_th);
	double detect_y_mid = pos_y + obs_detect_d_ * std::sin(pos_th);
	mid_line_cost = lineCost(pos_x, pos_y, detect_x_mid, detect_y_mid);
	// Calculate line cost of left path
	double detect_ang_left = detect_ang_l_ * PI / 180;
	double detect_x_left = pos_x + obs_detect_d_ * std::cos(pos_th + detect_ang_left) / std::cos(detect_ang_left);
	double detect_y_left = pos_y + obs_detect_d_ * std::sin(pos_th + detect_ang_left) / std::cos(detect_ang_left);
	left_line_cost = lineCost(pos_x, pos_y, detect_x_left, detect_y_left);

	// Calculate line cost of right 
	double detect_ang_right = detect_ang_r_ * PI / 180;
	double detect_x_right = pos_x + obs_detect_d_ * std::cos(pos_th - detect_ang_right) / std::cos(detect_ang_right);
	double detect_y_right = pos_y + obs_detect_d_ * std::sin(pos_th - detect_ang_right) / std::cos(detect_ang_right);
	right_line_cost = lineCost(pos_x, pos_y, detect_x_right, detect_y_right);
}


void TPDynamicPlanner::getObsDetectionArea(double pos_x, double pos_y, double pos_th,
                        double& detect_x_m, double& detect_y_m,
                        double& detect_x_l, double& detect_y_l,
                        double& detect_x_r, double& detect_y_r){
	detect_x_m = pos_x + obs_detect_d_ * std::cos(pos_th);
	detect_y_m = pos_y + obs_detect_d_ * std::sin(pos_th);

	double detect_ang_left = detect_ang_l_ * PI / 180;
	detect_x_l = pos_x + obs_detect_d_ * std::cos(pos_th + detect_ang_left) / std::cos(detect_ang_left);
	detect_y_l = pos_y + obs_detect_d_ * std::sin(pos_th + detect_ang_left) / std::cos(detect_ang_left);
	
	double detect_ang_right = detect_ang_r_ * PI / 180;
	detect_x_r = pos_x + obs_detect_d_ * std::cos(pos_th - detect_ang_right) / std::cos(detect_ang_right);
	detect_y_r = pos_y + obs_detect_d_ * std::sin(pos_th - detect_ang_right) / std::cos(detect_ang_right);
}



bool TPDynamicPlanner::isFrontPathFree(double pos_x, double pos_y, double pos_th){
	double detect_x_m = pos_x + obs_detect_d_ * std::cos(pos_th);
	double detect_y_m = pos_y + obs_detect_d_ * std::sin(pos_th);
	return (lineCost(pos_x, pos_y, detect_x_m, detect_y_m) >= 0);
}

double TPDynamicPlanner::lineCost(double pos_x, double pos_y, double detect_x, double detect_y){
	// Get buffer for obstacle checking points
	std::vector<std::pair<double, double>> collision_check_buffer;
	double delt_x = (detect_x - pos_x) / check_num_; 
	double delt_y = (detect_y - pos_y) / check_num_;
	double delt_dist = std::sqrt(delt_x * delt_x + delt_y * delt_y);

	for(int i = 0; i < check_num_ - 2; ++i){
		collision_check_buffer.push_back(std::pair<double, double>(pos_x + (i + 1) * delt_x, pos_y + (i + 1) * delt_y));
	}

	double line_cost = 0;

	// Check collision
        std::vector<double> avr_obs_dist;
	if(obs_buffer_.size()){
		for(int check_ind = 0; check_ind < collision_check_buffer.size(); ++check_ind){
			std::vector<double> obs_dist_buffer;
			// Traverse all obstacle points in obstacle buffer
			{
				ReadLock read_lock(obs_buffer_rw_mutex_);
				for(int obs_ind = 0; obs_ind < obs_buffer_.size(); ++obs_ind){
					double obs_dist_del_x = obs_buffer_[obs_ind].first - collision_check_buffer[check_ind].first;
					double obs_dist_del_y = obs_buffer_[obs_ind].second - collision_check_buffer[check_ind].second;
					double obs_dist = std::sqrt(obs_dist_del_x * obs_dist_del_x + obs_dist_del_y * obs_dist_del_y);
					if(obs_dist <= std::max(delt_dist, std::sqrt(2) * ship_width_)){
						return -1;
					}
					else{
						obs_dist_buffer.push_back(obs_dist);
					}
				}
			}// end of read lock
			std::sort(obs_dist_buffer.begin(), obs_dist_buffer.end());
			if(obs_dist_buffer.size()){
				// Calculate the average value for each collision checking point
				avr_obs_dist.push_back(std::accumulate(obs_dist_buffer.begin() + 1, obs_dist_buffer.end() - 1, 0) / (obs_dist_buffer.size() - 2));
			}
		}
	}
	else{
		return 0;
	}

	// Calculate line cost
	double avr_dist = std::accumulate(avr_obs_dist.begin(), avr_obs_dist.end(), 0);	
	line_cost = avr_dist + varianceCalc(avr_obs_dist, avr_dist) / avr_obs_dist.size();

	return line_cost;
}

double TPDynamicPlanner::varianceCalc(std::vector<double>& data_src, double avr){
		double variance = 0;
		// Caculate variance
		std::for_each(data_src.begin(), data_src.end(), [avr, &variance](double data){
			variance += std::pow((data - avr), 2);
		});

		return variance;
	}

};

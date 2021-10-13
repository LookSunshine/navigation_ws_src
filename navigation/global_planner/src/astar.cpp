/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}
// 每次取出open队列中代价f最小的一个点的索引，并进行下一次open队列的更新
// cycles = nx_ * ny_ * 2，为所有格子数的2倍
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear(); //清空队列，作为open队列
    int start_i = toIndex(start_x, start_y); // 起始点坐标（start_x, start_y）是全部数据中的第start_i个
    queue_.push_back(Index(start_i, 0)); // 将起始点和代价0放入open队列

    std::fill(potential, potential + ns_, POT_HIGH); // 将ns_个potential初始化为极大值，1e10
    potential[start_i] = 0; // 将起始点的potential置为0

    int goal_i = toIndex(end_x, end_y); // 目标点(end_x, end_y)是全部数据中的第goal_i个
    int cycle = 0; // 计数器

    while (queue_.size() > 0 && cycle < cycles) { // 当open队列不为空，并且循环次数小于cycles
        Index top = queue_[0]; // open队列中代价最小的点
        std::pop_heap(queue_.begin(), queue_.end(), greater1()); // pop_heap将根节点放在容器尾部
        queue_.pop_back(); // pop_back将容器尾部的点，也就是代价最小的点，移除

        int i = top.i; // 从open队列中取得的代价最小点的序号
        if (i == goal_i) // 如果序号与目标点序号相等
            return true; //找到了到目标点的path

        // 计算当前点的上下左右四个点的代价
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++; // 计数+1
    }

    return false; // 如果open队列空或者循环次数大于cycles，说没有找到到达目标点的path
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    // 判断当前要计算的点next_i是否溢出，是否已经遍历过，是否为障碍物或无信息的点
    if (next_i < 0 || next_i >= ns_) // 索引号小于0或者大于ns_，索引溢出
        return;

    if (potential[next_i] < POT_HIGH) // 代价值小于极大值，说明next_i已经在open队列中，已经遍历过了
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION)) // 有障碍或者是无信息的点
        return;
    // 索引next_i的点是可遍历的
    // 计算起始点start到当前点next_i的代价
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_; // x是当前点的行坐标，y是当前点的列坐标， nx_是地图上每行的格子数
    float distance = abs(end_x - x) + abs(end_y - y); // 计算当前点到目标点的曼哈顿距离

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_)); // push_back将当前点的索引和代价F加入open队列中
    std::push_heap(queue_.begin(), queue_.end(), greater1()); // push_heap进行小顶堆排序
}

} //end namespace global_planner

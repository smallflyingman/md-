@[TOC](ST_Bounds_Decider)

# 前言

STBoundsDecider的功能：
1、对动态障碍物以及阻塞当前引导线的最近一个静态障碍物进行st图构建
2、对不影响纵向规划的障碍物设置IGNORE属性,并按照设定轨迹给出每一个障碍物boundary的最优决策(overtake/yield)，最终决策出最优的Drivable_st_boundary

主要的代码路径为：
modules/planning/tasks/deciders/st_bounds_decider/**st_bounds_decider.cc**
modules/planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h

涉及相关的辅助类为：
modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.h
modules/planning/tasks/deciders/st_bounds_decider/st_obstacles_processor.cc
modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.h
modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.cc
modules/planning/tasks/deciders/st_bounds_decider/st_driving_limits.h
modules/planning/tasks/deciders/st_bounds_decider/st_driving_limits.cc

# 1、主要流程
1、调用InitSTBoundsDecider初始化
2、调用GenerateRegularSTBound生成regular_st_bound、regular_vt_bound、st_guide_line
3、调用 RecordSTGraphDebug()存储ST图数据，包括障碍物ST边界、自车t时刻的上下界、最终引导线
# 2、主要框架与函数依赖关系
主要框架：
主要的类是st_bounds_decider，其中的Process()作为入口，三个相关的辅助类st_obstacles_processor、st_guide_line、st_driving_limits
![Alt](https://img-blog.csdnimg.cn/37575cdef22b44a7bce6833b23710cf1.png#pic_center#pic_center)主要函数的依赖关系，其中以st_bounds_decider的成员函数和st_obstacles_processor为主。
![Alt](https://img-blog.csdnimg.cn/5ac003246f9d452b86b1ab79d851f359.png#pic_center#pic_center)
# 3、主要函数和数据结构
以下是以st_bounds_decider和st_obstacles_processor的类成员函数分节，其中涉及到的其他类函数直接贴出代码和路径进行分析。
|***st_bounds_decider***|
|------|
|st_bounds_config_ <br>st_guide_line_<br>st_driving_limits_<br>st_obstacles_processor_|
| STBoundsDecider()<br>Process<br>InitSTBoundsDecider()<br>GenerateFallbackSTBound()<br>GenerateRegularSTBound()<br>RemoveInvalidDecisions()<br>RankDecisions()<br>BackwardFlatten()<br> RecordSTGraphDebug()|

|***st_obstacles_processor***|
|------|
|planning_time_<br>planning_distance_<br>path_data_<br>vehicle_param_<br>adc_path_init_s_<br>path_decision_<br> obs_t_edges_<br>obs_t_edges_idx_<br>obs_id_to_st_boundary_<br> obs_id_to_decision_<br>candidate_clear_zones_<br>obs_id_to_alternative_st_boundary_<br>adc_low_road_right_segments_<br>history_ |
|STObstaclesProcessor()<br>Init()<br>MapObstaclesToSTBoundaries()<br>GetAllSTBoundaries()<br> GetSBoundsFromDecisions()<br>GetLimitingSpeedInfo()<br>SetObstacleDecision(<br>SetObstacleDecision(<br>ComputeObstacleSTBoundary()<br>GetOverlappingS()<br> GetSBoundingPathPointIndex()<br>IsPathPointAwayFromObstacle()<br>IsADCOverlappingWithObstacle()<br>FindSGaps()<br>DetermineObstacleDecision()<br>IsSWithinADCLowRoadRightSegment()|

|***st_guide_line***|
|----|
|t0_<br> s0_<br> v0_<br>SpeedData guideline_speed_data_|
|STGuideLine()<br>Init()<br>GetGuideSFromT()<br>UpdateBlockingInfo()|

|***st_driving_limits***|
|----|
| max_acc_<br>max_dec_<br>max_v_<br>upper_t0_<br>upper_v0_<br>upper_s0_<br>lower_t0_<br>lower_v0_<br>lower_s0_<br>curvature_speed_limits_s_v_<br>traffic_speed_limits_s_v_<br>obstacles_speed_limits_s_v_|
|STDrivingLimits() <br>Init()<br>GetVehicleDynamicsLimits()<br>UpdateBlockingInfo()|
## 3.1 STBoundsDecider()
功能：STBoundsDecider类的构造函数
输入：
```cpp
const TaskConfig& config  //task配置参数对象config
const std::shared_ptr<DependencyInjector>& injector //访问车辆状态的类对象injector
```

## 3.2 Process()
功能：STBoundsDecider类的入口，Process()依次调用InitSTBoundsDecider()、GenerateRegularSTBound()、 RecordSTGraphDebug()完成ST图边界构建。

输入：
1、Frame类对象：包括routing、规划起点、规划轨迹、参考线、车辆状态等
2、参考线信息reference_line_info：包括障碍物列表/决策/ST投影、 自车ST投影等

输出：返回Status::OK()

实现：
1、初始化相关的辅助类：
```cpp
 InitSTBoundsDecider(*frame, reference_line_info);
```
2、类似于扫描线的形式遍历时间轴，确定s的边界：
调用GenerateRegularSTBound()，结果存在定义的三个变量regular_st_bound、regular_vt_bound、st_guide_line
```cpp
 // Sweep the t-axis, and determine the s-boundaries step by step.
STBound regular_st_bound;
STBound regular_vt_bound;
std::vector<std::pair<double, double>> st_guide_line;
Status ret = GenerateRegularSTBound(&regular_st_bound, &regular_vt_bound,//GenerateRegularSTBound()后面小节说明
                                      &st_guide_line);
```

异常处理，如果调用GenerateRegularSTBound()出现错误，打印一个debug信息，返回故障状态码。如果调用GenerateRegularSTBound()生成的结果是空，返回一个error。
```cpp
if (!ret.ok()) {
    ADEBUG << "Cannot generate a regular ST-boundary.";
    return Status(ErrorCode::PLANNING_ERROR, ret.error_message());
  }
  if (regular_st_bound.empty()) {
    const std::string msg = "Generated regular ST-boundary is empty.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
```

创建一个ST图数据类对象的指针，指向类成员reference_line_info_的ST图数据，将上面生成的regular_st_bound、regular_vt_bound设置到StGraphData类对象的行驶数据。
```cpp
  StGraphData* st_graph_data = reference_line_info_->mutable_st_graph_data();
  st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);
```
```cpp
//reference_line_info类成员函数
.../apollo/modules/planning/common/reference_line_info.h
StGraphData* mutable_st_graph_data() { return &st_graph_data_; }
```
```cpp
//StGraphData类成员函数，主要是遍历s_boundary获得s_lower、s_upper、v_obs_lower、v_obs_upper
apollo/modules/planning/common/st_graph_data.cc
bool StGraphData::SetSTDrivableBoundary(
    const std::vector<std::tuple<double, double, double>>& s_boundary,
    const std::vector<std::tuple<double, double, double>>& v_obs_info) {
  if (s_boundary.size() != v_obs_info.size()) {
    return false;//s,v数量不相同则返回运行false
  }
  for (size_t i = 0; i < s_boundary.size(); ++i) {
    auto st_bound_instance = st_drivable_boundary_.add_st_boundary();
    st_bound_instance->set_t(std::get<0>(s_boundary[i]));
    st_bound_instance->set_s_lower(std::get<1>(s_boundary[i]));
    st_bound_instance->set_s_upper(std::get<2>(s_boundary[i]));
    if (std::get<1>(v_obs_info[i]) > -kObsSpeedIgnoreThreshold) { // 这里的kObsSpeedIgnoreThreshold = 100.0
      st_bound_instance->set_v_obs_lower(std::get<1>(v_obs_info[i]));
    }
    if (std::get<2>(v_obs_info[i]) < kObsSpeedIgnoreThreshold) {
      st_bound_instance->set_v_obs_upper(std::get<2>(v_obs_info[i]));
    }
  }
  return true;
}
```
3、记录ST数据用于debug
定义变量all_st_boundaries，调用st_obstacles_processor类对象的成员函数GetAllSTBoundaries()，获得所有障碍物的约束。调用RecordSTGraphDebug()将障碍物ST边界、自车0-7s的s边界和引导线数据存在reference_line_info里的debug里的planning_data里的st_graph里。
```cpp
//unordered_map<std::string, STBoundary>类型
STObstaclesProcessor::GetAllSTBoundaries() {
  return obs_id_to_st_boundary_;
}
```
```cpp
auto all_st_boundaries = st_obstacles_processor_.GetAllSTBoundaries();//
  std::vector<STBoundary> st_boundaries;
  for (const auto& st_boundary : all_st_boundaries) {
    st_boundaries.push_back(st_boundary.second);
  }
  ADEBUG << "Total ST boundaries = " << st_boundaries.size();
  STGraphDebug* st_graph_debug = reference_line_info->mutable_debug()
                                     ->mutable_planning_data()
                                     ->add_st_graph();
  RecordSTGraphDebug(st_boundaries, regular_st_bound, st_guide_line,
                     st_graph_debug);

```
## 3.3 InitSTBoundsDecider()
功能：初始化相关类st_obstacles_processor、st_guide_line、st_driving_limits，主要是st_obstacles_processor。将相关障碍物投影到ST图上。

输入：Frame、reference_line_info

输出：返回Status::OK()

实现：
1、初始化st_obstacles_processor类，将相关障碍物投影ST图：
构造指针指向参考线信息里的path_data路径数据和path_decision路径决策
```cpp
const PathData& path_data = reference_line_info->path_data();
PathDecision* path_decision = reference_line_info->path_decision();
```

将路径数据path_data、路径决策path_decision、路径的总长度、task参数配置的总时间7.0s、injector的历史信息，作为入参调用st_obstacles_processor类的Init()，将path_decisio作为入参调用MapObstaclesToSTBoundaries(）
```cpp
auto time1 = std::chrono::system_clock::now();
  st_obstacles_processor_.Init(path_data.discretized_path().Length(),
                               st_bounds_config_.total_time(), path_data,
                               path_decision, injector_->history());
  st_obstacles_processor_.MapObstaclesToSTBoundaries(path_decision);//后面小节说明
  auto time2 = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = time2 - time1;
  ADEBUG << "Time for ST Obstacles Processing = " << diff.count() * 1000
         << " msec.";
```

2、初始化st_guide_line、st_driving_limits
定义了几个常量
```cpp
static constexpr double desired_speed = 15.0;//期望速度
static constexpr double max_acc = 2.5;//最大加速度
  static constexpr double max_dec = 5.0;//最大减速度
  static constexpr double max_v = desired_speed * 1.5;//最大速度
  ```

引导线的初始化这里分为两种情况，如果路径数据朝着参考轨迹优化，则选择车道跟随混合模式下的
速度规划，这个混合模式是基于学习和基于规则的自车规划。否则以期望速度匀速行驶。is_optimized_towards_trajectory_reference()默认为false
```cpp
if (path_data.is_optimized_towards_trajectory_reference()) {
    st_guide_line_.Init(desired_speed,
                        injector_->learning_based_data()
                            ->learning_data_adc_future_trajectory_points());
  } else {
    st_guide_line_.Init(desired_speed);
  }
```
```cpp
 st_driving_limits_.Init(max_acc, max_dec, max_v,
                          frame.PlanningStartPoint().v());
```
## 3.4 GenerateRegularSTBound()
功能：生成常规的ST边界

输入：st_bound、vt_bound、st-guide-line

输出：返回Status::OK()

实现：
1、初始化ST边界
遍历ST图的时间轴，将每个离散时间点的上下边界初始数据max()塞入st_bound、vt_bound，初始数据分别是double类型的很大的正数和负数。
```cpp
for (double curr_t = 0.0; curr_t <= st_bounds_config_.total_time();  //  total_time()=7.0s,定义的路径\modules\planning\conf\planning_config.pb.txt
       curr_t += kSTBoundsDeciderResolution) {    //kSTBoundsDeciderResolution=0.1s
    st_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
    vt_bound->emplace_back(curr_t, std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::max());
  }
  ```

  2、获取驾驶限制边界driving_limits_bound
  定义五个变量,两个三维元组
```cpp
double t, s_lower, s_upper, lower_obs_v, upper_obs_v;
    std::tie(t, s_lower, s_upper) = st_bound->at(i);
    std::tie(t, lower_obs_v, upper_obs_v) = vt_bound->at(i);
   ```
   调用 STDrivingLimits::GetVehicleDynamicsLimits(),获得最大加减速度所能达到的s边界
  ```cpp
  auto driving_limits_bound = st_driving_limits_.GetVehicleDynamicsLimits(t);  //后面小节说明
    s_lower = std::fmax(s_lower, driving_limits_bound.first);
    s_upper = std::fmin(s_upper, driving_limits_bound.second);
    ADEBUG << "Bounds for s due to driving limits are "
           << "s_upper = " << s_upper << ", s_lower = " << s_lower;
  ```
  3、获取障碍物边界
定义了 available_s_bounds和available_obs_decisions两个向量，存储每个离散时间点可能的约束和相对应的决策。调用st_obstacles_processor类的成员函数GetSBoundsFromDecisions，获取自车所有可能的s边界，一个obs_decision对应一个s_bounds，如果出现错误就报错。
```cpp
std::vector<std::pair<double, double>> available_s_bounds;
    std::vector<ObsDecSet> available_obs_decisions;
    if (!st_obstacles_processor_.GetSBoundsFromDecisions(//后面详细介绍
            t, &available_s_bounds, &available_obs_decisions)) {
      const std::string msg =
          "Failed to find a proper boundary due to obstacles.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
   ```
   将上面的t、s边界、决策组成一个新数据结构，定义为可选择的选项available_choices,并调用RemoveInvalidDecisions()去除不符合driving_limit的决策。
   ```cpp
   std::vector<std::pair<STBoundPoint, ObsDecSet>> available_choices;
    ADEBUG << "Available choices are:";
    for (int j = 0; j < static_cast<int>(available_s_bounds.size()); ++j) {
      ADEBUG << "  (" << available_s_bounds[j].first << ", "
             << available_s_bounds[j].second << ")";
      available_choices.emplace_back(
          std::make_tuple(0.0, available_s_bounds[j].first,
                          available_s_bounds[j].second),
          available_obs_decisions[j]);
    }
    RemoveInvalidDecisions(driving_limits_bound, &available_choices);
   ```
如果available_choices为空就报错，并用RankDecisions()对决策排序，选择最靠前的决策。如果s_lower比top_choice还要小，说明被ST图下方障碍物限制了，置标志位，s_lower更新。如果s_upper比top_choice还要大，说明被ST图上方障碍物限制了，置标志位，s_upper更新。
```cpp
//如果t随时间变化且小于7.0s,则返回s,t,否则按照匀速行驶返回s,t
apollo/modules/planning/tasks/deciders/st_bounds_decider/st_guide_line.cc
double STGuideLine::GetGuideSFromT(double t) {
  common::SpeedPoint speed_point;
  if (t < guideline_speed_data_.TotalTime() &&
      guideline_speed_data_.EvaluateByTime(t, &speed_point)) {
    s0_ = speed_point.s();
    t0_ = t;
    return speed_point.s();
  }
  return s0_ + (t - t0_) * v0_;
}
```
```cpp
if (!available_choices.empty()) {
      ADEBUG << "One decision needs to be made among "
             << available_choices.size() << " choices.";
      double guide_line_s = st_guide_line_.GetGuideSFromT(t);
      st_guide_line->emplace_back(t, guide_line_s);
      RankDecisions(guide_line_s, driving_limits_bound, &available_choices);
      // Select the top decision.
      auto top_choice_s_range = available_choices.front().first;
      bool is_limited_by_upper_obs = false;
      bool is_limited_by_lower_obs = false;
      if (s_lower < std::get<1>(top_choice_s_range)) {
        s_lower = std::get<1>(top_choice_s_range);
        is_limited_by_lower_obs = true;
      }
      if (s_upper > std::get<2>(top_choice_s_range)) {
        s_upper = std::get<2>(top_choice_s_range);
        is_limited_by_upper_obs = true;
      }
   ```
   另外对没有决策的障碍物,调用st_obstacles_processor类的成员函数SetObstacleDecision设置决策
   ```cpp
   auto top_choice_decision = available_choices.front().second;
      st_obstacles_processor_.SetObstacleDecision(top_choice_decision);
   ```
   ```cpp
   //如果自车的决策是停车或让车，则赋给BoundaryType::YIELD，如果是超车则赋给BoundaryType::OVERTAKE
   void STObstaclesProcessor::SetObstacleDecision(
    const std::string& obs_id, const ObjectDecisionType& obs_decision) {
  obs_id_to_decision_[obs_id] = obs_decision;
  ObjectStatus object_status;
  object_status.mutable_motion_type()->mutable_dynamic();
  if (obs_decision.has_yield() || obs_decision.has_stop()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::YIELD);
    object_status.mutable_decision_type()->mutable_yield();
  } else if (obs_decision.has_overtake()) {
    obs_id_to_st_boundary_[obs_id].SetBoundaryType(
        STBoundary::BoundaryType::OVERTAKE);
    object_status.mutable_decision_type()->mutable_overtake();
  }
  history_->mutable_history_status()->SetObjectStatus(obs_id, object_status);
}
```
   调用GetLimitingSpeedInfo()获取新旧设置决策的障碍物对自车每个时刻的s上下界限制，然后更新st_driving_limits和 st_guide_line_，如果执行失败就报error。
  ```cpp
   std::pair<double, double> limiting_speed_info;
      if (st_obstacles_processor_.GetLimitingSpeedInfo(t,
                                                       &limiting_speed_info)) {
        st_driving_limits_.UpdateBlockingInfo(
            t, s_lower, limiting_speed_info.first, s_upper,
            limiting_speed_info.second);
        st_guide_line_.UpdateBlockingInfo(t, s_lower, true); //true是更新下方障碍物的限制
        st_guide_line_.UpdateBlockingInfo(t, s_upper, false);//false是更新上方障碍物的限制
        if (is_limited_by_lower_obs) {
          lower_obs_v = limiting_speed_info.first;
        }
        if (is_limited_by_upper_obs) {
          upper_obs_v = limiting_speed_info.second;
        }
      }
    } else {
      const std::string msg = "No valid st-boundary exists.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }

    // Update into st_bound
    st_bound->at(i) = std::make_tuple(t, s_lower, s_upper);
    vt_bound->at(i) = std::make_tuple(t, lower_obs_v, upper_obs_v);
  }
  ```

## 3.5 RemoveInvalidDecisions()
功能：移除无效的决策函数

输入：驾驶限制driving_limit、可选决策available_choices

实现：available_choices的数据结构是std::vector<std::pair<STBoundPoint, ObsDecSet>>，一组决策对应一个STBoundPoint。
while循环遍历决策组，如果s_lower大于运动加速度能到达的最大s或者s_upper小于运动加速度能到达的最小s，则跟最后一个元素swap()后，vector弹出该决策。
```cpp
size_t i = 0;
while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
```

## 3.6 RankDecisions()
功能：对一组决策进行排序

输入：s_guide_line、driving_limit、available_choices

实现：

```cpp
 bool has_swaps = true;//默认has_swap为真，进入排序的while循环赋false，有两种排序原则，发生swap()时赋true重新进入while循环
  while (has_swaps) {
    has_swaps = false;
    for (int i = 0; i < static_cast<int>(available_choices->size()) - 1; ++i) {... }//两种交换原则
    }
 ```
定义两个空间room：A,B，分别有A_s_lower，A_s_upper，B_s_lower，B_s_upper，初始化为0
两个空间为同一时刻的不同决策决定的不同空间。
```cpp
double A_s_lower = 0.0;
      double A_s_upper = 0.0;
      std::tie(std::ignore, A_s_lower, A_s_upper) =
          available_choices->at(i).first;
      double B_s_lower = 0.0;
      double B_s_upper = 0.0;
      std::tie(std::ignore, B_s_lower, B_s_upper) =
          available_choices->at(i + 1).first;
   ```
   ![在这里插入图片描述](https://img-blog.csdnimg.cn/a60b5ca4793144e9928d08fea66a15f8.jpeg#pic_center=15*15)


排序的第一种原则是两个空间中有一个3m的话，空间大的排前面
```cpp
double A_room = std::fmin(driving_limit.second, A_s_upper) -
                      std::fmax(driving_limit.first, A_s_lower);
      double B_room = std::fmin(driving_limit.second, B_s_upper) -
                      std::fmax(driving_limit.first, B_s_lower);
      if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {// kSTPassableThreshold = 3.0;\modules\planning\tasks\deciders\st_bounds_decider\st_bounds_decider.h
        if (A_room < B_room) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor larger room.";
        }
        continue;
      }
```
排序的第二种原则是两个空间都大于3m，包含引导线的排前面
```cpp
bool A_contains_guideline =
          A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
      bool B_contains_guideline =
          B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
      if (A_contains_guideline != B_contains_guideline) {
        if (!A_contains_guideline) {
          swap(available_choices->at(i + 1), available_choices->at(i));
          has_swaps = true;
          ADEBUG << "Swapping to favor overlapping with guide-line.";
        }
        continue;
              }
```

## 3.7 RecordSTGraphDebug()
功能：存储debug信息，主要是
1、所有相关障碍物的ST边界：boundary_debug、point_debug、
2、自车t时刻的上下界：point_debug
3、引导线：speed_point

输入：st_graph_data、st_bound、 st_guide_line、st_graph_debug

实现：
异常处理，不能记录或者没有st_graph_debug信息则打印一个信息

```cpp
  if (!FLAGS_enable_record_debug || !st_graph_debug) {
    ADEBUG << "Skip record debug info";
    return;
  }
```
定义变量boundary_debug存储障碍物决策的信息
```cpp
for (const auto& boundary : st_graph_data) {
    auto boundary_debug = st_graph_debug->add_boundary();
    boundary_debug->set_name(boundary.id());
    if (boundary.boundary_type() == STBoundary::BoundaryType::YIELD) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_YIELD);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = YIELD";
    } else if (boundary.boundary_type() == STBoundary::BoundaryType::OVERTAKE) {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_OVERTAKE);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = OVERTAKE";
    } else {
      boundary_debug->set_type(StGraphBoundaryDebug::ST_BOUNDARY_TYPE_UNKNOWN);
      ADEBUG << "Obstacle ID = " << boundary.id() << ", decision = UNKNOWN";
    }
```

定义 变量point_debug，存储障碍物的s
```cpp
for (const auto& point : boundary.points()) {
      auto point_debug = boundary_debug->add_point();
      point_debug->set_t(point.x());
      point_debug->set_s(point.y());
    }


```
获取自车每个时刻不同决策下s的上下限
```cpp
 auto boundary_debug = st_graph_debug->add_boundary();
  boundary_debug->set_name("Generated ST-Boundary");
  boundary_debug->set_type(
      StGraphBoundaryDebug::ST_BOUNDARY_TYPE_DRIVABLE_REGION);
  for (const auto& st_bound_pt : st_bound) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_lower = 0.0;
    std::tie(t, s_lower, std::ignore) = st_bound_pt;
    point_debug->set_t(t);
    point_debug->set_s(s_lower);
    ADEBUG << "(" << t << ", " << s_lower << ")";
  }
  for (int i = static_cast<int>(st_bound.size()) - 1; i >= 0; --i) {
    auto point_debug = boundary_debug->add_point();
    double t = 0.0;
    double s_upper = 0.0;
    std::tie(t, std::ignore, s_upper) = st_bound[i];
    point_debug->set_t(t);
    point_debug->set_s(s_upper);
    ADEBUG << "(" << t << ", " << s_upper << ")";
  }
```
定义speed_point存储st_guide_line的s
```cpp
for (const auto& st_points : st_guide_line) {
    auto* speed_point = st_graph_debug->add_speed_profile();
    speed_point->set_t(st_points.first);
    speed_point->set_s(st_points.second);
  }
```

## 3.8 STObstaclesProcessor::MapObstaclesToSTBoundaries()
功能：path_decision里的障碍物列表的所有障碍物投影到ST图

输入：path_decision

输出：返回Status::OK()

实现：
对传入的参数进行检查判断，有道路决策是否为空，规划时间是否小于0，规划距离是否小于0，离散化的尺寸是否小于1
```cpp
if (path_decision == nullptr) {
    const std::string msg = "path_decision is nullptr";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_time_ < 0.0) {
    const std::string msg = "Negative planning time.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (planning_distance_ < 0.0) {
    const std::string msg = "Negative planning distance.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  if (path_data_.discretized_path().size() <= 1) {
    const std::string msg = "Number of path points is too few.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  obs_id_to_st_boundary_.clear();
```
自车低路权段的预处理,如果自车路径点正在离开正向车道或者正在离开逆向车道，并且起始点是低路权状态，则自车低路权段部分对象adc_low_road_right_segments_塞入路径点数据，此s处将置位false定为高路权。如果自车正在车道行驶，并且起始点是低路权，则此s处将置位true定为低路权。
```cpp
 bool is_adc_low_road_right_beginning = true; 
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
    double path_pt_s = 0.0;
    PathData::PathPointType path_pt_type;
    std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
    if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
        path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      if (is_adc_low_road_right_beginning) {
        adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
        is_adc_low_road_right_beginning = false;
      } else {
        adc_low_road_right_segments_.back().second = path_pt_s;
      }
    } else if (path_pt_type == PathData::PathPointType::IN_LANE) {
      if (!is_adc_low_road_right_beginning) {
        is_adc_low_road_right_beginning = true;
      }
    }
  }
 ```
 遍历每个障碍物，检查是否有决策，没有则报错
```cpp
std::unordered_set<std::string> non_ignore_obstacles;
  std::tuple<std::string, STBoundary, Obstacle*> closest_stop_obstacle;
  std::get<0>(closest_stop_obstacle) = "NULL";
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    // Sanity checks.
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (obs_ptr == nullptr) {
      const std::string msg = "Null obstacle pointer.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
```
调用ComputeObstacleSTBoundary()处理每个障碍物并获得ST边界，然后调用CreateInstanceAccurate（）获得lower_points, upper_points上下界，并和障碍物id一起赋给boundary 和alternative_boundary。
```cpp
//如果上下界点的数量不一样，或者只有0/1个,则返回空。否则遍历便捷点，将上下边界点的s,t向量放在STBoundary中返回。
/apollo/modules/planning/common/speed/st_boundary.cc
STBoundary STBoundary::CreateInstanceAccurate(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs, true);
}
```
```cpp

    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    bool is_caution_obstacle = false;//初始化是否为警告型障碍物
    double obs_caution_end_t = 0.0;
    if (!ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,//后面小节说明
                                   &is_caution_obstacle, &obs_caution_end_t)) {
      // Obstacle doesn't appear on ST-Graph.
      continue;
    }
    auto boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
    boundary.set_id(obs_ptr->Id());
    if (is_caution_obstacle) {
      boundary.set_obstacle_road_right_ending_t(obs_caution_end_t);//幅值障碍物在ST图消失的的时间，按STObstaclesProcessor::ComputeObstacleSTBoundary（）中说明，静态obs是7.0s，动态obs是预测的轨迹点相对时间
    }
    // Update the trimmed obstacle into alternative st-bound storage
    // for later uses.
    //对Boundary进行裁剪，将非低路权的lower_points和upper_points删掉，将alternative_boundary赋给obs_id_to_alternative_st_boundary_。
    while (lower_points.size() > 2 &&
           lower_points.back().t() > obs_caution_end_t) {
      lower_points.pop_back();
    }
    while (upper_points.size() > 2 &&
           upper_points.back().t() > obs_caution_end_t) {
      upper_points.pop_back();
    }
    auto alternative_boundary =
        STBoundary::CreateInstanceAccurate(lower_points, upper_points);
    alternative_boundary.set_id(obs_ptr->Id());
    obs_id_to_alternative_st_boundary_[obs_ptr->Id()] = alternative_boundary;
    ADEBUG << "Obstacle " << obs_ptr->Id()
           << " has an alternative st-boundary with "
           << lower_points.size() + upper_points.size() << " points.";
           

```
处理禁停区之外的所以障碍物，判断障碍物轨迹是否为空，以此将障碍物分为静态和动态,如果是为最近的静态障碍物则记录，如果动态障碍物在自车后面并且在非危险路段，则忽视该障碍物，否则记录下来。
```cpp
// Process all other obstacles than Keep-Clear zone.
    if (obs_ptr->Trajectory().trajectory_point().empty()) {
      // Obstacle is static.
      if (std::get<0>(closest_stop_obstacle) == "NULL" ||
          std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
              boundary.bottom_left_point().s()) {
        // If this static obstacle is closer for ADC to stop, record it.
        closest_stop_obstacle =
            std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
      }
    } else {
      // Obstacle is dynamic.
      if (boundary.bottom_left_point().s() - adc_path_init_s_ <
              kSIgnoreThreshold &&
          boundary.bottom_left_point().t() > kTIgnoreThreshold) {
        // Ignore obstacles that are behind.
        // TODO(jiacheng): don't ignore if ADC is in dangerous segments.
        continue;
      }
      obs_id_to_st_boundary_[obs_ptr->Id()] = boundary;
      obs_ptr->set_path_st_boundary(boundary);
      non_ignore_obstacles.insert(obs_ptr->Id());
      ADEBUG << "Adding " << obs_ptr->Id() << " into the ST-graph.";
    }
  }
  ```
candidate_clear_zones_存储所有的禁停区，遍历所有的禁停区，当禁停区和静态障碍物有重叠，静态障碍物停在了禁停区内，则更新最近的障碍物信息为禁停区边界。并将最近的静态障碍物也存入obs_id_to_st_boundary
```cpp
if (std::get<0>(closest_stop_obstacle) != "NULL") {
    std::string closest_stop_obs_id;
    STBoundary closest_stop_obs_boundary;
    Obstacle* closest_stop_obs_ptr;
    std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
             closest_stop_obs_ptr) = closest_stop_obstacle;
    ADEBUG << "Closest obstacle ID = " << closest_stop_obs_id;
    // Go through all Keep-Clear zones, and see if there is an even closer
    // stop fence due to them.
    if (!closest_stop_obs_ptr->IsVirtual()) {
      for (const auto& clear_zone : candidate_clear_zones_) {
        const auto& clear_zone_boundary = std::get<1>(clear_zone);
        if (closest_stop_obs_boundary.min_s() >= clear_zone_boundary.min_s() &&//障碍物停在了禁停区
            closest_stop_obs_boundary.min_s() <= clear_zone_boundary.max_s()) {
          std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
                   closest_stop_obs_ptr) = clear_zone;
          ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
          break;
        }
      }
    }
    obs_id_to_st_boundary_[closest_stop_obs_id] = closest_stop_obs_boundary;
    closest_stop_obs_ptr->set_path_st_boundary(closest_stop_obs_boundary);
    non_ignore_obstacles.insert(closest_stop_obs_id);
    ADEBUG << "Adding " << closest_stop_obs_ptr->Id() << " into the ST-graph.";
    ADEBUG << "min_s = " << closest_stop_obs_boundary.min_s();
  }
  ```
  对不在ST内的障碍物贴ignore标签
  ```cpp
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (non_ignore_obstacles.count(obs_ptr->Id()) == 0) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      if (!obs_ptr->HasLongitudinalDecision()) {
        obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                         ignore_decision);
      }
      if (!obs_ptr->HasLateralDecision()) {
        obs_ptr->AddLateralDecision("st_obstacle_processor", ignore_decision);
      }
    }
  }
  ```
  存储每个障碍物的时间边界，并按升序排序
  ```cpp
  for (const auto& it : obs_id_to_st_boundary_) {
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);
  }
  // Sort the edges.
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });
```
## 3.9 STObstaclesProcessor::ComputeObstacleSTBoundary（）
功能：计算障碍物的ST边界

输入：obstacle、lower_point、upper_points、is_caution_obstacle、obs_caution_end_t

实现：
判断障碍物是静态还是动态,，如果障碍物轨迹点为空，则是静态障碍物，并调用GetOverlappingS（）检查自车和障碍物是否有碰撞。将自车s的上下界更新，is_caution_obstacle置为true，obs_caution_end_t障碍物告警结束时间赋为规划时间，即7.0s
```cpp
if (obs_trajectory.trajectory_point().empty()) {
    // Processing a static obstacle.
    // Sanity checks.
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
    // Get the overlapping s between ADC path and obstacle's perception box.
    const Box2d& obs_box = obstacle.PerceptionBoundingBox();
    std::pair<double, double> overlapping_s;
    if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,//GetOverlapS后面小节说明
                        &overlapping_s)) {
      lower_points->emplace_back(overlapping_s.first, 0.0);
      lower_points->emplace_back(overlapping_s.first, planning_time_);
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);
    }
    *is_caution_obstacle = true;
    *obs_caution_end_t = planning_time_;
  } 
  ```
如果障碍物轨迹不为空，则为动态障碍物。同样调用GetOverlappingS（）检查自车和障碍物是否有碰撞检查自车和障碍物是否有碰撞。将自车s的上下界更新。判断碰撞的上下界是否在自车低路权段，如果是，is_caution_obstacle置为true，并且obs_caution_end_t赋为障碍物轨迹点的相对时间。
```cpp
else {
    // Processing a dynamic obstacle.
    // Go through every occurrence of the obstacle at all timesteps, and
    // figure out the overlapping s-max and s-min one by one.
    bool is_obs_first_traj_pt = true;
    for (const auto& obs_traj_pt : obs_trajectory.trajectory_point()) {
      // TODO(jiacheng): Currently, if the obstacle overlaps with ADC at
      // disjoint segments (happens very rarely), we merge them into one.
      // In the future, this could be considered in greater details rather
      // than being approximated.
      const Box2d& obs_box = obstacle.GetBoundingBox(obs_traj_pt);
      ADEBUG << obs_box.DebugString();
      std::pair<double, double> overlapping_s;
      if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                          &overlapping_s)) {
        ADEBUG << "Obstacle instance is overlapping with ADC path.";
        lower_points->emplace_back(overlapping_s.first,
                                   obs_traj_pt.relative_time());
        upper_points->emplace_back(overlapping_s.second,
                                   obs_traj_pt.relative_time());
        if (is_obs_first_traj_pt) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *is_caution_obstacle = true;
          }
        }
        if ((*is_caution_obstacle)) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *obs_caution_end_t = obs_traj_pt.relative_time();
          }
        }
      }
      is_obs_first_traj_pt = false;
    }
    if (lower_points->size() == 1) {
      lower_points->emplace_back(lower_points->front().s(),
                                 lower_points->front().t() + 0.1);
      upper_points->emplace_back(upper_points->front().s(),
                                 upper_points->front().t() + 0.1);
    }
  }

  return (!lower_points->empty() && !upper_points->empty());
}
```

##  3.10 st_obstacle_processor_.GetSBoundsFromDecisions()
功能：自车所有可能s边界和对应的决策列表

输入：available_s_bounds、available_obs_decisions

输出：bool值

实现：
![在这里插入图片描述](https://img-blog.csdnimg.cn/8d2064f9fcc14af490f789ef8c40dd50.jpeg#pic_center)

定义一个ObsTEdge类型的向量 new_t_edges，这个类型包括is_starting_t, t, s_min, s_max, obs_id，这个t_edge在InitSTBoundsDecider()运行时调用MapObstaclesToSTBoundarie完成的,while循环将t时刻之前的所有obs_t_edges塞入new_t_edges
```cpp
// Gather any possible change in st-boundary situations.
  ADEBUG << "There are " << obs_t_edges_.size() << " t-edges.";
  std::vector<ObsTEdge> new_t_edges;
  while (obs_t_edges_idx_ < static_cast<int>(obs_t_edges_.size()) &&
         std::get<1>(obs_t_edges_[obs_t_edges_idx_]) <= t) {
    if (std::get<0>(obs_t_edges_[obs_t_edges_idx_]) == 0 &&
        std::get<1>(obs_t_edges_[obs_t_edges_idx_]) == t) {
      break;
    }
    ADEBUG << "Seeing a new t-edge at t = "
           << std::get<1>(obs_t_edges_[obs_t_edges_idx_]);
    new_t_edges.push_back(obs_t_edges_[obs_t_edges_idx_]);
    ++obs_t_edges_idx_;
  }
```
t时刻之前的STBoundary都移除掉，这些edge是不是起始edge并且已经决策过的障碍物。
```cpp
for (const auto& obs_t_edge : new_t_edges) {
    if (std::get<0>(obs_t_edge) == 0) {
      ADEBUG << "Obstacle id: " << std::get<4>(obs_t_edge)
             << " is leaving st-graph.";
      if (obs_id_to_decision_.count(std::get<4>(obs_t_edge)) != 0) {
        obs_id_to_decision_.erase(std::get<4>(obs_t_edge));
      }
    }
  }
  ```
创建一个向量存放被移除的障碍物,遍历obs_id_to_decision，如果要被overtake的障碍物其被注意时间只持续到t时刻前0.5s，则将其id塞入obs_id_to_remove
```cpp
std::vector<std::string> obs_id_to_remove;
  for (const auto& obs_id_to_decision_pair : obs_id_to_decision_) {
    auto obs_id = obs_id_to_decision_pair.first;
    auto obs_decision = obs_id_to_decision_pair.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    if (obs_decision.has_overtake() &&
        obs_st_boundary.min_t() <= t - kOvertakenObsCautionTime &&
        obs_st_boundary.obstacle_road_right_ending_t() <=
            t - kOvertakenObsCautionTime) {
      obs_id_to_remove.push_back(obs_id_to_decision_pair.first);
    }
  }
  for (const auto& obs_id : obs_id_to_remove) {
    obs_id_to_decision_.erase(obs_id);
    // Change the displayed st-boundary to the alternative one:
    if (obs_id_to_alternative_st_boundary_.count(obs_id) > 0) {
      Obstacle* obs_ptr = path_decision_->Find(obs_id);
      obs_id_to_st_boundary_[obs_id] =
          obs_id_to_alternative_st_boundary_[obs_id];
      obs_id_to_st_boundary_[obs_id].SetBoundaryType(
          STBoundary::BoundaryType::OVERTAKE);
      obs_ptr->set_path_st_boundary(obs_id_to_alternative_st_boundary_[obs_id]);
    }
  }
 ```
遍历obs_id_to_decision，获取t时刻的障碍物上下界obs_max,obs_min,如果障碍物有yield或stop决策,那么自车t时刻的上界不能超obs_min；如果对障碍物有overtake决策，那么自车在t时刻的下界不能低于obs_max。
```cpp
 double s_min = 0.0;
  double s_max = planning_distance_;
  for (auto it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      s_max = std::fmin(s_max, obs_s_min);
    } else if (it.second.has_overtake()) {
      s_min = std::fmax(s_min, obs_s_max);
    }
  }
  if (s_min > s_max) {
    return false;
  }
  ADEBUG << "S-boundary based on existing decisions = (" << s_min << ", "
         << s_max << ")";
  ```
  对没有决策的障碍物，判断要采取yield还是overtake，否则就加入模糊障碍物
  ```cpp
  std::vector<ObsTEdge> ambiguous_t_edges;
  for (auto obs_t_edge : new_t_edges) {
    ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
           << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
           << std::get<3>(obs_t_edge) << "]";
    if (std::get<0>(obs_t_edge) == 1) {
      if (std::get<2>(obs_t_edge) >= s_max) {
        ADEBUG << "  Apparently, it should be yielded.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_max);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::YIELD);
      } else if (std::get<3>(obs_t_edge) <= s_min) {
        ADEBUG << "  Apparently, it should be overtaken.";
        obs_id_to_decision_[std::get<4>(obs_t_edge)] =
            DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                      std::get<3>(obs_t_edge), s_min);
        obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
            STBoundary::BoundaryType::OVERTAKE);
      } else {
        ADEBUG << "  It should be further analyzed.";
        ambiguous_t_edges.push_back(obs_t_edge);
      }
    }
  }
  ```
对于模糊障碍物，调用FindSGaps()获得每个edge的gap和相应的决策，调用DetermineObstacleDecision（）,如果gap的中点在obs_s_min之下就yield，如果gap的中点在obs_s_max之上就overtake,然后将决策放入available_obs_decisions。
```cpp
// For ambiguous ones, enumerate all decisions and corresponding bounds.
  auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
  if (s_gaps.empty()) {
    return false;
  }
  for (auto s_gap : s_gaps) {
    available_s_bounds->push_back(s_gap);
    std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
    for (auto obs_t_edge : ambiguous_t_edges) {
      std::string obs_id = std::get<4>(obs_t_edge);
      double obs_s_min = std::get<2>(obs_t_edge);
      double obs_s_max = std::get<3>(obs_t_edge);
      obs_decisions.emplace_back(
          obs_id,
          DetermineObstacleDecision(obs_s_min, obs_s_max,
                                    (s_gap.first + s_gap.second) / 2.0));
    }
    available_obs_decisions->push_back(obs_decisions);
  }

```

## 3.11STObstaclesProcessor::GetLimitingSpeedInfo()
功能：生成障碍物对自车的上下界速度限制

输入：t、limiting_speed_info

输出：返回bool值

实现：
判断obs_id_to_decision_是否为空，否则返回false。遍历obs_id_to_decision_，获取障碍物id和相应决策的映射表，初始化变量obs_s_min、obs_s_max、obs_ds_lower、obs_ds_upper，调用GetBoundarySRange（）和GetBoundarySlopes（）得到上面变量数据。
如果停车或者让车，障碍物下边界obs_s_min比s_max小，即自车和障碍物重叠，则自车上界最多obs_s_min，且速度小于障碍物下界速度。(上方障碍物)
如果超车，障碍物下边界obs_s_max比s_min大，即自车和障碍物重叠，则自车下界最小obs_s_max，且速度小于障碍物下界速度。（下方障碍物）
![在这里插入图片描述](https://img-blog.csdnimg.cn/54c16d2769be475a981dc7b4c4f91b2e.jpeg#pic_center)


```cpp
bool STObstaclesProcessor::GetLimitingSpeedInfo(
    double t, std::pair<double, double>* const limiting_speed_info) {
  if (obs_id_to_decision_.empty()) {
    // If no obstacle, then no speed limits.
    return false;
  }

  double s_min = 0.0;
  double s_max = planning_distance_;
  for (auto it : obs_id_to_decision_) {
    auto obs_id = it.first;
    auto obs_decision = it.second;
    auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
    double obs_s_min = 0.0;
    double obs_s_max = 0.0;
    obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
    double obs_ds_lower = 0.0;
    double obs_ds_upper = 0.0;
    obs_st_boundary.GetBoundarySlopes(t, &obs_ds_upper, &obs_ds_lower);
    if (obs_decision.has_yield() || obs_decision.has_stop()) {
      if (obs_s_min <= s_max) {
        s_max = obs_s_min;
        limiting_speed_info->second = obs_ds_lower;
      }
    } else if (it.second.has_overtake()) {
      if (obs_s_max >= s_min) {
        s_min = obs_s_max;
        limiting_speed_info->first = obs_ds_upper;
      }
    }
  }
  return s_min <= s_max;
}
```

## 3.12  STObstaclesProcessor::GetOverlappingS
功能：检测自车是否与障碍物重叠，并获得碰撞的上下界放在overlapping_s

输入：自车路径点adc_path_points、检测距离Box2d& obstacle_instance、自车横向宽度膨胀量adc_l_buffer、碰撞上下界overlapping_s

输出：返回bool值

定义两个点序号pt_before_idx和pt_after_idx，调用 GetSBoundingPathPointIndex（）返回要碰撞的第一个点的序号+/-1，并进行检查序号是否错误。
```cpp
 // Locate the possible range to search in details.
 //GetSBoundingPathPointIndex()后面小节说明
  int pt_before_idx = GetSBoundingPathPointIndex(//pt_before_idx是障碍物在自车前面时，检测的碰撞点序号
      adc_path_points, obstacle_instance, vehicle_param_.front_edge_to_center(),
      true, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index before is " << pt_before_idx;
  int pt_after_idx = GetSBoundingPathPointIndex(//pt_before_idx是障碍物在自车后面时，检测的碰撞点序号
      adc_path_points, obstacle_instance, vehicle_param_.back_edge_to_center(),
      false, 0, static_cast<int>(adc_path_points.size()) - 2);
  ADEBUG << "The index after is " << pt_after_idx;
  if (pt_before_idx == static_cast<int>(adc_path_points.size()) - 2) {
    return false;
  }
  if (pt_after_idx == 0) {
    return false;
  }

  if (pt_before_idx == -1) {
    pt_before_idx = 0;
  }
  if (pt_after_idx == -1) {
    pt_after_idx = static_cast<int>(adc_path_points.size()) - 2;
  }
  if (pt_before_idx >= pt_after_idx) {
    return false;
  }
```
在上面得到的两个序号点之间再进行overlap的搜索，其中调用的是IsADCOverlappingWithObstacle（）二次搜索。该函数将参考点从后轴中心移到自车中心，然后调用Box2d()计算自车边界框，再调用Box2d.HasOverlap在检查自车边界框和障碍物边界框是否有重叠。然后遍历序号点，更新overlapping约束的s上下界。
```cpp
bool STObstaclesProcessor::IsADCOverlappingWithObstacle(
    const PathPoint& adc_path_point, const Box2d& obs_box,
    const double l_buffer) const {
  // Convert reference point from center of rear axis to center of ADC.
  Vec2d ego_center_map_frame((vehicle_param_.front_edge_to_center() -//
                              vehicle_param_.back_edge_to_center()) *
                                 0.5,
                             (vehicle_param_.left_edge_to_center() -
                              vehicle_param_.right_edge_to_center()) *
                                 0.5);
  ego_center_map_frame.SelfRotate(adc_path_point.theta());
  ego_center_map_frame.set_x(ego_center_map_frame.x() + adc_path_point.x());//转换为自车中心点的xy坐标
  ego_center_map_frame.set_y(ego_center_map_frame.y() + adc_path_point.y());

  // Compute the ADC bounding box.
  Box2d adc_box(ego_center_map_frame, adc_path_point.theta(),
                vehicle_param_.length(), vehicle_param_.width() + l_buffer * 2);

  ADEBUG << "    ADC box is: " << adc_box.DebugString();
  ADEBUG << "    Obs box is: " << obs_box.DebugString();

  // Check whether ADC bounding box overlaps with obstacle bounding box.
  return obs_box.HasOverlap(adc_box);
}
```
```cpp
// Detailed searching.
  bool has_overlapping = false;
  for (int i = pt_before_idx; i <= pt_after_idx; ++i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->first = adc_path_points[std::max(i - 1, 0)].s();
      has_overlapping = true;
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  if (!has_overlapping) {
    return false;
  }
  for (int i = pt_after_idx; i >= pt_before_idx; --i) {
    ADEBUG << "At ADC path index = " << i << " :";
    if (IsADCOverlappingWithObstacle(adc_path_points[i], obstacle_instance,
                                     adc_l_buffer)) {
      overlapping_s->second = adc_path_points[i + 1].s();
      ADEBUG << "There is overlapping.";
      break;
    }
  }
  ```

## 3.13  STObstaclesProcessor::GetSBoundingPathPointIndex(）
功能：查找碰撞前后的点序号index

输入：自车轨迹点集adc_path_points、obstacle_instance、s_thresh、is_before、start_idx、end_idx

输出：返回第一个要碰撞的index

实现：二分递归查找第一个碰撞点
![在这里插入图片描述](https://img-blog.csdnimg.cn/4aa3359450aa4d74aff84031489c27b3.jpeg#pic_center)![在这里插入图片描述](https://img-blog.csdnimg.cn/ac807661d55a4c468a4fdf80e41e6c0f.jpeg#pic_center)这里是两个相邻路径点所构成的一对正交向量，x是障碍物和ADC之间向量在切向量的投影


```cpp
//IsPathPointAwayFromObstacle（）某路径点上自车是否在远离障碍物
//用向量计算obs边界框的corners到自车距离在切向量上的分量大小
bool STObstaclesProcessor::IsPathPointAwayFromObstacle(
    const PathPoint& path_point, const PathPoint& direction_point,
    const Box2d& obs_box, const double s_thresh, const bool is_before) {
  Vec2d path_pt(path_point.x(), path_point.y());
  Vec2d dir_pt(direction_point.x(), direction_point.y());
  LineSegment2d path_dir_lineseg(path_pt, dir_pt);//路径切向量
  LineSegment2d normal_line_seg(path_pt, path_dir_lineseg.rotate(M_PI_2));//路径法向量

  auto corner_points = obs_box.GetAllCorners();
  for (const auto& corner_pt : corner_points) {
    Vec2d normal_line_ft_pt;
    normal_line_seg.GetPerpendicularFoot(corner_pt, &normal_line_ft_pt);
    Vec2d path_dir_unit_vec = path_dir_lineseg.unit_direction();
    Vec2d perpendicular_vec = corner_pt - normal_line_ft_pt;
    double corner_pt_s_dist = path_dir_unit_vec.InnerProd(perpendicular_vec);//corner在路径上的投影距离
    if (is_before && corner_pt_s_dist < s_thresh) {//如果障碍物在前，corner_pt_s_dist小于s_thresh，则认为没有远离。这里s_thresh会取center_to_front
      return false;
    }
    if (!is_before && corner_pt_s_dist > -s_thresh) {//如果障碍物在后，绝对值corner_pt_s_dist小于s_thresh，则认为没有远离。这里s_thresh会取center_to_end
      return false;
    }
  }
  return true;
}
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/e488fd0ec4be4005a4a4d77125269ec6.jpeg#pic_center)


当is_before真，障碍物在自车前面， mid_idx取(start_idx + end_idx - 1) / 2 + 1，并且自车远离障碍物，在后半段轨迹点递归查找碰撞点，否则在前半段递归查找碰撞点
当is_before假，障碍物在自车后面， mid_idx取(start_idx + end_idx) / 2，并且自车远离障碍物，在前半段轨迹点递归查找碰撞点，否则在后半段递归查找碰撞点

```cpp
int STObstaclesProcessor::GetSBoundingPathPointIndex(
    const std::vector<PathPoint>& adc_path_points,
    const Box2d& obstacle_instance, const double s_thresh, const bool is_before,
    const int start_idx, const int end_idx) {
  if (start_idx == end_idx) {
    if (IsPathPointAwayFromObstacle(adc_path_points[start_idx],
                                    adc_path_points[start_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return start_idx;
    } else {
      return -1;
    }
  }

  if (is_before) {
    int mid_idx = (start_idx + end_idx - 1) / 2 + 1;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx, end_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx - 1);
    }
  } else {
    int mid_idx = (start_idx + end_idx) / 2;
    if (IsPathPointAwayFromObstacle(adc_path_points[mid_idx],
                                    adc_path_points[mid_idx + 1],
                                    obstacle_instance, s_thresh, is_before)) {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, start_idx,
                                        mid_idx);
    } else {
      return GetSBoundingPathPointIndex(adc_path_points, obstacle_instance,
                                        s_thresh, is_before, mid_idx + 1,
                                        end_idx);
    }
  }
}
```



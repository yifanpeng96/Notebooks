# Apollo 9.0
## flow
PlanningComponent::Init()

    DependencyInjector() = default;
        
    PlanningBase(const std::shared_ptr<DependencyInjector>& injector) {}
    OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
        : PlanningBase(injector) {}
    
    OnLanePlanning::Init(const PlanningConfig& config)
        
        PlanningBase::Init(const PlanningConfig& config)

        ReferenceLineProvider(
            const common::VehicleStateProvider* vehicle_state_provider,
            const ReferenceLineConfig* reference_line_config,
            const std::shared_ptr<relative_map::MapMsg>& relative_map = nullptr)
        
        ReferenceLineProvider::Start()

        PlanningBase::LoadPlanner()

        TrafficDecider::Init(const std::shared_ptr<DependencyInjector> &injector)

        PublicRoadPlanner::Init(
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_path)


PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate)
    
    OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb)

        VehicleStateProvider::Update(
            const localization::LocalizationEstimate &localization,
            const canbus::Chassis &chassis)
        
        ReferenceLineProvider::UpdateVehicleState(
            const VehicleState &vehicle_state)

        std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
            const canbus::Chassis& vehicle_chassis, const VehicleState& vehicle_state,
            const double current_timestamp, const double planning_cycle_time,
            const size_t preserved_points_num, const bool replan_by_offset,
            const PublishableTrajectory* prev_trajectory, std::string* replan_reason)
        
        EgoInfo::Update(const common::TrajectoryPoint& start_point,
                     const common::VehicleState& vehicle_state)
        
        Frame::Frame(uint32_t sequence_num, const LocalView &local_view,
             const common::TrajectoryPoint &planning_start_point,
             const common::VehicleState &vehicle_state)
        
        ReferenceLineProvider::GetReferenceLines(
            std::list<ReferenceLine> *reference_lines,
            std::list<hdmap::RouteSegments> *segments)

        Frame::Init(
            const common::VehicleStateProvider *vehicle_state_provider,
            const std::list<ReferenceLine> &reference_lines,
            const std::list<hdmap::RouteSegments> &segments,
            const std::vector<routing::LaneWaypoint> &future_route_waypoints,
            const EgoInfo *ego_info)

        TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info)
        
        PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory)
    

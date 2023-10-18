#include <memory>
#include <filesystem>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <rclcpp_action/client.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


#include "shr_utils/geometry.hpp"
#include <shr_parameters.hpp>
#include <shr_plan/actions.hpp>

#include <shr_plan/world_state_converter.hpp>

using namespace pddl_lib;


Domain load_domain(const std::string &domain_file) {
    std::string domain_str;
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
    std::filesystem::path domain_file_path = pkg_dir / "pddl" / domain_file;

    std::ifstream domain_file_stream(domain_file_path.c_str());
    std::stringstream ss;
    ss << domain_file_stream.rdbuf();
    domain_str = ss.str();
    return parse_domain(domain_str).value();
}

std::optional<std::string> getPlan(const std::string &domain, const std::string &problem) {
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);

    {
        std::ofstream domainFile("/tmp/plan_solver/domain.pddl");
        domainFile << domain;
        std::ofstream problemFile("/tmp/plan_solver/problem.pddl");
        problemFile << problem;
    }
    std::string cmd = "ros2 run plan_solver_py plan_solver -o /tmp/plan_solver/domain.pddl -f /tmp/plan_solver/problem.pddl > /dev/null";
    std::system(cmd.c_str());

    std::ifstream file("/tmp/plan_solver/bt.xml");
    if (!file) {
        return {};
    }
    std::stringstream ss;
    ss << file.rdbuf();
    return ss.str();
}


class UpdatePredicatesImpl : public UpdatePredicates {
    std::shared_ptr<WorldStateListener> world_state_converter;
    std::mutex mtx;


public:
    UpdatePredicatesImpl(std::shared_ptr<WorldStateListener> &world_state_converter) : world_state_converter(
            world_state_converter) {
    }

    void concurrent_update() {
        std::lock_guard<std::mutex> lock_guard(mtx);
        update();
    }

    TRUTH_VALUE success(TRUTH_VALUE val) const override {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE robot_at(TRUTH_VALUE val, Landmark lm) const override {
        if (world_state_converter->check_robot_at_loc(lm)) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE person_at(TRUTH_VALUE val, Time t, Person p, Landmark lm) const override {
        if (val == TRUTH_VALUE::UNKNOWN || t != "t1") {
            return val;
        }
        if (world_state_converter->check_person_at_loc(lm)) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE person_currently_at(TRUTH_VALUE val,Person p, Landmark lm) const override {
        if (val == TRUTH_VALUE::UNKNOWN) {
            return val;
        }
        if (world_state_converter->check_person_at_loc(lm)) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE person_at_door(TRUTH_VALUE val, WanderingProtocol w) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(w, params)) {
            auto msg = world_state_converter->get_world_state_msg();
            if (world_state_converter->check_person_at_loc(
                    params.pddl.WanderingProtocols.door_location[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }


    TRUTH_VALUE person_outside(TRUTH_VALUE val, WanderingProtocol w) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(w, params)) {
            auto msg = world_state_converter->get_world_state_msg();
            if (world_state_converter->check_person_at_loc(
                    params.pddl.WanderingProtocols.outside_location[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE time_to_eat(TRUTH_VALUE val, FoodProtocol f) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(f, params)) {
            if (compare_time(params.pddl.FoodProtocols.eat_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE already_ate(TRUTH_VALUE val, FoodProtocol f) const override {
        if (val == TRUTH_VALUE::TRUE) {
            return TRUTH_VALUE::TRUE;
        }
        //TODO this is not right. It should check if the current time window corresponds to f
        if (world_state_converter->get_world_state_msg()->person_eating == 1) {
            return TRUTH_VALUE::TRUE;
        }
        return val;
    }

    TRUTH_VALUE too_late_to_go_outside(TRUTH_VALUE val, WanderingProtocol w) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(w, params)) {
            if (compare_time(params.pddl.WanderingProtocols.too_late_to_leave_time[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol m) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (compare_time(params.pddl.MedicineProtocols.take_medication_time[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE person_taking_medicine(TRUTH_VALUE val, Time t) const override {
        if (val == TRUTH_VALUE::TRUE) {
            return TRUTH_VALUE::TRUE;
        }
        if (world_state_converter->get_world_state_msg()->person_taking_medicine == 1) {
            return TRUTH_VALUE::TRUE;
        }
        return val;
    }

    TRUTH_VALUE already_took_medicine(TRUTH_VALUE val, MedicineProtocol m) const override {
        if (val == TRUTH_VALUE::TRUE) {
            return TRUTH_VALUE::TRUE;
        }
        if (world_state_converter->get_world_state_msg()->person_taking_medicine == 1) {
            return TRUTH_VALUE::TRUE;
        }
        return val;
    }


private:
    bool compare_time(std::string param_time) const {
        auto msg = world_state_converter->get_world_state_msg();
        auto time = msg->time;
        std::stringstream ss(param_time);
        std::string time_1;
        std::string time_2;
        std::getline(ss, time_1, '/');
        std::getline(ss, time_2);

        auto time_1_secs = get_seconds(time_1);
        auto time_2_secs = get_seconds(time_2);

        const int second_in_day = 60 * 60 * 24;
        double clock_distance = fmod((time_2_secs - time_1_secs + second_in_day), second_in_day);
        double time_to_check_normalized = fmod((time.sec - time_1_secs + second_in_day), second_in_day);

        return time_to_check_normalized <= clock_distance;
    }

};

class HighLevelBT {
private:
    std::mutex mtx;
    bool terminate_thread_;
    UpdatePredicatesImpl &updater_;
    Domain domain_;
    BT::BehaviorTreeFactory factory_;
public:

    HighLevelBT(UpdatePredicatesImpl &updater) : updater_{updater} {
        terminate_thread_ = false;
        domain_ = load_domain("high_level_domain.pddl");
        factory_ = create_tree_factory<ProtocolActions>();
    }

    void tick_tree() {
        updater_.concurrent_update();
        auto &kb = KnowledgeBase::getInstance();
        kb.insert_predicate({"priority_1", {}});

        auto problem_str = kb.convert_to_problem(domain_);

        if (auto config = getPlan(domain_.str(), problem_str)) {
            auto tree = factory_.createTreeFromText(config.value());
            BT::NodeStatus res;
            res = tree.tickRoot();
            printf("high level running.. \n");

        }
        kb.erase_predicate({"success", {}});
    }

    void terminate_thread() {
        std::lock_guard<std::mutex> lock(mtx);
        terminate_thread_ = true;
    }

    bool should_terminate_thread() {
        std::lock_guard<std::mutex> lock(mtx);
        return terminate_thread_;
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("shrParameterNode");
    auto param_listener_ = std::make_shared<shr_parameters::ParamListener>(node);

    auto world_state_converter = std::make_shared<WorldStateListener>("WorldStatePDDLConverter", param_listener_);
    std::thread thread_1(
            [&world_state_converter]() {
                rclcpp::executors::MultiThreadedExecutor executor;
                executor.add_node(world_state_converter);
                while (!world_state_converter->should_terminate_node()) {
                    executor.spin_some();
                }
            }
    );

    {
        auto [ps, lock] = ProtocolState::getConcurrentInstance();
        ps.world_state_converter = world_state_converter;
        ps.call_client_ = rclcpp_action::create_client<shr_msgs::action::CallRequest>(
                world_state_converter, "make_call");
        while (!ps.call_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(rclcpp::get_logger("planning_controller"), "Waiting for /make_call action server...");
        }
        ps.nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                world_state_converter, "navigate_to_pose_with_localization");
        while (!ps.nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(rclcpp::get_logger("planning_controller"), "Waiting for /navigate_to_pose_with_localization action server...");
        }
        ps.read_action_client_ = rclcpp_action::create_client<shr_msgs::action::ReadScriptRequest>(
                world_state_converter, "read_script");
        while (!ps.read_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(rclcpp::get_logger("planning_controller"), "Waiting for /read_script action server...");
        }
        ps.video_action_client_ = rclcpp_action::create_client<shr_msgs::action::PlayVideoRequest>(
                world_state_converter, "play_video");
        while (!ps.read_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(rclcpp::get_logger("planning_controller"), "Waiting for /play_video action server...");
        }
    }

    // localize to start navigation and move to home position
    auto [ps, lock] = ProtocolState::getConcurrentInstance();


    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    std::cout << "before starting Btree localize and go home" << std::endl ;
    auto success = std::make_shared<std::atomic<int>>(-1);
    navigation_goal_.pose.header.frame_id = "map";
    navigation_goal_.pose.header.stamp = ps.world_state_converter->now();
    if (auto transform = ps.world_state_converter->get_tf("map", "home")) {
        navigation_goal_.pose.pose.orientation = transform.value().transform.rotation;
        navigation_goal_.pose.pose.position.x = transform.value().transform.translation.x;
        navigation_goal_.pose.pose.position.y = transform.value().transform.translation.y;
        navigation_goal_.pose.pose.position.z = transform.value().transform.translation.z;
    }
    // for blocking
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [&success](
            const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) {
        *success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    };
    ps.nav_client_->async_send_goal(navigation_goal_, send_goal_options);
    auto tmp = ps.active_protocol;
    // blocking
    while (*success == -1) {
        if (!(tmp == ps.active_protocol)) {
            ps.nav_client_->async_cancel_all_goals();
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    instantiate_high_level_problem();

    auto &kb = KnowledgeBase::getInstance();
    UpdatePredicatesImpl updater(world_state_converter);
    // run high level behavior tree on its own thread
    HighLevelBT high_level_bt(updater);
    std::thread thread_2(
            [&high_level_bt]() {
                while (!high_level_bt.should_terminate_thread()) {
                    high_level_bt.tick_tree();
                    rclcpp::sleep_for(std::chrono::milliseconds(2000));
                }
            }
    );

    // run the domains
    BT::BehaviorTreeFactory factory = create_tree_factory<ProtocolActions>();

    while (true) {
        rclcpp::sleep_for(std::chrono::seconds(1));

        std::string active_domain;
        auto protocol = ProtocolState::getActiveProtocol();
        if (!protocol.name.empty()) {
            active_domain = "low_level_domain.pddl";
            updater.concurrent_update();
            auto domain = load_domain(active_domain);
            auto problem_str = kb.convert_to_problem(domain);
            // it would be nice if the planner ran on a fix interval checking to see if the new plan is different.
            // That way, the plan could be aborted since the new one is more optimal.
            if (auto config = getPlan(domain.str(), problem_str)) {
                auto tree = factory.createTreeFromText(config.value());
                BT::NodeStatus res;
                if (ProtocolState::getActiveProtocol() == protocol) {
                    res = tree.tickRoot();
                    printf("low level running.. \n");
                }
                kb.erase_predicate({"success", {}});
            } else {
                std::cout << "failed to find low level plan!.\n";
                kb.insert_predicate({"low_level_failed", {}});
            }
        }
    }

    world_state_converter->terminate_node();
    high_level_bt.terminate_thread();
    thread_1.join();
    thread_2.join();
    rclcpp::shutdown();

    return 0;
}




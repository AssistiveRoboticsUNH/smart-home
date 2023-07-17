#include <memory>
#include <filesystem>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <rclcpp_action/client.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
//#include "cff_plan_solver/cff_plan_solver.hpp"

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
    return {};
}

class HighLevelBT {
private:
    std::mutex mtx;
    bool terminate_thread_;
    const UpdatePredicates &updater_;
    Domain domain_;
    BT::BehaviorTreeFactory factory_;
public:

    HighLevelBT(const UpdatePredicates &updater) : updater_{updater} {
        terminate_thread_ = false;
        domain_ = load_domain("high_level_domain.pddl");
        factory_ = create_tree_factory<ProtocolActions>();
    }

    void tick_tree() {
        updater_.update();
        auto &kb = KnowledgeBase::getInstance();
        kb.knownPredicates.concurrent_insert({"priority_1", {}});

        auto problem_str = kb.convert_to_problem(domain_);

        if (auto config = getPlan(domain_.str(), problem_str)) {
            auto tree = factory_.createTreeFromText(config.value());
            BT::NodeStatus res;
            do {
                res = tree.tickRoot();
                printf("running.. \n");
            } while (res == BT::NodeStatus::RUNNING);
        }
        kb.knownPredicates.concurrent_erase({"success", {}});
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

class UpdatePredicatesImpl : public UpdatePredicates {
    std::shared_ptr<WorldStateListener> world_state_converter;


public:
    UpdatePredicatesImpl(std::shared_ptr<WorldStateListener> &world_state_converter) : world_state_converter(
            world_state_converter) {

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
        auto msg = world_state_converter->get_world_state_msg();
        auto params = world_state_converter->get_params();
        if (world_state_converter->check_person_at_loc(lm) &&
            lm != params.pddl.WanderingProtocols.outside_location[0]) {// TODO this is hack
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


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("shrParameterNode");
    auto param_listener_ = std::make_shared<shr_parameters::ParamListener>(node);

    auto world_state_converter = std::make_shared<WorldStateListener>("WorldStatePDDLConverter", param_listener_);
    std::thread thread_1(
            [&world_state_converter]() {
                while (!world_state_converter->should_terminate_node()) {
                    rclcpp::spin_some(world_state_converter->get_node_base_interface());
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
                }
            }
    );

    ProtocolState::getInstance().world_state_converter = world_state_converter;
    ProtocolState::getInstance().call_client_ = rclcpp_action::create_client<shr_msgs::action::CallRequest>(
            world_state_converter, "make_call");
    ProtocolState::getInstance().nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            world_state_converter, "navigate_to_pose");
    ProtocolState::getInstance().read_action_client_ = rclcpp_action::create_client<shr_msgs::action::ReadScriptRequest>(
            world_state_converter, "read_script");


    while (world_state_converter->get_world_state_msg() == nullptr) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(node->get_name()), "waiting for first world state message");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    auto params = param_listener_->get_params();
    auto &kb = KnowledgeBase::getInstance();
    for (const auto &landmark: params.pddl.instances.Landmarks) {
        kb.objects.concurrent_insert({landmark, "Landmark"});
    }
    for (const auto &person: params.pddl.instances.Persons) {
        kb.objects.concurrent_insert({person, "Person"});
    }
    for (const auto &robot: params.pddl.instances.Robots) {
        kb.objects.concurrent_insert({robot, "Robot"});
    }

    for (const auto &meal: params.pddl.FoodProtocols.instances) {
        kb.objects.concurrent_insert({meal, "FoodProtocol"});
        InstantiatedParameter inst = {meal, "FoodProtocol"};
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.concurrent_insert({"remind_food_succeeded", {inst}});
        kb.unknownPredicates.concurrent_insert({"remind_food_succeeded2", {inst}});

    }
    for (const auto &protocol: params.pddl.MedicineProtocols.instances) {
        kb.objects.concurrent_insert({protocol, "MedicineProtocol"});
        InstantiatedParameter inst = {protocol, "MedicineProtocol"};
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.concurrent_insert({"notify_automated_succeeded", {inst}});
        kb.unknownPredicates.concurrent_insert({"notify_recorded_succeeded", {inst}});

    }
    for (const auto &protocol: params.pddl.FallProtocols.instances) {
        kb.objects.concurrent_insert({protocol, "FallProtocol"});
    }
    for (const auto &protocol: params.pddl.WanderingProtocols.instances) {
        kb.objects.concurrent_insert({protocol, "WanderingProtocol"});
    }

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
        InstantiatedParameter protocol = get_active_protocol();
        if (protocol.type == "FallProtocol") {
            active_domain = "fall_domain.pddl";
        } else if (protocol.type == "FoodProtocol") {
            active_domain = "food_domain.pddl";
        } else if (protocol.type == "MedicineProtocol") {
            active_domain = "medicine_domain.pddl";
        } else if (protocol.type == "WanderingProtocol") {
            active_domain = "wandering_domain.pddl";
        } else {
            // no work to do
            continue;
        }

        // updater.update(); needs to be called, but that is done by the high level bt
        auto domain = load_domain(active_domain);
        auto problem_str = kb.convert_to_problem(domain);
        if (auto config = getPlan(domain.str(), problem_str)) {
            auto tree = factory.createTreeFromText(config.value());
            BT::NodeStatus res;
            try {
                do {
                    res = tree.tickRoot();
                    printf("running.. \n");
                } while (res == BT::NodeStatus::RUNNING);
            } catch (const std::runtime_error &ex) {
                std::cout << ex.what() << std::endl;
                res = BT::NodeStatus::FAILURE;
            }
            kb.knownPredicates.concurrent_erase({"success", {}});

        }


    }

    world_state_converter->terminate_node();
    high_level_bt.terminate_thread();
    thread_1.join();
    thread_2.join();
    rclcpp::shutdown();

    return 0;
}
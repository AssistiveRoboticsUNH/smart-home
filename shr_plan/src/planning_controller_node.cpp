#include <memory>
#include <filesystem>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/bool.hpp"
#include "shr_msgs/action/gather_information_request.hpp"
#include "shr_msgs/msg/midnight_warning_protocol.hpp"
#include "shr_msgs/msg/medicine_reminder_protocol.hpp"
#include "shr_msgs/msg/food_reminder_protocol.hpp"
#include "shr_msgs/msg/world_state.hpp"
#include "shr_msgs/msg/success_protocol.hpp"

#include <rclcpp_action/client.hpp>
#include "shr_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "cff_plan_solver/cff_plan_solver.hpp"

#include <shr_plan_parameters.hpp>
#include <shr_plan/actions.hpp>

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


class WorldStatePDDLConverter : public rclcpp::Node {
private:
    rclcpp::Subscription<shr_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    shr_msgs::msg::WorldState world_state_;
    std::mutex mtx;
    bool terminate_node_;
public:

    WorldStatePDDLConverter(const std::string &node_name, const shr_plan_parameters::Params &params) : rclcpp::Node(
            node_name) {
        terminate_node_ = false;
        world_state_sub_ = create_subscription<shr_msgs::msg::WorldState>(
                params.world_state_topic, 10, [this](const shr_msgs::msg::WorldState::SharedPtr msg) {
                    set_world_state_msg(*msg);
                });
    }

    void terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        terminate_node_ = true;
    }

    bool should_terminate_node() {
        std::lock_guard<std::mutex> lock(mtx);
        return terminate_node_;
    }

    void set_world_state_msg(const shr_msgs::msg::WorldState &msg) {
        std::lock_guard<std::mutex> lock(mtx);
        world_state_ = msg;
    }

    shr_msgs::msg::WorldState get_world_state_msg() {
        std::lock_guard<std::mutex> lock(mtx);
        return world_state_;
    }
};

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
        factory_ = create_tree_factory();
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
    WorldStatePDDLConverter &world_state_converter;

public:
    UpdatePredicatesImpl(WorldStatePDDLConverter &world_state_converter) : world_state_converter(
            world_state_converter) {

    }

    TRUTH_VALUE success(TRUTH_VALUE val) const override {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE medicine_location(TRUTH_VALUE val, MedicineProtocol m, Landmark lm) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.medicine_location == lm && m == "daily") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE robot_at(TRUTH_VALUE val, Robot r, Landmark lm) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.robot_location == lm) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE person_at(TRUTH_VALUE val, Person p, Landmark lm) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.patient_location == lm) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE food_location(TRUTH_VALUE val, FoodProtocol f, Landmark loc) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.eat_location == loc) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE door_location(TRUTH_VALUE val, WonderingProtocol w, Landmark lm) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.door_location == lm) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE person_at_door(TRUTH_VALUE val, WonderingProtocol w) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.patient_location == "door") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE person_outside(TRUTH_VALUE val, WonderingProtocol w) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.patient_location == "outside") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE time_to_eat(TRUTH_VALUE val, FoodProtocol f) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if ((msg.time_to_eat_dinner && f == "dinner") ||
            (msg.time_to_eat_lunch && f == "lunch") ||
            (msg.time_to_eat_breakfast && f == "breakfast")) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE too_late_to_go_outside(TRUTH_VALUE val, WonderingProtocol w) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.too_late_to_leave) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol m) const override {
        auto msg = world_state_converter.get_world_state_msg();
        if (msg.time_to_take_medicine && m == "daily") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("shrParameterNode");
    auto param_listener_ = shr_plan_parameters::ParamListener(node);
    auto params = param_listener_.get_params();


    WorldStatePDDLConverter world_state_converter("WorldStatePDDLConverter", params);
    std::thread thread_1(
            [&world_state_converter]() {
                while (!world_state_converter.should_terminate_node()) {
                    rclcpp::spin_some(world_state_converter.get_node_base_interface());
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
                }
            }
    );

    auto &kb = KnowledgeBase::getInstance();
    for (const auto &landmark: params.pddl_instances.Landmark) {
        kb.objects.concurrent_insert({landmark, "Landmark"});
    }
    for (const auto &person: params.pddl_instances.Person) {
        kb.objects.concurrent_insert({person, "Person"});
    }
    for (const auto &robot: params.pddl_instances.Robot) {
        kb.objects.concurrent_insert({robot, "Robot"});
    }

    for (const auto &meal: params.pddl_instances.FoodProtocols) {
        kb.objects.concurrent_insert({meal, "FoodProtocol"});
        InstantiatedParameter inst = {meal, "FoodProtocol"};
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.concurrent_insert({"remind_food_succeeded", {inst}});
        kb.unknownPredicates.concurrent_insert({"remind_food_succeeded2", {inst}});

    }
    for (const auto &protocol: params.pddl_instances.MedicineProtocols) {
        kb.objects.concurrent_insert({protocol, "MedicineProtocol"});
        InstantiatedParameter inst = {protocol, "MedicineProtocol"};
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.concurrent_insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.concurrent_insert({"notify_automated_succeeded", {inst}});
        kb.unknownPredicates.concurrent_insert({"notify_recorded_succeeded", {inst}});

    }
    for (const auto &protocol: params.pddl_instances.FallProtocols) {
        kb.objects.concurrent_insert({protocol, "FallProtocol"});
    }
    for (const auto &protocol: params.pddl_instances.WonderingProtocols) {
        kb.objects.concurrent_insert({protocol, "WonderingProtocol"});
    }

    UpdatePredicatesImpl updater(world_state_converter);
    // run high level behavior tree on its own thread
    HighLevelBT high_level_bt(updater);
    std::thread thread_2(
            [&high_level_bt]() {
                while (!high_level_bt.should_terminate_thread()) {
                    high_level_bt.tick_tree();
                    rclcpp::sleep_for(std::chrono::milliseconds(800));
                }
            }
    );

    // run the domains
    BT::BehaviorTreeFactory factory = create_tree_factory();


    while (true) {
        rclcpp::sleep_for(std::chrono::seconds(3));

        std::string active_domain;
        InstantiatedParameter protocol = get_active_protocol();
        if (protocol.type == "FallProtocol") {
            active_domain = "fall_domain.pddl";
        } else if (protocol.type == "FoodProtocol") {
            active_domain = "food_domain.pddl";
        } else if (protocol.type == "MedicineProtocol") {
            active_domain = "medicine_domain.pddl";
        } else if (protocol.type == "WonderingProtocol") {
            active_domain = "wondering_domain.pddl";
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

    world_state_converter.terminate_node();
    high_level_bt.terminate_thread();
    thread_1.join();
    thread_2.join();
    rclcpp::shutdown();

    return 0;
}
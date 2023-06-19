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

    TRUTH_VALUE person_at(TRUTH_VALUE val, Person p, Landmark lm) {
        auto msg = get_world_state_msg();
        if (msg.patient_location == lm.value) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE robot_at(TRUTH_VALUE val, Robot r, Landmark lm) {
        auto msg = get_world_state_msg();
        if (msg.robot_location == lm.value) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE medicine_location(TRUTH_VALUE val, MedicineProtocol m, Landmark lm) {
        auto msg = get_world_state_msg();
        if (msg.medicine_location == lm.value && m.value == "daily") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE time_to_eat(TRUTH_VALUE val, FoodProtocol f) {
        auto msg = get_world_state_msg();
        if ((msg.time_to_eat_dinner && f.value == "dinner") ||
            (msg.time_to_eat_lunch && f.value == "lunch") ||
            (msg.time_to_eat_breakfast && f.value == "breakfast")) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol m) {
        auto msg = get_world_state_msg();
        if (msg.time_to_take_medicine && m.value == "daily") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }


    TRUTH_VALUE person_on_ground(TRUTH_VALUE val) {

        return val;
    }

    TRUTH_VALUE too_late_to_go_outside(TRUTH_VALUE val) {
        auto msg = get_world_state_msg();
        if (msg.too_late_to_leave) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE success(TRUTH_VALUE val) {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE priority_1(TRUTH_VALUE val) {
        return TRUTH_VALUE::TRUE;
    }

    TRUTH_VALUE priority_2(TRUTH_VALUE val) {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE priority_3(TRUTH_VALUE val) {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE priority_4(TRUTH_VALUE val) {
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE priority_5(TRUTH_VALUE val) {
        return TRUTH_VALUE::FALSE;
    }


};

Domain load_domain(std::string &domain_file) {
    std::string domain_str;
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
    std::filesystem::path domain_file_path = pkg_dir / "pddl" / domain_file;

    std::ifstream domain_file_stream(domain_file_path.c_str());
    std::stringstream ss;
    ss << domain_file_stream.rdbuf();
    domain_str = ss.str();
    return parse_domain(domain_str).value();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("shrParameterNode");
    auto param_listener_ = shr_plan_parameters::ParamListener(node);
    auto params = param_listener_.get_params();


    WorldStatePDDLConverter world_state_converter("WorldStatePDDLConverter", params);
    std::thread thread(
            [&world_state_converter]() {
                while (!world_state_converter.should_terminate_node()) {
                    rclcpp::spin_some(world_state_converter.get_node_base_interface());
                }
            }
    );

    UpdatePredicates updater;

    updater.set_person_at([&world_state_converter](TRUTH_VALUE val, Person p, Landmark lm) {
        return world_state_converter.person_at(val, p, lm);
    });
    updater.set_robot_at([&world_state_converter](TRUTH_VALUE val, Robot r, Landmark lm) {
        return world_state_converter.robot_at(val, r, lm);
    });
    updater.set_medicine_location([&world_state_converter](TRUTH_VALUE val, MedicineProtocol m, Landmark lm) {
        return world_state_converter.medicine_location(val, m, lm);
    });
    updater.set_time_to_eat(
            [&world_state_converter](TRUTH_VALUE val, FoodProtocol f) {
                return world_state_converter.time_to_eat(val, f);
            });
    updater.set_time_to_take_medicine(
            [&world_state_converter](TRUTH_VALUE val, MedicineProtocol m) {
                return world_state_converter.time_to_take_medicine(val, m);
            });
    updater.set_person_on_ground(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.person_on_ground(val); });
    updater.set_too_late_to_go_outside(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.too_late_to_go_outside(val); });
    updater.set_success(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.success(val); });
    updater.set_priority_1(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_1(val); });
    updater.set_priority_2(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_2(val); });
    updater.set_priority_3(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_3(val); });
    updater.set_priority_4(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_4(val); });
    updater.set_priority_5(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_5(val); });

    auto &kb = KnowledgeBase::getInstance();
    for (const auto &landmark: params.pddl_instances.Landmark) {
        kb.objects.push_back({landmark, "Landmark"});
    }
    for (const auto &person: params.pddl_instances.Person) {
        kb.objects.push_back({person, "Person"});
    }
    for (const auto &robot: params.pddl_instances.Robot) {
        kb.objects.push_back({robot, "Robot"});
    }
    for (const auto &meal: params.pddl_instances.FoodProtocol) {
        kb.objects.push_back({meal, "FoodProtocol"});
    }
    for (const auto &protocol: params.pddl_instances.MedicineProtocol) {
        kb.objects.push_back({protocol, "MedicineProtocol"});
    }
    for (const auto &protocol: params.pddl_instances.FallProtocol) {
        kb.objects.push_back({protocol, "FallProtocol"});
    }
    for (const auto &protocol: params.pddl_instances.WonderingProtocol) {
        kb.objects.push_back({protocol, "WonderingProtocol"});
    }

    InstantiatedParameter med_prot = {params.pddl_instances.MedicineProtocol[0], "MedicineProtocol"};
    kb.unknownPredicates.insert({"guide_to_succeeded_attempt_1", {med_prot}});
    kb.unknownPredicates.insert({"guide_to_succeeded_attempt_2", {med_prot}});
    kb.unknownPredicates.insert({"notify_automated_succeeded", {med_prot}});
    kb.unknownPredicates.insert({"notify_recorded_succeeded", {med_prot}});

    // run the domains
    BT::BehaviorTreeFactory factory = create_tree_factory();
    active_domain = "high_level_domain.pddl";

    while (true) {
        auto domain = load_domain(active_domain);
        updater.update();
        auto problem_str = kb.convert_to_problem(domain);
        if (auto config = getPlan(domain.str(), problem_str)) {
            auto tree = factory.createTreeFromText(config.value());
            while (tree.tickRoot() == BT::NodeStatus::RUNNING) {
                printf("running.. \n");
            }
        }
        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    world_state_converter.terminate_node();
    thread.join();

    rclcpp::shutdown();

    return 0;
}
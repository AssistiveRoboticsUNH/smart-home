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

    TRUTH_VALUE food_location(TRUTH_VALUE val, FoodProtocol m, Landmark lm) {
        auto msg = get_world_state_msg();
        if (msg.eat_location == lm.value) {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE door_location(TRUTH_VALUE val, WonderingProtocol w, Landmark lm) {
        auto msg = get_world_state_msg();
        if (msg.door_location == lm.value) {
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


    TRUTH_VALUE person_on_ground(TRUTH_VALUE val, FallProtocol f) {

        return val;
    }

    TRUTH_VALUE person_outside(TRUTH_VALUE val, WonderingProtocol w) {
        auto msg = get_world_state_msg();
        if (msg.patient_location == "outside") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE person_at_door(TRUTH_VALUE val, WonderingProtocol w) {
        auto msg = get_world_state_msg();
        if (msg.patient_location == "door") {
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE too_late_to_go_outside(TRUTH_VALUE val, WonderingProtocol w) {
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

    updater.register_person_at([&world_state_converter](TRUTH_VALUE val, Person p, Landmark lm) {
        return world_state_converter.person_at(val, p, lm);
    });
    updater.register_robot_at([&world_state_converter](TRUTH_VALUE val, Robot r, Landmark lm) {
        return world_state_converter.robot_at(val, r, lm);
    });
    updater.register_medicine_location([&world_state_converter](TRUTH_VALUE val, MedicineProtocol m, Landmark lm) {
        return world_state_converter.medicine_location(val, m, lm);
    });
    updater.register_time_to_eat(
            [&world_state_converter](TRUTH_VALUE val, FoodProtocol f) {
                return world_state_converter.time_to_eat(val, f);
            });
    updater.register_time_to_take_medicine(
            [&world_state_converter](TRUTH_VALUE val, MedicineProtocol m) {
                return world_state_converter.time_to_take_medicine(val, m);
            });
    updater.register_person_on_ground(
            [&world_state_converter](TRUTH_VALUE val, FallProtocol f) {
                return world_state_converter.person_on_ground(val, f);
            });

    updater.register_person_outside(
            [&world_state_converter](TRUTH_VALUE val, WonderingProtocol w) {
                return world_state_converter.person_outside(val, w);
            });

    updater.register_person_at_door(
            [&world_state_converter](TRUTH_VALUE val, WonderingProtocol w) {
                return world_state_converter.person_at_door(val, w);
            });

    updater.register_too_late_to_go_outside(
            [&world_state_converter](TRUTH_VALUE val, WonderingProtocol w) {
                return world_state_converter.too_late_to_go_outside(val, w);
            });
    updater.register_success(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.success(val); });
    updater.register_priority_1(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_1(val); });
    updater.register_priority_2(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_2(val); });
    updater.register_priority_3(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_3(val); });
    updater.register_priority_4(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_4(val); });
    updater.register_priority_5(
            [&world_state_converter](TRUTH_VALUE val) { return world_state_converter.priority_5(val); });
    updater.register_food_location([&world_state_converter](TRUTH_VALUE val, FoodProtocol f, Landmark lm) {
        return world_state_converter.food_location(val, f, lm);
    });
    updater.register_door_location([&world_state_converter](TRUTH_VALUE val, WonderingProtocol w, Landmark lm) {
        return world_state_converter.door_location(val, w, lm);
    });

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
    for (const auto &meal: params.pddl_instances.FoodProtocols) {
        kb.objects.push_back({meal, "FoodProtocol"});
        InstantiatedParameter inst = {meal, "FoodProtocol"};
        kb.unknownPredicates.insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.insert({"remind_food_succeeded", {inst}});
        kb.unknownPredicates.insert({"remind_food_succeeded2", {inst}});

    }
    for (const auto &protocol: params.pddl_instances.MedicineProtocols) {
        kb.objects.push_back({protocol, "MedicineProtocol"});
        InstantiatedParameter inst = {protocol, "MedicineProtocol"};
        kb.unknownPredicates.insert({"guide_to_succeeded_attempt_1", {inst}});
        kb.unknownPredicates.insert({"guide_to_succeeded_attempt_2", {inst}});
        kb.unknownPredicates.insert({"notify_automated_succeeded", {inst}});
        kb.unknownPredicates.insert({"notify_recorded_succeeded", {inst}});

    }
    for (const auto &protocol: params.pddl_instances.FallProtocols) {
        kb.objects.push_back({protocol, "FallProtocol"});
    }
    for (const auto &protocol: params.pddl_instances.WonderingProtocols) {
        kb.objects.push_back({protocol, "WonderingProtocol"});
    }
    // run the domains
    BT::BehaviorTreeFactory factory = create_tree_factory();
    active_domain = "high_level_domain.pddl";

    while (true) {
        auto domain = load_domain(active_domain);
        updater.update();
        auto problem_str = kb.convert_to_problem(domain);
        if (auto config = getPlan(domain.str(), problem_str)) {
            auto tree = factory.createTreeFromText(config.value());
            BT::NodeStatus res;
            do {
                res = tree.tickRoot();
                printf("running.. \n");
            } while (res == BT::NodeStatus::RUNNING);

            if (res == BT::NodeStatus::FAILURE) {
                active_domain = "high_level_domain.pddl";
            }
        }

        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    world_state_converter.terminate_node();
    thread.join();
    rclcpp::shutdown();

    return 0;
}
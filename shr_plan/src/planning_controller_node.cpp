#include <memory>
#include <filesystem>
#include <fstream>


#include <ctime>
#include <unordered_map>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>

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
#include <shr_plan/intersection_helpers.hpp>
#include <cstdlib>  // for getenv

using namespace pddl_lib;

Domain load_domain(const std::string &domain_file) {
    std::string domain_str;
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
    std::cout <<  "pkg_dir: "<< pkg_dir << std::endl;
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
    // std::string path = homeDir + "/planner_data";
    std::string homeDir = std::getenv("HOME");
//    std::cout << "homeDir: " << homeDir << std::endl;
    std::string path = homeDir + "/planner_data";
    {
        std::ofstream domainFile(path + "/plan_solver/domain.pddl");
        domainFile << domain;
        std::ofstream problemFile(path + "/plan_solver/problem.pddl");
        problemFile << problem;
    }

//    std::string cmd = "ros2 run plan_solver_py plan_solver -o /home/olagh48652/planner_data/plan_solver/domain.pddl -f /home/olagh48652/planner_data/plan_solver/problem.pddl > /dev/null";
    std::string cmd = "ros2 run plan_solver_py plan_solver -o ";
    cmd += homeDir;
    cmd += "/planner_data/plan_solver/domain.pddl -f ";
    cmd += homeDir;
    cmd += "/planner_data/plan_solver/problem.pddl > /dev/null";
//    std::cout << "Command: " << cmd << std::endl;
    std::system(cmd.c_str());

    std::ifstream file(path + "/plan_solver/bt.xml");
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
        auto &kb = KnowledgeBase::getInstance();
        bool pred_started = kb.find_predicate({"started", {}});
        
        if (pred_started){
            // RCLCPP_WARN(rclcpp::get_logger("pred_started inside AT"), "robot at true  ");
            if (world_state_converter->check_robot_at_loc(lm)) {
                return TRUTH_VALUE::TRUE;
            } else {
                return TRUTH_VALUE::FALSE;
            }
         } else {
            if (lm == "home"){
                // RCLCPP_WARN(rclcpp::get_logger("ROBOT AT"), "robot at true  ");
                std::cout << "robot at true " << lm << std::endl;

                return TRUTH_VALUE::TRUE;
            }
         }
         return TRUTH_VALUE::FALSE;
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

    // In high level found
    TRUTH_VALUE person_currently_at(TRUTH_VALUE val, Person p, Landmark lm) const override {
        if (val == TRUTH_VALUE::UNKNOWN) {
            return val;
        }
        if (world_state_converter->check_person_at_loc(lm)) {
            std::cout << "person_At " << lm << std::endl;
            return TRUTH_VALUE::TRUE;
        } else {
            return TRUTH_VALUE::FALSE;
        }
    }

    TRUTH_VALUE time_to_take_medicine(TRUTH_VALUE val, MedicineProtocol m) const override {
        auto params = world_state_converter->get_params();

        // Debugging: Print all available medicine protocols
        // for (const auto &protocol : params.pddl.MedicineProtocols.instances) {
        //     RCLCPP_INFO(rclcpp::get_logger("time_to_take_medicine"), "🔍 Available protocol: %s", protocol.c_str());
        // }

        if (auto index = get_inst_index(m, params)) {
            std::string time_range = params.pddl.MedicineProtocols.take_medication_times[index.value()];
            // RCLCPP_INFO(rclcpp::get_logger("time_to_take_medicine"),
            //             "Checking MedicineProtocol: %s | Time Range: %s",
            //             m.c_str(), time_range.c_str());

            if (compare_time(time_range)) {
                // RCLCPP_INFO(rclcpp::get_logger("time_to_take_medicine"),
                //             "✅ TIME MATCH! Triggering protocol for: %s", m.c_str());
                return TRUTH_VALUE::TRUE;
            } // else {
            // //     RCLCPP_INFO(rclcpp::get_logger("time_to_take_medicine"),
            // //                 "❌ Time does not match for: %s", m.c_str());
            // }
         } // else {
        //     RCLCPP_ERROR(rclcpp::get_logger("time_to_take_medicine"),
        //                  "⚠️ Could not find index for protocol: %s", m.c_str());
        // }

        return TRUTH_VALUE::FALSE;
    }


    TRUTH_VALUE time_for_gym_reminder(TRUTH_VALUE val, GymReminderProtocol m) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (compare_time(params.pddl.GymReminderProtocols.gym_reminder_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE time_for_medicine_refill_reminder(TRUTH_VALUE val, MedicineRefillReminderProtocol m) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (compare_time(params.pddl.MedicineRefillReminderProtocols.medicine_refill_reminder_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE time_for_medicine_pharmacy_reminder(TRUTH_VALUE val, MedicineRefillPharmacyReminderProtocol m) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (compare_time(params.pddl.MedicineRefillPharmacyReminderProtocols.medicine_refill_pharmacy_reminder_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE time_for_walking_reminder(TRUTH_VALUE val, WalkingProtocol m) const override {
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (compare_time(params.pddl.WalkingProtocols.walking_reminder_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
        }
        return TRUTH_VALUE::FALSE;
    }

    TRUTH_VALUE good_weather(TRUTH_VALUE val, WalkingProtocol w) const override {
        // RCLCPP_INFO(rclcpp::get_logger("WeatherDebug"), "🌤️ Entering good_weather function for WalkingProtocol: ");

        auto world_state_msg = world_state_converter->get_world_state_msg();
        if (!world_state_msg) {
            // RCLCPP_ERROR(rclcpp::get_logger("WeatherDebug"), "❌ Error: world_state_msg is NULL! Returning UNKNOWN.");
            return TRUTH_VALUE::UNKNOWN;
        }

        int weather_status = world_state_msg->good_weather;
        // RCLCPP_INFO(rclcpp::get_logger("WeatherDebug"), "🔍 Current good_weather value: %d", weather_status);

        if (weather_status == 1) {
            // RCLCPP_INFO(rclcpp::get_logger("WeatherDebug"), "✅ Weather is GOOD for WalkingProtocol: ");
            return TRUTH_VALUE::TRUE;
        }

        // RCLCPP_WARN(rclcpp::get_logger("WeatherDebug"), "⚠️ Weather is NOT good for WalkingProtocol: ");
        return TRUTH_VALUE::FALSE;
    }

    // low level
    TRUTH_VALUE person_taking_medicine(TRUTH_VALUE val, Time t) const override {
        if (val == TRUTH_VALUE::TRUE) {
            return TRUTH_VALUE::TRUE;
        }
        if (world_state_converter->get_world_state_msg()->person_taking_medicine == 1) {
            return TRUTH_VALUE::TRUE;
        }
        return val;
    }

    // TODO: Check if this needs to be sandwiched between time window
    TRUTH_VALUE already_took_medicine(TRUTH_VALUE val, MedicineProtocol m) const override {
        if (val == TRUTH_VALUE::TRUE) {
            return TRUTH_VALUE::TRUE;
        }
        auto params = world_state_converter->get_params();
        if (auto index = get_inst_index(m, params)) {
            if (world_state_converter->get_world_state_msg()->person_taking_medicine == 1 &&
                compare_time(params.pddl.MedicineProtocols.take_medication_times[index.value()])) {
                return TRUTH_VALUE::TRUE;
            }
            return val;
        }
    }



private:

    int get_current_weekday() const{
        time_t now = time(0);
        tm *ltm = localtime(&now);
        return ltm->tm_wday;
    }

    std::vector<int> parse_days(const std::string& days_str) const{
        static const std::unordered_map<std::string, int> days_map = {
            {"Sunday", 0}, {"Monday", 1}, {"Tuesday", 2}, {"Wednesday", 3},
            {"Thursday", 4}, {"Friday", 5}, {"Saturday", 6}
        };
    
        std::vector<int> days;
        std::stringstream ss(days_str);
        std::string day;
        
        while (std::getline(ss, day, ',')) {
            auto it = days_map.find(day);
            if (it != days_map.end()) {
                days.push_back(it->second);
            }
        }
    
        return days;
    }

    

        // ✅ Compares time and day to trigger protocols
    bool compare_time(std::string param_time) const {
        auto msg = world_state_converter->get_world_state_msg();
        auto time = msg->time;

        std::stringstream ss(param_time);
        std::string days_str, time_1, time_2;
        
        std::getline(ss, days_str, ' ');  // Extract the days string (e.g., "Monday,Wednesday" or "Everyday")
        std::getline(ss, time_1, '/');    // Extract start time (e.g., "15h00m0s")
        std::getline(ss, time_2);         // Extract end time (e.g., "16h00m0s")

        int current_weekday = get_current_weekday();
        
        // ✅ If "Everyday", skip day validation
        if (days_str != "Everyday") {
            std::vector<int> scheduled_days = parse_days(days_str);
            if (std::find(scheduled_days.begin(), scheduled_days.end(), current_weekday) == scheduled_days.end()) {
                // RCLCPP_WARN(rclcpp::get_logger("compare_time"), 
                //             "❌ Today (%d) is not in the scheduled days (%s). Skipping trigger.",
                //             current_weekday, days_str.c_str());
                return false;
            }
        }

        auto time_1_secs = get_seconds(time_1);
        auto time_2_secs = get_seconds(time_2);
        auto current_time_secs = time.sec;

        const int second_in_day = 60 * 60 * 24;
        double clock_distance = fmod((time_2_secs - time_1_secs + second_in_day), second_in_day);
        double time_to_check_normalized = fmod((current_time_secs - time_1_secs + second_in_day), second_in_day);

        // 🔍 Debugging Log
        // RCLCPP_INFO(rclcpp::get_logger("compare_time"), 
                    // "🕒 Checking Time: %s | Today: %d | Days: %s | Start: %d | End: %d | Current: %d | Normalized: %d | Distance: %d",
                    // param_time.c_str(), current_weekday, days_str.c_str(), time_1_secs, time_2_secs, 
                    // current_time_secs, (int)time_to_check_normalized, (int)clock_distance);

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
            // printf("high level running.. \n");

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
        lock.Lock();
        ps.world_state_converter = world_state_converter;

        ps.nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
                world_state_converter, "navigate_to_pose");
//        while (!ps.nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("navigate_to_pose"), "Waiting for /navigate_to_pose action server...");
//        }
        ps.read_action_client_ = rclcpp_action::create_client<shr_msgs::action::ReadScriptRequest>(
                world_state_converter, "read_script");
//        while (!ps.read_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("read_script"), "Waiting for /read_script action server...");
//        }
        ps.audio_action_client_ = rclcpp_action::create_client<shr_msgs::action::PlayAudioRequest>(
                world_state_converter, "play_audio");
//        while (!ps.audio_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("play_audio"), "Waiting for /play_audio action server...");
//        }
        ps.docking_ = rclcpp_action::create_client<shr_msgs::action::DockingRequest>(
                world_state_converter, "docking");
//        while (!ps.docking_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("docking"), "Waiting for /docking action server...");
//        }
        ps.undocking_ = rclcpp_action::create_client<shr_msgs::action::DockingRequest>(
                world_state_converter, "undocking");
//        while (!ps.undocking_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("undocking"), "Waiting for /undocking action server...");
//        }
        ps.localize_ = rclcpp_action::create_client<shr_msgs::action::LocalizeRequest>(
                world_state_converter, "localize");
//        while (!ps.localize_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("localize"), "Waiting for /localize action server...");
//        }
        ps.call_client_ = rclcpp_action::create_client<shr_msgs::action::CallRequest>(
                world_state_converter, "make_call");
//        while (!ps.call_client_->wait_for_action_server(std::chrono::seconds(5))) {
//            RCLCPP_INFO(rclcpp::get_logger("make_call"), "Waiting for /make_call action server...");
//        }

        // 🔴 Ensure the action client exists
        ps.voice_action_client_ = rclcpp_action::create_client<shr_msgs::action::QuestionResponseRequest>(
                world_state_converter, "question_response_action");

        // while (!ps.voice_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        //     RCLCPP_INFO(rclcpp::get_logger("voice"), "Waiting for /question_response_action server...");
        // }

        lock.UnLock();
    }

    // localize to start navigation and move to home position
    auto [ps, lock] = ProtocolState::getConcurrentInstance();

    instantiate_high_level_problem();

    auto &kb = KnowledgeBase::getInstance();
    UpdatePredicatesImpl updater(world_state_converter);
    // run high level behavior tree on its own thread
    HighLevelBT high_level_bt(updater);
    // std::thread thread_2(
    //         [&high_level_bt]() {
    //             while (!high_level_bt.should_terminate_thread()) {
    //                 high_level_bt.tick_tree();
    //                 rclcpp::sleep_for(std::chrono::milliseconds(2000));
    //             }
    //         }
    // );


    std::thread thread_2(
        [&high_level_bt, &ps]() {
            bool first_run = true;  // Flag to ensure the check runs only once
    
            while (!high_level_bt.should_terminate_thread()) {
                if (first_run) {
                    auto start_time = std::chrono::steady_clock::now();
                    while ((std::chrono::steady_clock::now() - start_time < std::chrono::seconds(20)) &&
                           ps.world_state_converter->get_world_state_msg()->robot_charging != 1) {
                        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Check every 500ms
                    }
                    first_run = false;  // Ensure the check doesn't run again
                }
    
                high_level_bt.tick_tree();
                rclcpp::sleep_for(std::chrono::milliseconds(2000));  // Regular tick interval
            }
        }
    );

    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("shr_plan");
    std::filesystem::path outputFile = pkg_dir / "include" / "shr_plan" / "intersection.txt";

    std::cout << "outputFile: "  << outputFile.c_str() << std::endl;
    auto predicates = read_predicates_from_file(outputFile.c_str());

    // Print the predicates
    for (const auto& [first, second, third] : predicates) {
        std::cout << "Keyword: " << first << ", ProtocolName: " << second << ", ProtocolType: " << third << std::endl;
        InstantiatedParameter active_protocol = {second, third};
        InstantiatedPredicate pred{first, {active_protocol}};
        kb.insert_predicate(pred);
    }

    // run the domains
    BT::BehaviorTreeFactory factory = create_tree_factory<ProtocolActions>();

    // TODO: set landmark to dockign station at boot
    // (robot_at ?lmr - Landmark)
    InstantiatedParameter landmark = {"home", "Landmark"};
    InstantiatedPredicate pred_rob_at{"robot_at", {landmark}};
    kb.insert_predicate(pred_rob_at);

    std::cout << "insert_predicate robot_at at home !.\n";

    // if navigation is on set the started predicate to true
    if (ps.nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
            kb.insert_predicate({"started", {}});
            std::cout << "insert_predicate started !.\n";
    }

    

    while (true) {
        rclcpp::sleep_for(std::chrono::seconds(1));

        std::string active_domain;
        auto protocol = ProtocolState::getActiveProtocol();

        if (!protocol.name.empty() && !ProtocolState::isRobotInUse()) {
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




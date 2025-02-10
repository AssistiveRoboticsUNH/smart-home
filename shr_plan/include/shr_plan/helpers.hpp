//
// Created by marzan on 2/10/25.
//

#pragma once

namespace pddl_lib {

    int get_seconds(const std::string &time_str) {
        std::stringstream ss(time_str);
        std::string seconds;
        std::string minute;
        std::string hour;
        std::getline(ss, hour, 'h');
        std::getline(ss, minute, 'm');
        std::getline(ss, seconds, 's');
        return std::stoi(hour) * 60 * 60 + std::stoi(minute) * 60 + std::stoi(seconds);
    }

    std::optional<long> get_inst_index(MedicineProtocol m, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.MedicineProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), m);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(GymReminderProtocol m, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.GymReminderProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), m);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(MedicineRefillReminderProtocol m, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.MedicineRefillReminderProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), m);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(MedicineRefillPharmacyReminderProtocol m, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.MedicineRefillPharmacyReminderProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), m);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(InstantiatedParameter inst, const shr_parameters::Params &params) {
        if (inst.type == "MedicineProtocol") {
            return get_inst_index((MedicineProtocol) inst.name, params);
        } else if (inst.type == "GymReminderProtocol") {
            return get_inst_index((GymReminderProtocol) inst.name, params);
        }
        else if (inst.type == "MedicineRefillReminderProtocol") {
            return get_inst_index((MedicineRefillReminderProtocol) inst.name, params);
        }else if (inst.type == "MedicineRefillPharmacyReminderProtocols") {
            return get_inst_index((MedicineRefillPharmacyReminderProtocol) inst.name, params);
        }
        return {};
    }

    std::string
    replace_token(const std::string &protocol_content, const std::string &token, const std::string &new_token) {
        if (token == new_token) {
            return protocol_content;
        }
        auto check_string = [token](const std::string &protocol_content, size_t index) {
            if (token.size() + index > protocol_content.size()) {
                return 0ul;
            }
            if (protocol_content.substr(index, token.size()) == token) {
                return token.size();
            }
            return 0ul;
        };
        std::stringstream ss;
        auto i = 0ul;
        while (i < protocol_content.size()) {
            auto offset = check_string(protocol_content, i);
            if (offset > 0) {
                ss << new_token;
                i += offset;
            } else {
                ss << protocol_content[i];
                i++;
            }
        }
        return ss.str();
    }

} // pddl_lib
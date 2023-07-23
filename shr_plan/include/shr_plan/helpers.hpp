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

    std::optional<long> get_inst_index(WanderingProtocol w, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.WanderingProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), w);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(FoodProtocol f, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.FoodProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), f);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
    }

    std::optional<long> get_inst_index(FallProtocol f, const shr_parameters::Params &params) {
        const auto &instances = params.pddl.FallProtocols.instances;
        auto it = std::find(instances.begin(), instances.end(), f);
        if (it != instances.end()) {
            auto index = std::distance(instances.begin(), it);
            return index;
        } else {
            return {};
        }
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

    std::optional<long> get_inst_index(InstantiatedParameter inst, const shr_parameters::Params &params) {
        if (inst.type == "WanderingProtocol") {
            return get_inst_index((WanderingProtocol) inst.name, params);
        } else if (inst.type == "MedicineProtocol") {
            return get_inst_index((MedicineProtocol) inst.name, params);
        } else if (inst.type == "FoodProtocol") {
            return get_inst_index((FoodProtocol) inst.name, params);
        } else if (inst.type == "FallProtocol") {
            return get_inst_index((FallProtocol) inst.name, params);
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
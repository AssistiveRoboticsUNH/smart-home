//
// Created by olagh48652 on 2/8/25.
//

#ifndef PLAN_SOLVER_INTERSECTION_HELPERS_HPP
#define PLAN_SOLVER_INTERSECTION_HELPERS_HPP


#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>


void write_to_intersection(const std::string& filePath,
                           std::vector<std::tuple<std::string, std::string, std::string>> keyword_protocol_list) {
    std::ofstream ofs(filePath);
    if (!ofs) {
        std::cerr << "Failed to open output file: " << filePath << std::endl;
        return;
    }

    // Write each tuple to the file
    for (const auto& entry : keyword_protocol_list) {
        ofs << std::get<0>(entry) << " "   // Keyword
            << std::get<1>(entry) << " "   // Protocol Name
            << std::get<2>(entry) << "\n"; // Protocol Type
    }
    ofs.close();
    std::cout << "Predicates successfully written to " << filePath << std::endl;
}

std::vector<std::tuple<std::string, std::string, std::string>> read_predicates_from_file(const std::string& filePath) {
    std::ifstream ifs(filePath);
    if (!ifs) {
        std::cerr << "Failed to open file for reading: " << filePath << std::endl;
        return {};
    }

    std::vector<std::tuple<std::string, std::string, std::string>> predicates;
    std::string keyword, protocolName, protocolType;

    // Read each line and extract keyword, protocolName, and protocolType
    while (ifs >> keyword >> protocolName >> protocolType) {
        predicates.emplace_back(keyword, protocolName, protocolType);
    }

    ifs.close();
    return predicates;
}

#endif //PLAN_SOLVER_INTERSECTION_HELPERS_HPP

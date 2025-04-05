#include <iostream>
#include <fstream>
#include <vector>
#include <tuple>

// --- Function: write_to_intersection ---
// Writes a list of tuples (keyword, protocolName, type) to a file.
void write_to_intersection(const std::string& filePath,
                           const std::vector<std::tuple<std::string, std::string, std::string>>& keyword_protocol_list) {
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

// --- Function: read_from_intersection ---
// Reads from the file and prints each line.
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

int main() {
    std::string outputFile = "intersection.txt";

    // Sample data to write
    std::vector<std::tuple<std::string, std::string, std::string>> keyword_protocol_list = {
            {"already_took_medicine", "am_meds", "MedicineProtocol"},
            {"already_took_medicine", "pm_meds", "MedicineProtocol"},
            {"already_reminded_exercise", "exercise_reminder", "ExerciseReminderProtocol"}
    };

    // Write to file
    write_to_intersection(outputFile, keyword_protocol_list);

    // Read from file
    auto predicates = read_predicates_from_file(outputFile);

    // Print the predicates
    for (const auto& [first, second, third] : predicates) {
        std::cout << "First: " << first << ", Second: " << second << ", Third: " << third << std::endl;
    }

    return 0;
}
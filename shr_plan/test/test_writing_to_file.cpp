#include <iostream>
#include <fstream>
#include <unordered_set>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <sstream>

// Function to trim leading and trailing spaces
std::string trim(const std::string &str) {
    size_t start = str.find_first_not_of(" \t\n\r");
    size_t end = str.find_last_not_of(" \t\n\r");
    return (start == std::string::npos || end == std::string::npos) ? "" : str.substr(start, end - start + 1);
}

// Function to write unique predicates to a file
void writeToFile(const std::string &filename, const std::vector<std::string> &predicates) {
    std::unordered_set<std::string> existingPredicates;

    // Read existing content from the file and add to the set
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        existingPredicates.insert(trim(line));  // Trim and insert each line from the file
    }
    file.close();

    // Open the file again in append mode
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile.is_open()) {
        std::cerr << "Error opening output file!" << std::endl;
        return;
    }

    // Only append unique predicates (after trimming)
    for (const auto &predicate : predicates) {
        std::string trimmedPredicate = trim(predicate);  // Trim the current predicate
        if (existingPredicates.find(trimmedPredicate) == existingPredicates.end()) {
            outFile << predicate << "\n";  // Append if it's not already in the set
            existingPredicates.insert(trimmedPredicate);  // Mark this predicate as written
        }
    }

    outFile.close();
}

int main() {
    // Define some sample predicates to write
    std::vector<std::string> predicates = {
            "already_took_medicine pm_meds",
            "already_took_medicine pm_meds", // Duplicate, will not be written again
            "already_reminded_move move_reminder",
            "already_reminded_move move_reminder"  // Duplicate, will not be written again
    };

    // Output file where predicates will be appended
    std::string outputFile = "output.txt";

    // Call the function to write unique predicates
    writeToFile(outputFile, predicates);

    std::cout << "Predicates written to " << outputFile << std::endl;

    return 0;
}

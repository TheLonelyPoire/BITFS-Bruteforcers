#include "IOUtils.hpp"

#include <iostream>
#include <sstream>
#include <fstream>


std::vector<std::unordered_map<std::string, std::string>> parse_csv(const std::string& filename) {
    std::vector<std::unordered_map<std::string, std::string>> data;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data;
    }

    std::string line;
    std::vector<std::string> headers;
    while (std::getline(file, line)) {

        std::stringstream ss(line);
        std::string cell;
        std::unordered_map<std::string, std::string> row;

        if (headers.empty()) { // Read headers
            while (std::getline(ss, cell, ',')) {
                headers.push_back(cell);
            }
        }
        else { // Read data
            size_t col = 0;
            while (std::getline(ss, cell, ',')) {
                if (col < headers.size()) {
                    row[headers[col]] = cell;
                }
                col++;
            }
            data.push_back(row);
        }
    }

    file.close();
    return data;
}
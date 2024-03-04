#pragma once
#include <string> 

#include <vector>
#include <unordered_map>

// Function to parse CSV file (courtesy of ChatGPT). 
//
// Output is a list (std::vector) of the rows of data in the CSV, each of which is a dictionary
// (std::unordered_map) with both keys and values as strings. The keys are the column names,
// and the values are the entries for that column in the given row.
std::vector<std::unordered_map<std::string, std::string>> parse_csv(const std::string& filename);
#include "Configurator.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

unsigned short Configurator::lidarStdev = 10;
unsigned short Configurator::odoStdev = 1;
unsigned short Configurator::compassStdev = 2;

void Configurator::getConfigFromFile(const std::string& filename) {
    std::ifstream file("../../" + filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << " , Using default values." << std::endl;
    }

    std::string line;
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        char delimiter;
        if (ss >> Configurator::lidarStdev >> delimiter >> Configurator::odoStdev >> delimiter >> Configurator::compassStdev) {
            return;
        }
    }
    std::cerr << "Error reading values from file: " << filename << " , Using default values." << std::endl;
}

unsigned short Configurator::getLidarStdev() { return Configurator::lidarStdev; }
unsigned short Configurator::getOdoStdev() { return Configurator::odoStdev; }
unsigned short Configurator::getCompassStdev() { return Configurator::compassStdev; }
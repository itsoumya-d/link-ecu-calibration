#include "ecu.hpp"

#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    const std::string output = argc > 1 ? argv[1] : "dashboard.html";

    link::Calibration calibration;
    calibration.fuel = {
        {2000, 40, 55}, {2000, 80, 78}, {2000, 120, 92},
        {4000, 40, 62}, {4000, 80, 88}, {4000, 120, 104},
        {6000, 40, 58}, {6000, 80, 82}, {6000, 120, 98},
    };
    calibration.ignition = {
        {2000, 40, 28}, {2000, 80, 22}, {2000, 120, 16},
        {4000, 40, 32}, {4000, 80, 25}, {4000, 120, 18},
        {6000, 40, 30}, {6000, 80, 23}, {6000, 120, 15},
    };
    calibration.targetAfr = 12.5;
    calibration.revLimitRpm = 7000.0;

    const link::OperatingPoint point{4000.0, 100.0, 28.0, 90.0, 1.02};
    const auto fuel = link::calculateFuel(calibration, point);
    const auto ignition = link::calculateIgnition(calibration, point);

    std::cout << "Link ECU calibration\n";
    std::cout << "VE: " << link::formatNumber(fuel.ve) << " %\n";
    std::cout << "Pulse width: " << link::formatNumber(fuel.pulseMs) << " ms\n";
    std::cout << "Duty cycle: " << link::formatNumber(fuel.dutyPercent) << " %\n";
    std::cout << "Lambda error: " << link::formatNumber(fuel.lambdaError, 3) << "\n";
    std::cout << "Final ignition: " << link::formatNumber(ignition.finalAdvanceDeg) << " deg\n";

    std::ofstream file(output);
    file << link::renderDashboard(calibration, point);
    std::cout << "Dashboard: " << output << "\n";
    return 0;
}

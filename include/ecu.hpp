#pragma once

#include <string>
#include <vector>

namespace link {

struct FuelCell {
    double rpm;
    double mapKpa;
    double ve;
};

struct IgnitionCell {
    double rpm;
    double mapKpa;
    double advanceDeg;
};

struct Calibration {
    std::vector<FuelCell> fuel;
    std::vector<IgnitionCell> ignition;
    double targetAfr = 12.5;
    double injectorBaseMs = 8.0;
    double deadTimeMs = 1.0;
    double revLimitRpm = 7000.0;
    double knockRetardDeg = 0.0;
    double iatRetardDeg = 1.0;
};

struct OperatingPoint {
    double rpm = 4000.0;
    double mapKpa = 100.0;
    double iatC = 25.0;
    double coolantC = 90.0;
    double measuredLambda = 1.0;
};

struct FuelResult {
    double ve;
    double targetLambda;
    double lambdaError;
    double pulseMs;
    double dutyPercent;
};

struct IgnitionResult {
    double baseAdvanceDeg;
    double iatCorrectionDeg;
    double finalAdvanceDeg;
};

double bilinearVe(const Calibration& calibration, double rpm, double mapKpa);
double bilinearAdvance(const Calibration& calibration, double rpm, double mapKpa);
FuelResult calculateFuel(const Calibration& calibration, const OperatingPoint& point);
IgnitionResult calculateIgnition(const Calibration& calibration, const OperatingPoint& point);
bool revLimitActive(const Calibration& calibration, double rpm);
std::vector<std::string> validateCalibration(const Calibration& calibration);
std::string renderDashboard(const Calibration& calibration, const OperatingPoint& point);
std::string formatNumber(double value, int decimals = 2);

}  // namespace link

#include "ecu.hpp"

#include <cmath>
#include <iostream>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        ++failures;
    }
}

void expectNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "FAIL: " << message << " (expected " << expected << ", got " << actual << ")\n";
        ++failures;
    }
}

link::Calibration baseCalibration() {
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
    calibration.injectorBaseMs = 8.0;
    calibration.deadTimeMs = 1.0;
    calibration.revLimitRpm = 7000.0;
    calibration.knockRetardDeg = 0.0;
    calibration.iatRetardDeg = 1.0;
    return calibration;
}

void testFuelInterpolationAtCell() {
    const auto calibration = baseCalibration();
    expectNear(link::bilinearVe(calibration, 2000, 40), 55.0, 1e-9, "VE at exact cell");
}

void testFuelInterpolationClampsLow() {
    const auto calibration = baseCalibration();
    expectNear(link::bilinearVe(calibration, 500, 10), 55.0, 1e-9, "VE clamps low");
}

void testFuelInterpolationClampsHigh() {
    const auto calibration = baseCalibration();
    expectNear(link::bilinearVe(calibration, 9000, 200), 98.0, 1e-9, "VE clamps high");
}

void testFuelBilinearMidpoint() {
    const auto calibration = baseCalibration();
    expectNear(link::bilinearVe(calibration, 3000, 60), 70.75, 1e-9, "VE bilinear midpoint");
}

void testIgnitionInterpolation() {
    const auto calibration = baseCalibration();
    expectNear(link::bilinearAdvance(calibration, 3000, 60), 26.75, 1e-9, "ignition bilinear midpoint");
}

void testFuelPulseWidth() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateFuel(calibration, {4000, 100, 25, 90, 1.0});
    expectNear(result.ve, 96.0, 1e-9, "VE at 4000/100");
    expectNear(result.pulseMs, 10.03168, 1e-5, "pulse width");
}

void testDeadTimeIncluded() {
    auto calibration = baseCalibration();
    calibration.deadTimeMs = 2.0;
    const auto result = link::calculateFuel(calibration, {4000, 100, 25, 90, 1.0});
    expectNear(result.pulseMs, 11.03168, 1e-5, "dead time included");
}

void testDutyCycle() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateFuel(calibration, {4000, 100, 25, 90, 1.0});
    expectNear(result.dutyPercent, 33.4389, 1e-3, "duty cycle");
}

void testZeroRpmDuty() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateFuel(calibration, {0, 100, 25, 90, 1.0});
    expectNear(result.dutyPercent, 0.0, 1e-9, "zero rpm duty");
}

void testTargetLambda() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateFuel(calibration, {4000, 100, 25, 90, 1.0});
    expectNear(result.targetLambda, 12.5 / 14.7, 1e-9, "target lambda");
}

void testLambdaError() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateFuel(calibration, {4000, 100, 25, 90, 1.02});
    expectNear(result.lambdaError, 1.02 - (12.5 / 14.7), 1e-9, "lambda error");
}

void testIgnitionBaseAndIat() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateIgnition(calibration, {4000, 100, 28, 90, 1.0});
    expectNear(result.baseAdvanceDeg, 21.5, 1e-9, "base ignition");
    expectNear(result.iatCorrectionDeg, -0.8, 1e-9, "IAT correction");
    expectNear(result.finalAdvanceDeg, 20.7, 1e-9, "final ignition");
}

void testIatBelowTwentyHasNoCorrection() {
    const auto calibration = baseCalibration();
    const auto result = link::calculateIgnition(calibration, {4000, 100, 15, 90, 1.0});
    expectNear(result.iatCorrectionDeg, 0.0, 1e-9, "no IAT correction below 20C");
}

void testKnockRetard() {
    auto calibration = baseCalibration();
    calibration.knockRetardDeg = 3.0;
    const auto result = link::calculateIgnition(calibration, {4000, 100, 20, 90, 1.0});
    expectNear(result.finalAdvanceDeg, 18.5, 1e-9, "knock retard");
}

void testIgnitionClampsAtZero() {
    auto calibration = baseCalibration();
    calibration.knockRetardDeg = 100.0;
    const auto result = link::calculateIgnition(calibration, {4000, 100, 20, 90, 1.0});
    expectNear(result.finalAdvanceDeg, 0.0, 1e-9, "ignition clamps at zero");
}

void testRevLimit() {
    const auto calibration = baseCalibration();
    expect(link::revLimitActive(calibration, 7000), "rev limit active at limit");
    expect(!link::revLimitActive(calibration, 6999), "rev limit clear below limit");
}

void testValidationPasses() {
    const auto calibration = baseCalibration();
    expect(link::validateCalibration(calibration).empty(), "valid calibration");
}

void testValidationEmptyTables() {
    link::Calibration calibration;
    const auto errors = link::validateCalibration(calibration);
    expect(errors.size() >= 2, "empty tables reported");
}

void testValidationTargetAfr() {
    auto calibration = baseCalibration();
    calibration.targetAfr = 9.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid target AFR");
}

void testValidationInjector() {
    auto calibration = baseCalibration();
    calibration.injectorBaseMs = 0.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid injector base");
}

void testValidationDeadTime() {
    auto calibration = baseCalibration();
    calibration.deadTimeMs = 11.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid dead time");
}

void testValidationRevLimit() {
    auto calibration = baseCalibration();
    calibration.revLimitRpm = 500.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid rev limit");
}

void testValidationVe() {
    auto calibration = baseCalibration();
    calibration.fuel[0].ve = 200.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid VE");
}

void testValidationAdvance() {
    auto calibration = baseCalibration();
    calibration.ignition[0].advanceDeg = 80.0;
    expect(!link::validateCalibration(calibration).empty(), "invalid advance");
}

void testDashboardContainsTokens() {
    const auto calibration = baseCalibration();
    const auto html = link::renderDashboard(calibration, {4000, 100, 28, 90, 1.02});
    expect(html.find("Link ECU") != std::string::npos, "dashboard brand");
    expect(html.find("#3C1053") != std::string::npos, "dashboard purple token");
    expect(html.find("#D22730") != std::string::npos, "dashboard red token");
    expect(html.find("10.03") != std::string::npos, "dashboard pulse width");
}

void testDashboardShowsValidation() {
    link::Calibration calibration = baseCalibration();
    calibration.targetAfr = 9.0;
    const auto html = link::renderDashboard(calibration, {4000, 100, 25, 90, 1.0});
    expect(html.find("target AFR") != std::string::npos, "dashboard validation error");
}

void testFormatNumber() {
    expect(link::formatNumber(10.03168) == "10.03", "format two decimals");
    expect(link::formatNumber(10.03168, 3) == "10.032", "format three decimals");
}

void testDashboardRevLimitAlert() {
    const auto calibration = baseCalibration();
    const auto html = link::renderDashboard(calibration, {7200, 100, 25, 90, 1.0});
    expect(html.find("ACTIVE") != std::string::npos, "rev limit alert");
}

}  // namespace

int main() {
    testFuelInterpolationAtCell();
    testFuelInterpolationClampsLow();
    testFuelInterpolationClampsHigh();
    testFuelBilinearMidpoint();
    testIgnitionInterpolation();
    testFuelPulseWidth();
    testDeadTimeIncluded();
    testDutyCycle();
    testZeroRpmDuty();
    testTargetLambda();
    testLambdaError();
    testIgnitionBaseAndIat();
    testIatBelowTwentyHasNoCorrection();
    testKnockRetard();
    testIgnitionClampsAtZero();
    testRevLimit();
    testValidationPasses();
    testValidationEmptyTables();
    testValidationTargetAfr();
    testValidationInjector();
    testValidationDeadTime();
    testValidationRevLimit();
    testValidationVe();
    testValidationAdvance();
    testDashboardContainsTokens();
    testDashboardShowsValidation();
    testFormatNumber();
    testDashboardRevLimitAlert();

    if (failures == 0) {
        std::cout << "28 tests passed\n";
        return 0;
    }
    std::cerr << failures << " test(s) failed\n";
    return 1;
}

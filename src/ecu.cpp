#include "ecu.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace link {
namespace {

constexpr double kStoichAfr = 14.7;

struct Bracket {
    std::size_t low;
    std::size_t high;
    double fraction;
};

std::vector<double> uniqueAxis(const std::vector<double>& values) {
    std::vector<double> axis;
    for (double value : values) {
        if (axis.empty() || std::abs(axis.back() - value) > 1e-9) axis.push_back(value);
    }
    return axis;
}

Bracket bracket(const std::vector<double>& axis, double value) {
    if (axis.empty()) return {0, 0, 0.0};
    if (axis.size() == 1 || value <= axis.front()) return {0, 0, 0.0};
    if (value >= axis.back()) return {axis.size() - 1, axis.size() - 1, 0.0};
    for (std::size_t i = 0; i + 1 < axis.size(); ++i) {
        if (value <= axis[i + 1]) {
            const double span = axis[i + 1] - axis[i];
            return {i, i + 1, span <= 1e-12 ? 0.0 : (value - axis[i]) / span};
        }
    }
    return {axis.size() - 1, axis.size() - 1, 0.0};
}

const FuelCell* findFuel(const Calibration& calibration, double rpm, double mapKpa) {
    for (const auto& cell : calibration.fuel) {
        if (std::abs(cell.rpm - rpm) < 1e-9 && std::abs(cell.mapKpa - mapKpa) < 1e-9) return &cell;
    }
    return nullptr;
}

const IgnitionCell* findIgnition(const Calibration& calibration, double rpm, double mapKpa) {
    for (const auto& cell : calibration.ignition) {
        if (std::abs(cell.rpm - rpm) < 1e-9 && std::abs(cell.mapKpa - mapKpa) < 1e-9) return &cell;
    }
    return nullptr;
}

double interpolateFuel(const Calibration& calibration, double rpm, double mapKpa) {
    std::vector<double> rpms;
    std::vector<double> maps;
    for (const auto& cell : calibration.fuel) {
        rpms.push_back(cell.rpm);
        maps.push_back(cell.mapKpa);
    }
    std::sort(rpms.begin(), rpms.end());
    std::sort(maps.begin(), maps.end());
    rpms = uniqueAxis(rpms);
    maps = uniqueAxis(maps);
    const auto r = bracket(rpms, rpm);
    const auto m = bracket(maps, mapKpa);
    auto valueAt = [&](double rValue, double mValue) {
        const FuelCell* cell = findFuel(calibration, rValue, mValue);
        return cell ? cell->ve : 0.0;
    };
    const double v00 = valueAt(rpms[r.low], maps[m.low]);
    const double v10 = valueAt(rpms[r.high], maps[m.low]);
    const double v01 = valueAt(rpms[r.low], maps[m.high]);
    const double v11 = valueAt(rpms[r.high], maps[m.high]);
    const double low = v00 + (v10 - v00) * r.fraction;
    const double high = v01 + (v11 - v01) * r.fraction;
    return low + (high - low) * m.fraction;
}

double interpolateIgnition(const Calibration& calibration, double rpm, double mapKpa) {
    std::vector<double> rpms;
    std::vector<double> maps;
    for (const auto& cell : calibration.ignition) {
        rpms.push_back(cell.rpm);
        maps.push_back(cell.mapKpa);
    }
    std::sort(rpms.begin(), rpms.end());
    std::sort(maps.begin(), maps.end());
    rpms = uniqueAxis(rpms);
    maps = uniqueAxis(maps);
    const auto r = bracket(rpms, rpm);
    const auto m = bracket(maps, mapKpa);
    auto valueAt = [&](double rValue, double mValue) {
        const IgnitionCell* cell = findIgnition(calibration, rValue, mValue);
        return cell ? cell->advanceDeg : 0.0;
    };
    const double v00 = valueAt(rpms[r.low], maps[m.low]);
    const double v10 = valueAt(rpms[r.high], maps[m.low]);
    const double v01 = valueAt(rpms[r.low], maps[m.high]);
    const double v11 = valueAt(rpms[r.high], maps[m.high]);
    const double low = v00 + (v10 - v00) * r.fraction;
    const double high = v01 + (v11 - v01) * r.fraction;
    return low + (high - low) * m.fraction;
}

}  // namespace

double bilinearVe(const Calibration& calibration, double rpm, double mapKpa) {
    return interpolateFuel(calibration, std::max(0.0, rpm), std::max(0.0, mapKpa));
}

double bilinearAdvance(const Calibration& calibration, double rpm, double mapKpa) {
    return interpolateIgnition(calibration, std::max(0.0, rpm), std::max(0.0, mapKpa));
}

FuelResult calculateFuel(const Calibration& calibration, const OperatingPoint& point) {
    const double ve = bilinearVe(calibration, point.rpm, point.mapKpa);
    const double targetLambda = calibration.targetAfr / kStoichAfr;
    const double pulse = calibration.deadTimeMs +
        (ve / 100.0) * (point.mapKpa / 100.0) * (kStoichAfr / calibration.targetAfr) * calibration.injectorBaseMs;
    const double periodMs = point.rpm > 0.0 ? 120000.0 / point.rpm : 0.0;
    const double duty = periodMs > 0.0 ? (pulse / periodMs) * 100.0 : 0.0;
    return {
        ve,
        targetLambda,
        point.measuredLambda - targetLambda,
        pulse,
        duty,
    };
}

IgnitionResult calculateIgnition(const Calibration& calibration, const OperatingPoint& point) {
    const double base = bilinearAdvance(calibration, point.rpm, point.mapKpa);
    const double iatCorrection = std::max(0.0, (point.iatC - 20.0) / 10.0) * calibration.iatRetardDeg;
    const double finalAdvance = std::clamp(base - calibration.knockRetardDeg - iatCorrection, 0.0, 50.0);
    return {base, -iatCorrection, finalAdvance};
}

bool revLimitActive(const Calibration& calibration, double rpm) {
    return rpm >= calibration.revLimitRpm;
}

std::vector<std::string> validateCalibration(const Calibration& calibration) {
    std::vector<std::string> errors;
    if (calibration.fuel.empty()) errors.push_back("fuel table is empty");
    if (calibration.ignition.empty()) errors.push_back("ignition table is empty");
    if (calibration.targetAfr < 10.0 || calibration.targetAfr > 20.0) errors.push_back("target AFR must be between 10 and 20");
    if (calibration.injectorBaseMs <= 0.0 || calibration.injectorBaseMs > 30.0) errors.push_back("injector base must be between 0 and 30 ms");
    if (calibration.deadTimeMs < 0.0 || calibration.deadTimeMs > 10.0) errors.push_back("dead time must be between 0 and 10 ms");
    if (calibration.revLimitRpm < 1000.0 || calibration.revLimitRpm > 12000.0) errors.push_back("rev limit must be between 1000 and 12000 rpm");
    for (const auto& cell : calibration.fuel) {
        if (cell.ve < 0.0 || cell.ve > 150.0) {
            errors.push_back("fuel VE must be between 0 and 150");
            break;
        }
    }
    for (const auto& cell : calibration.ignition) {
        if (cell.advanceDeg < -10.0 || cell.advanceDeg > 60.0) {
            errors.push_back("ignition advance must be between -10 and 60 degrees");
            break;
        }
    }
    return errors;
}

std::string formatNumber(double value, int decimals) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(decimals) << value;
    return stream.str();
}

std::string renderDashboard(const Calibration& calibration, const OperatingPoint& point) {
    const FuelResult fuel = calculateFuel(calibration, point);
    const IgnitionResult ignition = calculateIgnition(calibration, point);
    const bool limited = revLimitActive(calibration, point.rpm);
    const auto errors = validateCalibration(calibration);

    std::ostringstream html;
    html << R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Link ECU Calibration</title>
<style>
:root{--purple:#3C1053;--deep:#310E41;--red:#D22730;--surface:#F3EEF7;--white:#fff;--muted:#6B5A75;--radius:12px}
*{box-sizing:border-box}body{margin:0;background:var(--white);color:var(--purple);font-family:"Inclusive Sans",Arial,sans-serif;line-height:1.45}
.shell{max-width:1180px;margin:0 auto;padding:40px 28px}header{background:var(--purple);color:var(--white);padding:24px;border-bottom:4px solid var(--red)}
h1,h2{font-family:"Link Display","Inclusive Sans",Arial,sans-serif;margin:0}h1{font-size:44px;line-height:1}h2{font-size:19px}
.kicker{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:#E8D9F2;margin:0 0 8px}
.grid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin:22px 0}
.card{background:var(--surface);border-radius:var(--radius);padding:16px}.card span{display:block;font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted)}.card strong{display:block;margin-top:8px;font-size:26px;color:var(--purple)}
.card.alert{border-top:4px solid var(--red)}.card.alert strong{color:var(--red)}
table{width:100%;border-collapse:collapse;margin-top:12px}th{text-align:left;font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);border-bottom:2px solid var(--purple);padding:9px}td{padding:11px 9px;border-bottom:1px solid #E3D8EB}.num{text-align:right;font-variant-numeric:tabular-nums}
.panel{border:1px solid #E3D8EB;border-radius:var(--radius);padding:18px;margin-top:16px}.errors{color:var(--red)}footer{margin-top:28px;padding-top:14px;border-top:1px solid #E3D8EB;color:var(--muted);font-size:12px}
@media(max-width:800px){.grid{grid-template-columns:repeat(2,1fr)}}
</style>
</head>
<body>
<div class="shell">
<header>
<p class="kicker">Link ECU · calibration core</p>
<h1>Fuel. Ignition. Confidence.</h1>
<p>Engine calibration at 4,000 rpm and 100 kPa, with lambda and protection checks.</p>
</header>
<section class="grid">
<div class="card"><span>VE</span><strong>)HTML" << formatNumber(fuel.ve) << R"HTML( %</strong></div>
<div class="card"><span>Pulse width</span><strong>)HTML" << formatNumber(fuel.pulseMs) << R"HTML( ms</strong></div>
<div class="card"><span>Duty cycle</span><strong>)HTML" << formatNumber(fuel.dutyPercent) << R"HTML( %</strong></div>
<div class="card )HTML" << (limited ? "alert" : "") << R"HTML("><span>Rev limit</span><strong>)HTML" << (limited ? "ACTIVE" : "CLEAR") << R"HTML(</strong></div>
</section>
<section class="panel">
<h2>Calibration result</h2>
<table>
<tr><th>Metric</th><th class="num">Value</th></tr>
<tr><td>Target lambda</td><td class="num">)HTML" << formatNumber(fuel.targetLambda, 3) << R"HTML(</td></tr>
<tr><td>Measured lambda</td><td class="num">)HTML" << formatNumber(point.measuredLambda, 3) << R"HTML(</td></tr>
<tr><td>Lambda error</td><td class="num">)HTML" << formatNumber(fuel.lambdaError, 3) << R"HTML(</td></tr>
<tr><td>Base ignition</td><td class="num">)HTML" << formatNumber(ignition.baseAdvanceDeg) << R"HTML( deg</td></tr>
<tr><td>IAT correction</td><td class="num">)HTML" << formatNumber(ignition.iatCorrectionDeg) << R"HTML( deg</td></tr>
<tr><td>Final ignition</td><td class="num">)HTML" << formatNumber(ignition.finalAdvanceDeg) << R"HTML( deg</td></tr>
</table>
</section>
<section class="panel">
<h2>Validation</h2>
)HTML";
    if (errors.empty()) {
        html << "<p>Calibration valid: fuel and ignition tables are within bounds.</p>";
    } else {
        html << "<ul class=\"errors\">";
        for (const auto& error : errors) html << "<li>" << error << "</li>";
        html << "</ul>";
    }
    html << R"HTML(
</section>
<footer>Link ECU calibration reference · C++17 · CMake · CTest</footer>
</div>
</body>
</html>)HTML";
    return html.str();
}

}  // namespace link

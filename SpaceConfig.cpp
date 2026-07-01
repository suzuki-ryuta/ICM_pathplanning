#include "SpaceConfig.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cctype>
#include <cmath>

static constexpr double DENOM_MM = 297.0;  // fixed denominator
static constexpr double Y_TARGET  = 10.0;  // target nondimensional max y

std::string SpaceConfig::trim(const std::string& s) {
    const char* ws = " \t\r\n";
    size_t start = s.find_first_not_of(ws);
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(ws);
    return s.substr(start, end - start + 1);
}

bool SpaceConfig::startsWithKey(const std::string& line, const std::string& key) {
    // Trim left spaces then check prefix "key" followed by optional spaces then '='
    size_t i = 0;
    while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
    if (i >= line.size()) return false;
    // check key match
    if (line.size() - i < key.size()) return false;
    for (size_t k = 0; k < key.size(); ++k) {
        if (line[i + k] != key[k]) return false;
    }
    // next non-space char after key should be '='
    size_t j = i + key.size();
    while (j < line.size() && std::isspace(static_cast<unsigned char>(line[j]))) ++j;
    if (j < line.size() && line[j] == '=') return true;
    return false;
}

bool SpaceConfig::load(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << filepath << " for reading\n";
        return false;
    }

    // Read whole file to memory (we need it later for save)
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(ifs, line)) {
        lines.push_back(line);
    }
    ifs.close();

    // Parse [range] section to find belt_width and cluster_distance_threshold
    bool inRange = false;
    belt_width = DENOM_MM; // default fallback
    cluster_distance_threshold = 6.0; // default fallback

    for (const auto &ln : lines) {
        std::string t = trim(ln);
        if (t.empty()) continue;
        if (t.size() > 0 && t[0] == ';') continue; // comment
        if (t.front() == '[' && t.back() == ']') {
            std::string sec = t.substr(1, t.size() - 2);
            if (sec == "range") inRange = true;
            else inRange = false;
            continue;
        }
        if (!inRange) continue;

        size_t eq = ln.find('=');
        if (eq == std::string::npos) continue;
        std::string key = trim(ln.substr(0, eq));
        std::string val = trim(ln.substr(eq + 1));

        if (key == "belt_width") {
            size_t sc = val.find(';');
            std::string valnum = (sc == std::string::npos) ? trim(val) : trim(val.substr(0, sc));
            if (!valnum.empty()) {
                belt_width = atof(valnum.c_str());
            }
        }
        else if (key == "cluster_distance_threshold") {
            size_t sc = val.find(';');
            std::string valnum = (sc == std::string::npos) ? trim(val) : trim(val.substr(0, sc));
            if (!valnum.empty()) {
                cluster_distance_threshold = atof(valnum.c_str());
            }
        }
    }

    computeNondimensional();
    return true;
}

void SpaceConfig::computeNondimensional() {
    y_nd = (Y_TARGET / DENOM_MM) * belt_width;
    x_nd = y_nd;
    th_nd = 0.5 * y_nd;
}

bool SpaceConfig::save(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open " << filepath << " for reading (save)\n";
        return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(ifs, line)) lines.push_back(line);
    ifs.close();

    std::vector<std::string> out;
    bool inRange = false;
    bool wrote_x = false, wrote_y = false, wrote_th = false;
    bool range_section_seen = false;

    // Convert nondimensional to int
    int x_i = static_cast<int>(std::round(x_nd));
    int y_i = static_cast<int>(std::round(y_nd));
    int th_i = static_cast<int>(std::round(th_nd));

    for (size_t idx = 0; idx < lines.size(); ++idx) {
        std::string ln = lines[idx];
        std::string t = trim(ln);

        if (t.size() > 0 && t.front() == '[' && t.back() == ']') {
            if (inRange) {
                if (!wrote_x) out.push_back("x=" + std::to_string(x_i));
                if (!wrote_y) out.push_back("y=" + std::to_string(y_i));
                if (!wrote_th) out.push_back("th=" + std::to_string(th_i));
                inRange = false;
            }

            out.push_back(ln);
            std::string sec = t.substr(1, t.size() - 2);
            if (sec == "range") {
                inRange = true;
                range_section_seen = true;
            }
            continue;
        }

        if (!inRange) {
            out.push_back(ln);
            continue;
        }

        if (t.empty() || t.front() == ';') {
            out.push_back(ln);
            continue;
        }

        size_t eq = ln.find('=');
        if (eq == std::string::npos) {
            out.push_back(ln);
            continue;
        }

        std::string key = trim(ln.substr(0, eq));

        if (key == "x") {
            out.push_back("x=" + std::to_string(x_i));
            wrote_x = true;
        } else if (key == "y") {
            out.push_back("y=" + std::to_string(y_i));
            wrote_y = true;
        } else if (key == "th" || key == "theta") {
            out.push_back("th=" + std::to_string(th_i));
            wrote_th = true;
        } else {
            out.push_back(ln);
        }
    }

    if (inRange) {
        if (!wrote_x) out.push_back("x=" + std::to_string(x_i));
        if (!wrote_y) out.push_back("y=" + std::to_string(y_i));
        if (!wrote_th) out.push_back("th=" + std::to_string(th_i));
    }

    if (!range_section_seen) {
        out.push_back("");
        out.push_back("[range]");
        out.push_back("x=" + std::to_string(x_i));
        out.push_back("y=" + std::to_string(y_i));
        out.push_back("th=" + std::to_string(th_i));
        out.push_back("belt_width=" + std::to_string(static_cast<int>(std::round(belt_width))));
    }

    std::ofstream ofs(filepath, std::ios::trunc);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open " << filepath << " for writing\n";
        return false;
    }

    for (const auto &o : out) ofs << o << "\n";
    ofs.close();
    return true;
}

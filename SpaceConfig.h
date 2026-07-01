#pragma once
#include <string>

class SpaceConfig {
public:
    // belt_width read from INI (mm)
    double belt_width;
    // cluster_distance_threshold read from INI
    double cluster_distance_threshold;

    // computed nondimensional values
    double x_nd;
    double y_nd;
    double th_nd;

    // load INI and compute values
    bool load(const std::string& filepath);

    // overwrite x,y,th in [range] keeping belt_width line intact
    bool save(const std::string& filepath);

private:
    void computeNondimensional();

    // helpers
    static std::string trim(const std::string& s);
    static bool startsWithKey(const std::string& line, const std::string& key);
};

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <vector>
#include <cmath>

constexpr double pi() { return M_PI; }
constexpr double deg2rad(const double x) { return x * pi() / 180.0; }
constexpr double rad2deg(double x) { return x * 180.0 / pi(); }

std::string hasData(const std::string& s);

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, std::vector<double> maps_x,
                                        std::vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x,
                                                   std::vector<double> maps_y);
std::vector<double> getFrenet(double x, double y, double theta,
                              std::vector<double> maps_x,
                              std::vector<double> maps_y);
std::vector<double> getXY(double s, double d, std::vector<double> maps_s,
                    std::vector<double> maps_x, std::vector<double> maps_y);

#endif // UTILS_H

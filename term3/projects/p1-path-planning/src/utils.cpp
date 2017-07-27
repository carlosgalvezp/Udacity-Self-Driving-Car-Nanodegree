#include "utils.h"

#include <limits>

#include <Eigen/Dense>

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, std::vector<double> maps_x,
                                        std::vector<double> maps_y)
{
    double closestLen = std::numeric_limits<double>::max();
    int closestWaypoint = 0;

    for(std::size_t i = 0U; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x,
                                                   std::vector<double> maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4.0)
    {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta,
                        std::vector<double> maps_x,
                        std::vector<double> maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000.0 - maps_x[prev_wp];
    double center_y = 2000.0 - maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1.0;
    }

    // calculate s value
    double frenet_s = 0.0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0.0, 0.0, proj_x, proj_y);

    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, std::vector<double> maps_s,
                    std::vector<double> maps_x, std::vector<double> maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2.0;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}

void generateJerkMinTrajectory(const double x0, const double x0_d, const double x0_dd,
                               const double xf, const double xf_d, const double xf_dd,
                               const double t,
                               std::vector<double>& trajectory_coeffs)
{
    // Define output
    const std::size_t kNrCoeffs = 6U;  // 5th-order polynomial
    trajectory_coeffs.resize(kNrCoeffs);

    // Compute the last coefficients
    const double t2 = t * t;
    const double t3 = t * t2;
    const double t4 = t * t3;
    const double t5 = t * t4;

    Eigen::Matrix3d A;
    A <<     t3,       t4,        t5,
         3.0*t2,   4.0*t3,    5.0*t4,
         6.0*t,   12.0*t2,   20.0*t3;

    Eigen::Vector3d b;
    b << xf    - (x0 + x0_d*t + 0.5*x0_dd*t2),
         xf_d  - (     x0_d   +     x0_dd*t),
         xf_dd - (                  x0_dd);

    Eigen::Vector3d a3a4a5 = A.inverse() * b;

    // Set output
    trajectory_coeffs[0U] = x0;
    trajectory_coeffs[1U] = x0_d;
    trajectory_coeffs[2U] = 0.5 * x0_dd;
    trajectory_coeffs[3U] = a3a4a5[0U];
    trajectory_coeffs[4U] = a3a4a5[1U];
    trajectory_coeffs[5U] = a3a4a5[2U];
}

double evaluatePolynomial(const std::vector<double>& coeffs, const double x)
{
    double result = 0.0;
    for (std::size_t i = 0U; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * std::pow(x, i);
    }

    return result;
}

std::vector<double> differentiatePolynomial(const std::vector<double> &coeffs)
{
    std::vector<double> output(coeffs.size() - 1U);
    for (std::size_t i = 1U; i < coeffs.size(); ++i)
    {
        output[i - 1U] = i * coeffs[i];
    }

    return output;
}

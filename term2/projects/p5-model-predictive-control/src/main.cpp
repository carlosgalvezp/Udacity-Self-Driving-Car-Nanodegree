#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>

#include "json.hpp"
#include "MPC.h"
#include "tools.h"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");

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

int main()
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        std::string sdata = std::string(data).substr(0, length);
//        std::cout << sdata << std::endl;

        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
        {
            std::string s = hasData(sdata);

            if (s != "")
            {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    // Get reference trajectory, in global coordinates
                    std::vector<double> ptsx = j[1]["ptsx"];
                    std::vector<double> ptsy = j[1]["ptsy"];

                    // Get vehicle state
                    double px = j[1]["x"];                  // [m]
                    double py = j[1]["y"];                  // [m]
                    double psi = j[1]["psi"];               // [m]
                    double v = j[1]["speed"];               // [mph]

                    // Transform velocity to m/s
                    v = Tools::mphtoms(v);                       // [m/s]

                    // Convert trajectory points from global to local frame
                    Eigen::VectorXd xvals(ptsx.size());
                    Eigen::VectorXd yvals(ptsy.size());
                    const double c = std::cos(psi);
                    const double s = std::sin(psi);

                    for (std::size_t i = 0U; i < ptsx.size(); ++i)
                    {
                        xvals[i] =  c * (ptsx[i] - px) + s * (ptsy[i] - py);
                        yvals[i] = -s * (ptsx[i] - px) + c * (ptsy[i] - py);
                    }

                    // Fit 3rd-order polynomial to trajectory
                    const int order = 3;
                    const Eigen::VectorXd trajectory = Tools::polyfit(xvals, yvals, order);

                    // Setup vehicle state in local coordinates
                    const double x_local = 0.0;
                    const double y_local = 0.0;
                    const double psi_local = 0.0;

                    // Compute CTE and epsi in local coordinates
                    const double cte_local = Tools::polyeval(trajectory, x_local);

                    // epsi = psi - atan(f'(x)) where f is the trajectory
                    // Since we evaluate at x_local = 0, the only remaining
                    // term is trajectory[1]
                    const double psi_traj = std::atan(trajectory[1]);
                    const double epsi_local = psi_local - psi_traj;

                    // Create state vector in local coordinates
                    Eigen::VectorXd state(6);
                    state << x_local, y_local, psi_local, v, cte_local, epsi_local;

                    // Placeholder for actuator commands from MPC
                    Actuators commands;

                    // Placeholder for predicted trajectory from MPC
                    std::vector<double> mpc_x_vals;
                    std::vector<double> mpc_y_vals;

                    // Run MPC routine
                    const auto t1 = std::chrono::high_resolution_clock::now();
                    mpc.computeCommands(state, trajectory, commands,
                                        mpc_x_vals, mpc_y_vals);
                    const auto t2 = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> t = t2 - t1;

                    std::cout << "Computation time: " << t.count() << " ms" << std::endl;

                    // Extract actuator commands
                    const double steering = commands.steering / Tools::deg2rad(25.0);
                    const double acceleration = commands.acceleration;

                    // Output through JSON message. The minus sign in steering
                    // is due to the simulator using a different sign convention
                    // for angles
                    json msgJson;
                    msgJson["steering_angle"] = -steering;
                    msgJson["throttle"] = acceleration;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    std::vector<double> next_x_vals(ptsx.size());
                    std::vector<double> next_y_vals(ptsy.size());

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    for (std::size_t i = 0U; i < ptsx.size(); ++i)
                    {
                        next_x_vals[i] = xvals[i];
                        next_y_vals[i] = yvals[i];
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
                       size_t, size_t)
    {
        const std::string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;

    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}

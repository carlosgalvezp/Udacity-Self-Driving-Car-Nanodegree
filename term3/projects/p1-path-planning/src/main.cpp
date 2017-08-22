#include <limits>
#include <fstream>
#include <cmath>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <uWS/uWS.h>

#include "json.hpp"
#include "sensor_fusion_data.h"
#include "ego_vehicle_data.h"
#include "map.h"
#include "path_planner.h"
#include "utils.h"

// for convenience
using json = nlohmann::json;

int main()
{
    uWS::Hub h;
    PathPlanner path_planner;
    EgoVehicleData ego_vehicle_data;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    MapData map_raw_data("../data/highway_map.csv");
    Map map(map_raw_data);

    h.onMessage([&map, &path_planner, &ego_vehicle_data](
                    uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                    uWS::OpCode opCode)
    {
        (void) opCode;
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
        
                if (event == "telemetry")
                {
                    // j[1] is the data JSON object
                    // Main car's localization Data
                    ego_vehicle_data.x = j[1]["x"];
                    ego_vehicle_data.y = j[1]["y"];
                    ego_vehicle_data.s = j[1]["s"];
                    ego_vehicle_data.d = j[1]["d"];
                    ego_vehicle_data.yaw = j[1]["yaw"];
                    ego_vehicle_data.speed = mph2ms(j[1]["speed"]);

                    // Previous path data given to the Planner
                    const auto& previous_path_x = j[1]["previous_path_x"];
                    const auto& previous_path_y = j[1]["previous_path_y"];

                    // Previous path's end s and d values
//                    double end_path_s = j[1]["end_path_s"];
//                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    const std::vector<std::vector<double>>& sensor_fusion_raw = j[1]["sensor_fusion"];
                    SensorFusionData sensor_fusion_data(sensor_fusion_raw);

                    // Plan desired trajectory
                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;

                    auto t1 = std::chrono::high_resolution_clock::now();
                    path_planner.generateTrajectory(ego_vehicle_data,
                                                    sensor_fusion_data,
                                                    map,
                                                    previous_path_x,
                                                    previous_path_y,
                                                    next_x_vals, next_y_vals);
                    auto t2 = std::chrono::high_resolution_clock::now();
                    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                    std::cout << "Computation time: " << diff << " us" << std::endl;

                    // Output
                    json msgJson;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
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
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t)
    {
        (void)data;
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
        (void)ws;
        (void)req;
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
        (void)code;
        (void)message;
        (void)length;
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

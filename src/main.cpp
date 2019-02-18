#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double steer_value;
          double throttle_value;

          // Convert vectors to Eigen::VectorXd
          int n_points = ptsx.size();
          VectorXd pointsX(n_points);
          pointsX.fill(0.0);
          
          VectorXd pointsY(n_points);
          pointsX.fill(0.0);

          for (unsigned int i = 0; i < ptsx.size(); i++) 
          {
            double xdiff = ptsx[i] - px;
            double ydiff = ptsy[i] - py;

            pointsX[i] = xdiff * cos(-psi) - ydiff * sin(-psi);
            pointsY[i] = xdiff * sin(-psi) + ydiff * cos(-psi);
          }

          auto coeffs = polyfit(pointsX, pointsY, 3); 
          std::cout << "coeffs: " << coeffs.format(singleLineFormat) << std::endl;
          
          double cte = polyeval(coeffs, 0);
          std::cout << "cte: " << cte << std::endl;
          
          double epsi = -atan(coeffs[1]);
          std::cout << "epsi: " << epsi << std::endl;

          VectorXd state(6);
          state << 1, 0, 0, v, cte, epsi;
          std::cout << "STATE: " << state.format(singleLineFormat) << std::endl;
          
          std::vector<std::tuple<double, double>> predicted_path;
          auto vars = mpc.Solve(state, coeffs, predicted_path);
          VectorXd varsXd(8);
          for(int idx = 0; idx < 8; idx++) { varsXd[idx] = vars[idx]; }
          std::cout << "SOLVED VARS: " << varsXd.format(singleLineFormat) << std::endl;

          steer_value = -1.0 * vars[6] / deg2rad(25);
          throttle_value = vars[7];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          std::cout << "STEER=" << steer_value << " THROTTLE=" << throttle_value << std::endl;
          
          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (const auto& coords : predicted_path) {
              mpc_x_vals.push_back(std::get<0>(coords));
              mpc_y_vals.push_back(std::get<1>(coords));
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.0;
          int num_points = 20;
          
          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << std::endl << msg << std::endl;
          
          // Artifical Actuator Latency
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
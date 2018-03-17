#define _USE_MATH_DEFINES
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <cmath>

// PID regulator initial constants
// Tuning by Ziegler-Nichols method gives,
// see https://en.wikipedia.org/wiki/PID_controller#Ziegler%E2%80%93Nichols_method
// P=0.2 : oscillates a bit, 0.1 less, 0,05 barely => Ku=0.1
// Each oscillation is about 280 entries => Tu=280
// Gives these PID parameters:
// Kp = 0.6 Ku = 0.06
// Ki = 1.2 Ku / Tu = 0.00043
// Kd = 3 Ku Tu / 40 = 1.2
// That passed the whole track but with wheels on/close to the curb too much to feel safe,
// was a bit too "soft" though in long curves. Which means Kp and Ki could be increased a bit.
// Manual adjustments then result in:
// Kp = 0.1
// Ki = 0.0008
// Kd = 2.0
//
// Then tuned Kd, added dynamic throttling, 0.2 is fine at curves or when off-centre,
// but where steering around 0 and in the middle of the road, throttle 0.9 is fine

constexpr double KpInit = 0.1;
constexpr double KiInit = 0.0008;
constexpr double KdInit = 3.0;

constexpr bool debug = false;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(const std::string &s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_last_of(']');
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  pid.Init(KpInit, KiInit, KdInit);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (!s.empty()) {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const double cte = std::stod(j[1]["cte"].get<std::string>());
          const double speed = std::stod(j[1]["speed"].get<std::string>());
          const double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          pid.UpdateError(cte);
          const double steer_value = pid.steer_value();

          if (debug) {
            std::cout << "CTE: " << cte
                      << " PID Total error: " << pid.TotalError()
                      << " P I D-errors: " << pid.p_error << ", " << pid.i_error << ", " << pid.d_error
                      << " Steering Value: " << steer_value
                      << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;

          // Throttle control, if curvy or off-center, slow down,
          // otherwise give lots of gas >8-D
          double throttle;
          // abs(cte) = 1.6 near the curb/edge of road
          if (fabs(steer_value) > 0.1 || fabs(cte) > 0.5) {
            throttle = 0.2;
          } else {
            throttle = 0.9;
          }
          msgJson["throttle"] = throttle;

          // Control the car
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (debug) {
            std::cout << msg << std::endl;
          }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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

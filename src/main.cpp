#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void setMyParam(PID pidx, int idx, double pd_input) {
  if (idx == 0) {
    pidx.Kp_ = pd_input;
  } else if (idx == 1) {
    pidx.Ki_ = pd_input;
  } else {
    pidx.Kd_ = pd_input;
  }
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.

  // The below initialization values were found manually by trial and error
  pid.Init(0.07,0.002,2);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          std::cout << endl << endl << " ------------------ start new ------------------" << endl << std::endl;
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          std::cout << pid.prev_cte << "prev" << std::endl;
          std::cout << cte << "after" << std::endl;
          std::cout << pid.Kp_ << " p val" << std::endl;
          std::cout << pid.Ki_ << " i val" << std::endl;
          std::cout << pid.Kd_ << " d val" << std::endl;
          std::cout << pid.p_error << " p error" << std::endl;
          std::cout << pid.i_error << " i error" << std::endl;
          std::cout << pid.d_error << " d error" << std::endl;

          pid.UpdateError(cte);
          double current_error = cte*cte;
          cout << pid.best_error << " this is best error" << endl;
          cout << current_error << " this is correct error" << endl;
          cout << pid.p_param_error_high << " this is pid  error high flag" << endl;
          double sumDp = pid.dp[0] + pid.dp[1] + pid.dp[2];
          cout << pid.dp[0] << endl;
          cout << pid.dp[1] << endl;
          cout << pid.dp[2] << endl;

          /*  this is my twiddle experiment which needs to be explored further
          double threshold =0.000001;

          if (pid.train_p_param) {
            if (sumDp > threshold) {
              if (pid.p_param_error_high) {
                pid.Kp_ += pid.dp[0];
              }
              if (current_error < pid.best_error && !pid.p_param_error_high) {
                pid.best_error = current_error;
                pid.dp[0] *= 1.1;
                pid.p_param_error_high = true;
                pid.train_p_param = false;
                pid.train_i_param = true;
              } else if (current_error >= pid.best_error && !pid.p_param_error_high) {
                pid.Kp_ += pid.dp[0];
                pid.dp[0] *= 0.9;
                pid.p_param_error_high = true;
                pid.train_p_param = false;
                pid.train_i_param = true;
              } else if (current_error < pid.best_error && pid.p_param_error_high) {
                pid.best_error = current_error;
                pid.dp[0] *= 1.1;
                pid.train_p_param = false;
                pid.train_i_param = true;
              } else if (current_error >= pid.best_error && pid.p_param_error_high){
                pid.Kp_ -= 2*pid.dp[0];
                pid.p_param_error_high = false;
              }
            }
          }else if (pid.train_i_param) {
            if (sumDp > threshold) {
              if (pid.i_param_error_high) {
                pid.Ki_ += pid.dp[1];
              }
              if (current_error < pid.best_error && !pid.i_param_error_high) {
                pid.best_error = current_error;
                pid.dp[1] *= 1.1;
                pid.i_param_error_high = true;
                pid.train_i_param = false;
                pid.train_d_param = true;
              } else if (current_error >= pid.best_error && !pid.i_param_error_high) {
                pid.Ki_ += pid.dp[1];
                pid.dp[1] *= 0.9;
                pid.i_param_error_high = true;
                pid.train_i_param = false;
                pid.train_d_param = true;
              } else if (current_error < pid.best_error && pid.i_param_error_high) {
                pid.best_error = current_error;
                pid.dp[1] *= 1.1;
                pid.train_i_param = false;
                pid.train_d_param = true;
              } else if (current_error >= pid.best_error && pid.i_param_error_high){
                pid.Ki_ -= 2*pid.dp[1];
                cout << "am i stuck here" << endl;
                pid.i_param_error_high = false;
              }
            }
          }else if (pid.train_d_param) {
            if (sumDp > threshold) {
              if (pid.d_param_error_high) {
                pid.Kd_ += pid.dp[2];
              }
              if (current_error < pid.best_error && !pid.d_param_error_high) {
                pid.best_error = current_error;
                pid.dp[2] *= 1.1;
                pid.d_param_error_high = true;
                pid.train_p_param = true;
                pid.train_d_param = false;
              } else if (current_error >= pid.best_error && !pid.d_param_error_high) {
                pid.Kd_ += pid.dp[2];
                pid.dp[2] *= 0.9;
                pid.d_param_error_high = true;
                pid.train_p_param = true;
                pid.train_d_param = false;
              } else if (current_error < pid.best_error && pid.d_param_error_high) {
                pid.best_error = current_error;
                pid.dp[2] *= 1.1;
                pid.train_p_param = true;
                pid.train_d_param = false;
              } else if (current_error >= pid.best_error && pid.d_param_error_high){
                pid.Kd_ -= 2*pid.dp[2];

                pid.d_param_error_high = false;
              }
            }
          } */

          // twiddle
          /*

          double threshold = 0.00001;
          while (sumDp > threshold) {
            for (int i = 0; i<pid.dp.size(); i++) {
              cout << pid.dp[0] << endl;
              pid.p[i] += pid.dp[i];
              setMyParam(pid, i, pid.p[i]);
              pid.UpdateError(cte);
              double err = pid.TotalError();
              //double err = pid.total_error;
              if (err < best_error) {
                best_error = err;
                pid.dp[i] *= 1.1;
              } else {
                pid.p[i] -= 2*pid.dp[i];
                setMyParam(pid, i, pid.p[i]);
                pid.UpdateError(cte);
                double err = pid.TotalError();
                //err = pid.total_error;
                if (err < best_error) {
                  best_error = err;
                  pid.dp[i] *= 1.1;
                } else {
                  pid.p[i] += pid.dp[i];
                  setMyParam(pid, i, pid.p[i]);
                  pid.dp[i] *= 0.9;

                }
              }
            }
            sumDp = pid.dp[0] + pid.dp[1] + pid.dp[2];
          }
          */

          std::cout << pid.Kp_ << " p val" << std::endl;
          //std::cout << pid.p[0] << "ss p val" << std::endl;
          std::cout << pid.Ki_ << " i val" << std::endl;
          //std::cout << pid.p[1] << "ss p val" << std::endl;
          std::cout << pid.Kd_ << " d val" << std::endl;
          //std::cout << pid.p[2] << "ss p val" << std::endl;

          if (pid.TotalError() <-1) {
            steer_value = 1;
          } else if (pid.TotalError()>1) {
            steer_value = -1;
          } else {
            steer_value = -1*pid.TotalError();
          }


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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

#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#define DEBUG_SWITCH false
#define ENABLE_TWIDDLE false

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int Twiddle_state = 1;
//1: Search upper bound
//2: Update upper_err and Search lower bound
//3: update best err and update search range
int Twiddle_para = 0;
//0: Kp
//1: Ki
//2: Kd

double dK[3] = {0.01, 0.00001, 0.01};

double best_err = 9999;
double upper_err;
double lower_err;

bool init_err = true;
int Twiddle_count = 0;
double speed_control = 15.0;
int lap_steps = int(50000 / speed_control);


void Twiddle(PID &pid, double err) {
  std::cout<<"------------------------------"<<std::endl;
  std::cout<<"Twiddle session: #"<<Twiddle_count<<"\tTState:"<<Twiddle_state<<"\tPara:"<<Twiddle_para<<std::endl;
  Twiddle_count++;
  std::cout<<"Current K:";
  pid.PrintK();
  std::cout<<"Error:"<<err<<"\tBest error:" << best_err<<std::endl;
  if(Twiddle_state == 1) {
    pid.K[Twiddle_para] += dK[Twiddle_para];
    Twiddle_state++;
  }else if(Twiddle_state == 2) {
    upper_err = err;
    pid.K[Twiddle_para] -= 2*dK[Twiddle_para];
    Twiddle_state++;
  }else if(Twiddle_state == 3) {
    lower_err = err;

    if(upper_err > lower_err) {
      if(lower_err < best_err) {
        best_err = lower_err;
        dK[Twiddle_para] *= 1.1;
      }else{
        pid.K[Twiddle_para] += dK[Twiddle_para];
        dK[Twiddle_para] *= 0.9;
      }
    }else{
      if(upper_err < best_err) {
        best_err = upper_err;
        dK[Twiddle_para] *= 1.1;
      }else{
        pid.K[Twiddle_para] += dK[Twiddle_para];
        dK[Twiddle_para] *= 0.9;
      }
    }

    if(dK[0] < 0.01 && dK[1] < 0.000001 && dK[2] < 0.001) {
      std::cout<<"*****************************"<<std::endl;
      speed_control += 2;
      std::cout<<"New speed:"<<speed_control<<std::endl;
      lap_steps = int(50000 / speed_control);
      dK[0] = 0.01;
      dK[1] = 0.00001;
      dK[2] = 0.01;
      best_err = 9999;
      Twiddle_state=0;
      Twiddle_para = 0;
    } else {
      Twiddle_state=1;
      Twiddle_para = (Twiddle_para + 1) % 3;
      pid.K[Twiddle_para] += dK[Twiddle_para];
      Twiddle_state++;
    }

    
  }
  pid.Init(pid.K[0], pid.K[1] , pid.K[2]);
  std::cout<<"Updated K:";
  pid.PrintK();
}

int main() {
  uWS::Hub h;

  PID pid;
  
  double steer_value;
  double throttle_value;


  int step_count = 0;

  // 0: 
  //4900 at 10mph

  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(-0.202, -0.000147787, -0.221);

  h.onMessage([&pid, &throttle_value, &steer_value, &step_count](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());


          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // Speed control
          if(speed < speed_control) {
            throttle_value = 0.3;
          }
          else {
            throttle_value = 0;
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();


          // DEBUG
          if(DEBUG_SWITCH){
            std::cout << "#" << step_count << "\tSteering Value: " << steer_value << std::endl;

          }

          // Twiddle
          if(ENABLE_TWIDDLE) {
            step_count++;
            if(step_count > lap_steps) {
              double err = pid.MSE();
              if(init_err==true) {
                best_err = err;
                init_err = false;
              }
              Twiddle(pid, err);
              step_count = 0;
            }
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
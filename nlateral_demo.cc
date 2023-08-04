// Copyright 2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

#include <sys/mman.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "moteus_protocol.h"
#include "pi3hat_moteus_interface.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace {
template <typename Vector, typename KeyGetter>
double Average(const Vector& vector, KeyGetter key_getter) {
  double total = 0.0;
  double count = 0;
  for (const auto& item : vector) {
    total += key_getter(item);
    count += 1;
  }
  return total / count;
}

std::vector<std::string> Split(const std::string str) {
  std::vector<std::string> result;

  size_t start = 0;
  auto pos = str.find(',');
  while (pos != std::string::npos) {
    result.push_back(str.substr(start, pos - start + 1));
    start = pos + 1;
    pos = str.find(',', start);
  }
  result.push_back(str.substr(start));
  return result;
}

struct Servo {
  int id = -1;
  int bus = 1;
  double position_scale = 1.0;
  double force_scale = 1.0;

  static Servo Parse(const std::string& message) {
    auto fields = Split(message);
    Servo result;
    result.id = std::atol(fields[0].c_str());
    fields[0].erase(fields[0].begin());
    for (const auto& field : fields) {
      if (field.at(0) == 'b') {
        result.bus = std::stol(field.substr(1));
      } else if (field.at(0) == 'p') {
        result.position_scale = std::stod(field.substr(1));
        if (std::isfinite(result.position_scale) &&
            result.position_scale > 0.25 &&
            result.position_scale < 16.0) {
          // good
        } else {
          throw std::runtime_error("Position scale out of range: " + field);
        }
      } else if (field.at(0) == 'f') {
        result.force_scale = std::stod(field.substr(1));

        if (std::isfinite(result.force_scale) &&
            result.force_scale > 0.25 &&
            result.force_scale < 4.0) {
          // good
        } else {
          throw std::runtime_error("Force scale out of range: " + field);
        }
      } else {
        throw std::runtime_error("Unknown option: " + field);
      }
    }
    return result;
  }
};

struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--main-cpu") {
        main_cpu = std::stoull(args.at(++i));
      } else if (arg == "--can-cpu") {
        can_cpu = std::stoull(args.at(++i));
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
      } else if (arg == "-s" || arg == "--servo") {
        servos.push_back(Servo::Parse(args.at(++i)));
      } else if (arg == "--kp") {
        kp = std::stod(args.at(++i));
      } else if (arg == "--kd") {
        kd = std::stod(args.at(++i));
      } else if (arg == "--max-torque") {
        max_torque = std::stod(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  bool help = false;
  int main_cpu = 1;
  int can_cpu = 2;
  double period_s = 0.002;
  double kp = 0.0;
  double kd = 0.0;
  double max_torque = 0.3;
  std::vector<Servo> servos;
};

void DisplayUsage() {
  std::cout << "Usage: nlateral_demo [options]\n";
  std::cout << "\n";
  std::cout << "  -h, --help           display this usage message\n";
  std::cout << "  --main-cpu CPU       run main thread on a fixed CPU [default: 1]\n";
  std::cout << "  --can-cpu CPU        run CAN thread on a fixed CPU [default: 2]\n";
  std::cout << "  --period-s S         period to run control\n";
  std::cout << "  --kp XX.X            select kp value\n";
  std::cout << "  --kd XX.X            select kd value\n";
  std::cout << "  --max-torque XX.X    maximum torque to apply to a servo\n";
  std::cout << "  -s, --servo CFG      add one servo to be controlled\n";
  std::cout << "   CFG=ID[,option]...\n";
  std::cout << "    bN - pi3hat bus number N (default 1)\n";
  std::cout << "    pXX.X - scale position by this positive float\n";
  std::cout << "    fXX.X - scale force by this positive float\n";
  std::cout << "\n";
  std::cout << "Example w/ two moteus devkit motors on ID 1 and 2:\n";
  std::cout << "  sudo ./nlateral_demo -s 1 -s 2 --period-s 0.001 --kp 1.0 --kd 0.01\n";

}

void LockMemory() {
  // We lock all memory so that we don't end up having to page in
  // something later which can take time.
  {
    const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
    if (r < 0) {
      throw std::runtime_error("Error locking memory");
    }
  }
}

std::pair<double, double> MinMaxVoltage(
    const std::vector<MoteusInterface::ServoReply>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.result.voltage > rmax) { rmax = i.result.voltage; }
    if (i.result.voltage < rmin) { rmin = i.result.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

/// This holds the user-defined control logic.
class NLateralController {
 public:
  NLateralController(const Arguments& arguments) : arguments_(arguments) {
    for (const auto& servo : arguments.servos) {
      servos_[servo.id] = servo;
    }
  }

  /// This is called before any control begins, and must return the
  /// set of servos that are used, along with which bus each is
  /// attached to.
  std::map<int, int> servo_bus_map() const {
    if (arguments_.servos.size() < 2) {
      throw std::runtime_error(
          "At least 2 servos required, (specify -s more than once)\n");
    }

    std::map<int, int> result;
    for (const auto& servo : arguments_.servos) {
      if (result.count(servo.id)) {
        throw std::runtime_error("servo ID present multiple times");
      }
      result[servo.id] = servo.bus;
    }
    return result;
  }

  double torque(int id) const {
    const auto it = torques_.find(id);
    if (it == torques_.end()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return it->second;
  }

  /// This is also called before any control begins.  @p commands will
  /// be pre-populated with an entry for each servo returned by
  /// 'servo_bus_map'.  It can be used to perform one-time
  /// initialization like setting the resolution of commands and
  /// queries.
  void Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kIgnore;
    res.velocity = moteus::Resolution::kIgnore;
    res.feedforward_torque = moteus::Resolution::kFloat;
    res.kp_scale = moteus::Resolution::kInt8;
    res.kd_scale = moteus::Resolution::kInt8;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;

    for (auto& cmd : *commands) {
      cmd.resolution = res;
    }
  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id) {
    for (const auto& item : replies) {
      if (item.id == id) { return item.result; }
    }
    return {};
  }

  /// This is run at each control cycle.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).
  void Run(const std::vector<MoteusInterface::ServoReply>& status,
           std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;

    // Capture our initial positions.
    for (const auto& status_servo : status) {
      if (initial_positions_.count(status_servo.id) == 0 &&
          std::isfinite(status_servo.result.position)) {
        initial_positions_[status_servo.id] = status_servo.result.position;
      }
    }

    if (cycle_count_ < 5 && fault_) {
      for (auto& cmd : *output) {
        cmd.mode = moteus::Mode::kZeroVelocity;
      }
      return;
    }

    // For at least 5 cycles, or until we have heard from all servos,
    // just command stop.
    if (cycle_count_ < 5 ||
        initial_positions_.size() != arguments_.servos.size()) {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
      return;
    }

    if (status.size() != arguments_.servos.size()) {
      // We don't have data from all servos.  Just don't command
      // anything this cycle.  Hopefully the watchdog timeout will
      // keep us from running away.
      return;
    }

    if (cycle_count_ > 5 && !fault_) {
      // If any servo reports a fault, we fault all of them.
      for (const auto& servo_status : status) {
        if (servo_status.result.mode == moteus::Mode::kFault) {
          std::cout << "\n\nFault!  Servo " << servo_status.id
                    << " reports fault " << servo_status.result.fault
                    << "\n\n";
          fault_ = true;
          return;
        }
      }
    }

    // The n-lateral control law is to move all servos toward the
    // average of their positions and velocities using the
    // configured kp/kd constants.
    const double average_position = Average(
        status,
        [&](const auto& s) {
          return (s.result.position - initial_positions_.at(s.id)) *
              servos_.at(s.id).position_scale;
        });
    const double average_velocity = Average(
        status,
        [&](const auto& s) {
          return s.result.velocity * servos_.at(s.id).position_scale;
        });

    if (0) {
      std::cout << "avg pos: " << average_position << "  "
                << "avg_vel: " << average_velocity << "  ";
    }

    for (auto& cmd : *output) {
      const auto result = Get(status, cmd.id);
      const auto& servo = servos_.at(cmd.id);

      const double pos = (result.position -
                          initial_positions_.at(cmd.id)) * servo.position_scale;
      const auto p = -arguments_.kp * (pos - average_position);
      const auto d = -arguments_.kd * (result.velocity - average_velocity);

      const auto unlimited_torque = (p + d) * servo.force_scale;
      const auto torque =
          (unlimited_torque < -arguments_.max_torque) ?
          -arguments_.max_torque :
          (unlimited_torque > arguments_.max_torque) ?
          arguments_.max_torque :
          unlimited_torque;

      cmd.mode = moteus::Mode::kPosition;
      cmd.position.feedforward_torque = torque;
      cmd.position.kp_scale = 0.0;
      cmd.position.kd_scale = 0.0;

      torques_[cmd.id] = torque;

      if (0) {
        std::cout << cmd.id << ":" << torque << " ";
      }
    }
    if (0) {
      std::cout << "\n";
    }
  }

  bool fault() const { return fault_; }

 private:
  const Arguments arguments_;
  uint64_t cycle_count_ = 0;
  std::map<int, double> initial_positions_;
  std::map<int, double> torques_;
  std::map<int, Servo> servos_;
  bool fault_ = false;
};

template <typename Controller>
void Run(const Arguments& args, Controller* controller) {
  if (args.help) {
    DisplayUsage();
    return;
  }

  moteus::ConfigureRealtime(args.main_cpu);
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = args.can_cpu;
  moteus_options.servo_bus_map = controller->servo_bus_map();
  moteus_options.enable_aux = false;
  MoteusInterface moteus_interface{moteus_options};

  std::vector<MoteusInterface::ServoCommand> commands;
  for (const auto& pair : moteus_options.servo_bus_map) {
    commands.push_back({});
    commands.back().id = pair.first;
  }

  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  controller->Initialize(&commands);

  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;

  const auto period =
      std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  const auto status_period = std::chrono::milliseconds(100);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;

  // We will run at a fixed cycle time.
  while (true) {
    cycle_count++;
    margin_cycles++;
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        // NOTE: iomanip is not a recommended pattern.  We use it here
        // simply to not require any external dependencies, like 'fmt'.
        const auto volts = MinMaxVoltage(saved_replies);
        const std::string modes = [&]() {
          std::ostringstream result;
          result.precision(4);
          result << std::fixed;
          for (const auto& item : saved_replies) {
            result << item.id << "/"
                   << static_cast<int>(item.result.mode) << "/"
                   << item.result.position << "/"
                   << controller->torque(item.id)
                   << " ";
          }
          return result.str();
        }();
        std::cout << std::setprecision(6) << std::fixed
                  << "Cycles " << cycle_count
                  << "  margin: " << (total_margin / margin_cycles)
                  << std::setprecision(1)
                  << "  volts: " << volts.first << "/" << volts.second
                  << "  modes: " << modes
                  << "   \r";
        std::cout.flush();
        next_status += status_period;
        total_margin = 0;
        margin_cycles = 0;
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
        std::cout << "\nSkipped " << skip_count << " cycles\n";
      }
    }
    // Wait for the next control cycle to come up.
    {
      const auto pre_sleep = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next_cycle);
      const auto post_sleep = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
      total_margin += elapsed.count();
    }
    next_cycle += period;


    controller->Run(saved_replies, &commands);


    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new
      // one.
      const auto current_values = can_result.get();

      // We copy out the results we just got out.
      const auto rx_count = current_values.query_result_size;
      saved_replies.resize(rx_count);
      std::copy(replies.begin(), replies.begin() + rx_count,
                saved_replies.begin());
    }

    // Then we can immediately ask them to be used again.
    auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
    moteus_interface.Cycle(
        moteus_data,
        [promise](const MoteusInterface::Output& output) {
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();
  }
}
}

int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  LockMemory();

  NLateralController controller{args};
  Run(args, &controller);

  return 0;
}

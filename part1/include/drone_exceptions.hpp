#ifndef DRONE_EXCEPTIONS_HPP
#define DRONE_EXCEPTIONS_HPP

#include <stdexcept>
#include <string>

class DroneException : public std::runtime_error {
public:
  explicit DroneException(const std::string &message) : std::runtime_error(message) {}
};

class BatteryDepletedError : public DroneException {
public:
  explicit BatteryDepletedError(const std::string &message) : DroneException(message) {}
};

class InvalidStateError : public DroneException {
public:
  explicit InvalidStateError(const std::string &message) : DroneException(message) {}
};

class AltitudeError : public DroneException {
public:
  explicit AltitudeError(const std::string &message) : DroneException(message) {}
};

#endif
/* 
    - Utils file for the Medussa project
*/

#ifndef UTILS_CONSOLE_H
#define UTILS_CONSOLE_H
#include <string>
#include <array>
#include <memory>
#include <unistd.h>
#include <exception>
 
enum Calibration{cal_voltage, cal_current, cal_freq_factor, cal_imu_offset, cal_trim_turn};
 
/* Functions */
extern const char* bool2c(bool var);
extern bool fileExist(std::string path);
extern std::string executeShellCommand(std::string command);
extern void saveCalibrationOnserver(Calibration cal,  std::string value);

/* Env. Variables Functions */
extern int getEnv(const char* var, int default_var);
extern bool getEnv(const char* var, bool default_var);
extern float getEnv(const char* var, float default_var);
extern std::string getEnv(const char* var, const char* default_var);
 
#endif  /* End of UTILS_CONSOLE_H */
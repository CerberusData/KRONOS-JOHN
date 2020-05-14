/*
    - Console Utils
*/

#include "utils/console.hpp" 
 
bool fileExist(std::string path)
{
   return !(access(path.c_str(), R_OK) < 0);
}
 
std::string executeShellCommand(std::string command)
{ 
    std::array<char, 1024> buffer;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    std::string out = "";
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
 
    while (!feof(pipe.get()))
    {
        if (fgets(buffer.data(), 1024, pipe.get()) != NULL)
        {
            out = out + buffer.data();
        }
    }
 
    if (out.compare("") != 0)
    {
        out = out.substr(0, out.size() - 1);
    }
    return out;
}
 
void saveCalibrationOnserver(Calibration cal, std::string value)
{
    std::string calibration_param;
    switch (cal)
    {
    case cal_voltage:
        calibration_param="batVoltageFactor";
        break;
    case cal_current:
        calibration_param="batCurrentFactor";
        break;
    case cal_freq_factor:
        calibration_param= "pwmFrequencyFactor";
        break;
    case cal_imu_offset:
        calibration_param= "imuOffset";
        break;
    case cal_trim_turn:
        calibration_param= "trimTurn";
        break;
    }
    
    if(getenv("LIME_TOKEN") == NULL)
    { 
        printf("ERROR saving calibration to server LIME TOKEN Env. Var missing\n"); 
        return;
    }

    char* limeToken = getenv("LIME_TOKEN");
    
    std::string init = "curl -H 'Content-Type: application/json'  -H 'Authorization: " + std::string(limeToken) + "'";
    std::string data_values = "\"" +  calibration_param + "\":" + value;
    std::string data = "-d '{" + data_values + "}'";
    std::string server_url = getEnv("HTTP_SERVER_URL", "api.kiwicampus.com");
    std::string kiwibot_id = getEnv("KIWIBOT_ID", "327");
    std::string post = init + " " + data + " -X PATCH  https://" + server_url+ "/v2/bots/kiwibot" + kiwibot_id + "/configuration" ;
    executeShellCommand(post); 
}
 
 
extern const char* bool2c(bool var)
{
    return var ? "TRUE" : "FALSE";
}
 
extern int getEnv(const char* var, int default_var)
{
    try
    {
        return std::stoi(getenv(var));
    }
    catch (const std::exception& e)
    {
        return default_var;
    }   
}
 
extern std::string getEnv(const char* var, const char* default_var)
{
    try
    {
        return std::string(getenv(var));
    }
    catch (const std::exception& e)
    {
        return default_var;
    }
}
 
extern bool getEnv(const char* var, bool default_var)
{
    try
    {
        return std::string(getenv(var)).compare("1")==0;
    }
    catch (const std::exception& e)
    {
        return default_var;
    }
}
 
extern float getEnv(const char* var, float default_var)
{
    try
    {
        return std::stof(getenv(var));
    }
    catch (const std::exception& e)
    {
        return default_var;
    }     
}
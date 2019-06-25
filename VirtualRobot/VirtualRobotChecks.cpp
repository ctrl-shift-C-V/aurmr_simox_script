#include "VirtualRobotChecks.h"


namespace VirtualRobot
{

    VirtualRobotCheckException::VirtualRobotCheckException(
            const std::string& condition,
            const std::string& file, int line, const std::string& function, 
            const std::string& hint) :
        VirtualRobotException(makeMsg(condition, file, line, function, hint))
    {}
    
    std::string VirtualRobotCheckException::makeMsg(
            const std::string& condition, 
            const std::string& file, int line, const std::string& function, 
            const std::string& hint,
            const std::string& lhs, const std::string& rhs)
    {
        std::stringstream ss;
        ss << "Condition '" << condition << "' failed.";
        
        if (!(lhs.empty() && rhs.empty()))
        {
            ss << "\n(lhs = " << lhs << ", rhs = " << rhs << ")";
        }
        
        if (!hint.empty())
        {
            ss << "\n Hint: " << hint;
        }
        
        ss << "\n(line " << line << " in " << function << "() in " << file << ")" ;
        
        return ss.str();
    }

}

#pragma once

#include <sstream>

#include "VirtualRobotException.h"


namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT VirtualRobotCheckException :
            public VirtualRobotException
    {
    public:
        
        VirtualRobotCheckException(
                const std::string& condition,
                const std::string& filename, int line, const std::string& function,
                const std::string& hint = "");
        
        template <typename LhsT, typename RhsT>
        VirtualRobotCheckException(
                const std::string& condition, const LhsT& lhs, const RhsT& rhs,
                const std::string& filename, int line, const std::string& function,
                const std::string& hint = "") :
            VirtualRobotException(
                makeMsg(condition, filename, line, function, toString(lhs), toString(rhs), hint))
        {}
        
        
    private:
    
        template <typename T>
        static std::string toString(const T& t)
        {
            std::stringstream ss;
            ss << t;
            return ss.str();
        }
        
        static std::string makeMsg(
                const std::string& condition,
                const std::string& file, int line, const std::string& function,
                const std::string& hint = "",
                const std::string& lhs = "", const std::string& rhs = "");
        
    };


#define VR_CHECK_HINT(condition, hint) \
    do { \
        if( !(expression) ) { \
            throw ::VirtualRobot::VirtualRobotCheckException( \
                #condition, __FILE__, __LINE__, __FUNCTION__, hint); \
        } \
    } while(0);

    
#define VR_CHECK(condition) \
    VR_CHECK_HINT(condition, "")

    
#define VR_CHECK_COMPARISON_HINT(lhs, rhs, cmp, hint) \
    do { \
        if( !(lhs cmp rhs) ) {\
            throw ::VirtualRobot::VirtualRobotCheckException( \
                #lhs " " #cmp " " #rhs, lhs, rhs, __FILE__, __LINE__, __FUNCTION__, hint); \
        } \
    } while(0)
    
    
#define VR_CHECK_COMPARISON_HINT(lhs, rhs, cmp) \
    VR_CHECK_HINT(lhs, rhs, cmp, "")
    


#define VR_CHECK_EQUAL(lhs, rhs)                    VR_CHECK_COMPARISION(lhs, rhs, ==)
#define VR_CHECK_EQUAL_HINT(lhs, rhs, hint)         VR_CHECK_COMPARISION(lhs, rhs, ==, hint)
    
#define VR_CHECK_NOT_EQUAL(lhs, rhs)                VR_CHECK_COMPARISION(lhs, rhs, !=)
#define VR_CHECK_NOT_EQUAL_HINT(lhs, rhs, hint)     VR_CHECK_COMPARISION(lhs, rhs, !=, hint)

    
#define VR_CHECK_LESS(lhs, rhs)                     VR_CHECK_COMPARISION(lhs, rhs, <)
#define VR_CHECK_LESS_HINT(lhs, rhs, hint)          VR_CHECK_COMPARISION(lhs, rhs, <, hint)
    
#define VR_CHECK_LESS_EQUAL(lhs, rhs)               VR_CHECK_COMPARISION(lhs, rhs, <=)
#define VR_CHECK_LESS_EQUAL_HINT(lhs, rhs, hint)    VR_CHECK_COMPARISION(lhs, rhs, <=, hint)

    
#define VR_CHECK_GREATER(lhs, rhs)                  VR_CHECK_COMPARISION(lhs, rhs, >)
#define VR_CHECK_GREATER_HINT(lhs, rhs, hint)       VR_CHECK_COMPARISION(lhs, rhs, >, hint)

#define VR_CHECK_GREATER_EQUAL(lhs, rhs)            VR_CHECK_COMPARISION(lhs, rhs, >=)
#define VR_CHECK_GREATER_EQUAL_HINT(lhs, rhs, hint) VR_CHECK_COMPARISION(lhs, rhs, >=, hint)

    
#define VR_CHECK_NONNEGATIVE(value)                 VR_CHECK_GREATER_EQUAL(value, 0)
#define VR_CHECK_NONNEGATIVE_HINT(value, hint)      VR_CHECK_GREATER_EQUAL(value, 0, hint)

#define VR_CHECK_POSITIVE(value)                    VR_CHECK_GREATER(value, 0)
#define VR_CHECK_POSITIVE_HINT(value, hint)         VR_CHECK_GREATER(value, 0, hint)
    
    
#define VR_CHECK_FITS_SIZE(value, size) \ 
    VR_CHECK_NONNEGATIVE(value); \
    VR_CHECK_LESS(value, size)
    
#define VR_CHECK_FITS_SIZE_HINT(value, size, hint) \ 
    VR_CHECK_NONNEGATIVE(value, hint); \
    VR_CHECK_LESS(value, size, hint)



}

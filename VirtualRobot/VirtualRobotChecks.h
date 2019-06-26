#pragma once

#include <sstream>

#include "VirtualRobotException.h"


namespace VirtualRobot
{

    /**
     * @brief Exception class thrown by VR_CHECK_* macros ("soft" assertions).
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT VirtualRobotCheckException :
            public VirtualRobotException
    {
    public:
        
        /// Construct with condition and meta information as well as optional hint.
        VirtualRobotCheckException(
                const std::string& condition,
                const std::string& file, int line, const std::string& function,
                const std::string& hint = "");
        
        /// Construct with left- and right-hand-side operators.
        template <typename LhsT, typename RhsT>
        VirtualRobotCheckException(
                const std::string& condition, const LhsT& lhs, const RhsT& rhs,
                const std::string& file, int line, const std::string& function,
                const std::string& hint = "") :
            VirtualRobotException(
                makeMsg(condition, file, line, function, hint, toString(lhs), toString(rhs)))
        {}
        
        
    private:
    
        /// Convert `t` to a string using its << operator.
        template <typename T>
        static std::string toString(const T& t)
        {
            std::stringstream ss;
            ss << t;
            return ss.str();
        }
        
        /// Make the exception message.
        static std::string makeMsg(
                const std::string& condition,
                const std::string& file, int line, const std::string& function,
                const std::string& hint = "",
                const std::string& lhs = "", const std::string& rhs = "");
        
    };


/**
 * Check the given condition. 
 * @throw VirtualRobotCheckException If `condition``evaluates to false.
 */
#define VR_CHECK_HINT(condition, hint) \
    do { \
        if( !(condition) ) { \
            throw ::VirtualRobot::VirtualRobotCheckException( \
                #condition, __FILE__, __LINE__, __FUNCTION__, hint); \
        } \
    } while(0);

/**
 * Check the given condition. 
 * @throw VirtualRobotCheckException If `condition``evaluates to false.
 */    
#define VR_CHECK(condition) \
    VR_CHECK_HINT(condition, "")


#define VR_CHECK_COMPARISON_HINT(lhs, rhs, cmp, hint) \
    do { \
        if( !((lhs) cmp (rhs)) ) { \
            throw ::VirtualRobot::VirtualRobotCheckException( \
                #lhs " " #cmp " " #rhs, lhs, rhs, __FILE__, __LINE__, __FUNCTION__, hint); \
        } \
    } while(0)
    
    
#define VR_CHECK_COMPARISON(lhs, rhs, cmp) \
    VR_CHECK_COMPARISON_HINT(lhs, rhs, cmp, "")
    


#define VR_CHECK_EQUAL(lhs, rhs)                    VR_CHECK_COMPARISON(lhs, rhs, ==)
#define VR_CHECK_EQUAL_HINT(lhs, rhs, hint)         VR_CHECK_COMPARISON_HINT(lhs, rhs, ==, hint)
    
#define VR_CHECK_NOT_EQUAL(lhs, rhs)                VR_CHECK_COMPARISON(lhs, rhs, !=)
#define VR_CHECK_NOT_EQUAL_HINT(lhs, rhs, hint)     VR_CHECK_COMPARISON_HINT(lhs, rhs, !=, hint)


#define VR_CHECK_LESS(lhs, rhs)                     VR_CHECK_COMPARISON(lhs, rhs, <)
#define VR_CHECK_LESS_HINT(lhs, rhs, hint)          VR_CHECK_COMPARISON_HINT(lhs, rhs, <, hint)
    
#define VR_CHECK_LESS_EQUAL(lhs, rhs)               VR_CHECK_COMPARISON(lhs, rhs, <=)
#define VR_CHECK_LESS_EQUAL_HINT(lhs, rhs, hint)    VR_CHECK_COMPARISON_HINT(lhs, rhs, <=, hint)

    
#define VR_CHECK_GREATER(lhs, rhs)                  VR_CHECK_COMPARISON(lhs, rhs, >)
#define VR_CHECK_GREATER_HINT(lhs, rhs, hint)       VR_CHECK_COMPARISON_HINT(lhs, rhs, >, hint)

#define VR_CHECK_GREATER_EQUAL(lhs, rhs)            VR_CHECK_COMPARISON(lhs, rhs, >=)
#define VR_CHECK_GREATER_EQUAL_HINT(lhs, rhs, hint) VR_CHECK_COMPARISON_HINT(lhs, rhs, >=, hint)

    
#define VR_CHECK_NONNEGATIVE(value)                 VR_CHECK_GREATER_EQUAL(value, 0)
#define VR_CHECK_NONNEGATIVE_HINT(value, hint)      VR_CHECK_GREATER_EQUAL_HINT(value, 0, hint)

#define VR_CHECK_POSITIVE(value)                    VR_CHECK_GREATER(value, 0)
#define VR_CHECK_POSITIVE_HINT(value, hint)         VR_CHECK_GREATER_HINT(value, 0, hint)


#define VR_CHECK_FITS_SIZE(value, size) \
    VR_CHECK_NONNEGATIVE(value); \
    VR_CHECK_LESS(value, size)

#define VR_CHECK_FITS_SIZE_HINT(value, size, hint) \
    VR_CHECK_NONNEGATIVE_HINT(value, hint); \
    VR_CHECK_LESS_HINT(value, size, hint)



}

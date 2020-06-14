#include "string_tools.h"

#include "SimoxUtility/error/SimoxError.h"

#include <algorithm>

#include <iomanip>
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

namespace simox::alg
{

std::string to_lower(const std::string& str)
{
    std::string res = str;
    std::transform(res.begin(), res.end(), res.begin(), tolower);
    return res;
}

void trim(std::string& str, const std::locale& locale) {
    boost::trim(str, locale);
}

std::string trim_copy(const std::string& str, const std::locale& locale) {
    return boost::trim_copy(str, locale);
}

void trim_if(std::string& str, const std::string& trim) {
    boost::trim_if(str, boost::is_any_of(trim));
}

std::string trim_copy_if(std::string& str, const std::string& trim) {
    return boost::trim_copy_if(str, boost::is_any_of(trim));
}

std::vector<std::string> split(const std::string& str, const std::string& splitBy, bool trimElements, bool removeEmptyElements, const std::locale& locale)
{
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(splitBy));

    if (trimElements) std::for_each(strs.begin(), strs.end(), boost::bind(&boost::trim<std::string>, _1, locale));

    if (removeEmptyElements) strs.erase(std::remove_if(strs.begin(), strs.end(), [](const std::string& s) { return s.empty(); }), strs.end());

    return strs;
}

std::vector<std::string> split_check_size(const std::string& str, unsigned int expectedSize, const std::string& splitBy, bool trimElements, bool removeEmptyElements, const std::locale& locale)
{
    std::vector<std::string> strs = split(str, splitBy, trimElements, removeEmptyElements, locale);

    if (strs.size() != expectedSize)
        throw error::SimoxError("String " + str + " contains " + std::to_string(strs.size()) + " instead of " + std::to_string(expectedSize) + " values seperated by delimiter " + splitBy);
    return strs;
}

std::string join(const std::vector<std::string> vec, const std::string& delimiter, bool trimElements, bool ignoreEmptyElements, const std::locale& locale) {
    std::ostringstream ss;
    ss.imbue(locale);
    for (size_t index = 0; index < vec.size(); index++)
    {
        if (trimElements)
        {
            std::string trimmed = trim_copy(vec.at(index), locale);
            if (ignoreEmptyElements && trimmed.empty()) continue;
            ss << trimmed;
        }
        else
        {
            if (ignoreEmptyElements && vec.at(index).empty()) continue;
            ss << vec.at(index);
        }
        if (index != vec.size()) ss << delimiter;
    }
    return ss.str();
}

}

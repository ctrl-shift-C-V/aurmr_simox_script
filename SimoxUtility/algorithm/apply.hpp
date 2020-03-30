#pragma once

#include <functional>
#include <map>
#include <vector>


namespace simox
{


    template <typename ValueIn, typename ValueOut>
    ValueOut apply(const ValueIn& value, const std::function<ValueIn(ValueOut)>& op)
    {
        return op(value);
    }


    template <typename ValueIn, typename ValueOut>
    std::vector<ValueOut> apply(const std::vector<ValueIn>& vector, const std::function<ValueIn(ValueOut)>& op)
    {
        std::vector<ValueOut> result;
        std::transform(vector.begin(), vector.end(), std::back_inserter(result), [&op](const auto& v)
        {
            return apply(v, op);
        });
        return result;
    }


    template <typename ValueIn, typename ValueOut, typename Key>
    std::map<Key, ValueOut> apply(const std::map<Key, ValueIn>& map, const std::function<ValueIn(ValueOut)>& op)
    {
        std::map<Key, ValueOut> result;
        for (const auto& [name, value] : map)
        {
            result[name] = apply(value, op);
        }
        return result;
    }


}


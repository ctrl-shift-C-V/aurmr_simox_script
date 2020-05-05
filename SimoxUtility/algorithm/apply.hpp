#pragma once

#include <algorithm>
#include <map>
#include <type_traits>
#include <vector>


namespace simox::alg
{

    template <typename ValueIn, typename UnaryOp>
    std::vector<std::invoke_result_t<UnaryOp, ValueIn>>
    apply(const UnaryOp& op, const std::vector<ValueIn>& vector)
    {
        using ValueOut = std::invoke_result_t<UnaryOp, ValueIn>;
        std::vector<ValueOut> result;
        std::transform(vector.begin(), vector.end(), std::back_inserter(result), op);
        return result;
    }


    template <typename Key, typename ValueIn, typename UnaryOp>
    std::map<Key, std::invoke_result_t<UnaryOp, ValueIn>>
    apply(const UnaryOp& op, const std::map<Key, ValueIn>& map)
    {
        using ValueOut = std::invoke_result_t<UnaryOp, ValueIn>;
        std::map<Key, ValueOut> result;
        for (const auto& [name, value] : map)
        {
            result[name] = op(value);
        }
        return result;
    }


}

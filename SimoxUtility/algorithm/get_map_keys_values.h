#pragma once

#include <functional>
#include <map>
#include <set>
#include <vector>


namespace simox
{
    /// Get the keys of `map` in a vector.
    template <class K, class V, template<class...> class Template = std::map, class...Ts>
    std::vector<K> get_keys(const Template<K, V, Ts...>& map)
    {
        std::vector<K> keys;
        if constexpr(std::is_same_v<std::map<K, V, Ts...>, Template<K, V, Ts...>>)
        {
            keys.reserve(map.size());
        }
        for (const auto& [k, v] : map)
        {
            keys.push_back(k);
        }
        return keys;
    }


    /// Get the keys of `map` in a set.
    template <class K, class V, template<class...> class Template = std::map, class...Ts>
    std::set<K> get_keys_set(const Template<K, V, Ts...>& map)
    {
        std::set<K> keys;
        for (const auto& [k, v] : map)
        {
            keys.insert(k);
        }
        return keys;
    }


    /// Get the values of `map`.
    template <class K, class V, template<class...> class Template = std::map, class...Ts>
    std::vector<V> get_values(const Template<K, V, Ts...>& map)
    {
        std::vector<V> values;
        values.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            values.push_back(v);
        }
        return values;
    }


    /// Get the results of applying `unary_func` to the values of `map`.
    template <class R, class K, class V, template<class...> class Template = std::map, class...Ts>
    std::vector<V> get_values(const Template<K, V, Ts...>& map, std::function<R(const V&)> unary_func)
    {
        std::vector<V> values;
        values.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            values.push_back(unary_func(v));
        }
        return values;
    }
}

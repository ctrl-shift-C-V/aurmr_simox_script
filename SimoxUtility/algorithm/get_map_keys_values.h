#include <functional>
#include <map>
#include <set>
#include <vector>


namespace simox
{


    /// Get the keys of `map` in a vector.
    template <typename K, typename V>
    std::vector<K> get_keys(const std::map<K, V>& map)
    {
        std::vector<K> keys;
        keys.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            keys.push_back(k);
        }
        return keys;
    }


    /// Get the keys of `map` in a set.
    template <typename K, typename V>
    std::set<K> get_keys_set(const std::map<K, V>& map)
    {
        std::set<K> keys;
        for (const auto& [k, v] : map)
        {
            keys.insert(k);
        }
        return keys;
    }


    /// Get the values of `map`.
    template <typename K, typename V>
    std::vector<V> get_values(const std::map<K, V>& map)
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
    template <typename R, typename K, typename V>
    std::vector<V> get_values(const std::map<K, V>& map, std::function<R(const V&)> unary_func)
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

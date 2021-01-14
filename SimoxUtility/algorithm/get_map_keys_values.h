#pragma once

#include <functional>
#include <map>
#include <set>
#include <vector>
#include <type_traits>


namespace simox::alg
{
    /// Get the keys of `map` in a vector.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<K> get_keys(const MapT<K, V, Ts...>& map)
    {
        std::vector<K> keys;
        if constexpr(std::is_same_v<std::map<K, V, Ts...>, MapT<K, V, Ts...>>)
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
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::set<K> get_keys_set(const MapT<K, V, Ts...>& map)
    {
        std::set<K> keys;
        for (const auto& [k, v] : map)
        {
            keys.insert(k);
        }
        return keys;
    }


    /// Get the values of `map`.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<V> get_values(const MapT<K, V, Ts...>& map)
    {
        std::vector<V> values;
        values.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            values.push_back(v);
        }
        return values;
    }


    /// Get the results of applying `value_fn` to the values of `map`.
    template <class ValueFn, class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<std::invoke_result_t<ValueFn, const V&>>
    get_values(const MapT<K, V, Ts...>& map, ValueFn value_fn)
    {
        std::vector<std::invoke_result_t<ValueFn, const V&>> values;
        values.reserve(map.size());
        for (const auto& [k, v] : map)
        {
            values.push_back(value_fn(v));
        }
        return values;
    }
}



// Legacy definitions in old (general) namespace.
namespace simox
{
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<K> get_keys(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_keys(map);
    }
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::set<K> get_keys_set(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_keys_set(map);
    }
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<V> get_values(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_values(map);
    }
    template <class R, class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<V> get_values(const MapT<K, V, Ts...>& map, std::function<R(const V&)> value_fn)
    {
        return simox::alg::get_values(map, value_fn);
    }
}

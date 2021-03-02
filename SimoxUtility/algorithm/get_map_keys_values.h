#pragma once

#include <functional>
#include <map>
#include <set>
#include <type_traits>
#include <vector>


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
            keys.emplace_back(k);
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
            values.emplace_back(value_fn(v));
        }
        return values;
    }
    /// Get the results of applying `value_fn` to the values of `map`.
    template <class ValueFn, class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<std::invoke_result_t<ValueFn, V&>>
    get_values(MapT<K, V, Ts...>& map, ValueFn value_fn)
    {
        std::vector<std::invoke_result_t<ValueFn, V&>> values;
        values.reserve(map.size());
        for (auto& [k, v] : map)
        {
            values.emplace_back(value_fn(v));
        }
        return values;
    }


    /// Get the (const) pointers to the values of (const) `map`.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<const V*> get_value_ptrs(const MapT<K, V, Ts...>& map)
    {
        return get_values(map, [](const V& value)
        {
            return &value;
        });
    }
    /// Get the (non-const) pointers to the values of (non-const) `map`.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<V*> get_value_ptrs(MapT<K, V, Ts...>& map)
    {
        return get_values(map, [](V& value)
        {
            return &value;
        });
    }

    /// Get the const pointers to the values of `map`.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<const V*> get_value_cptrs(MapT<K, V, Ts...>& map)
    {
        return get_values(map, [](const V& value) { return &value; });
    }
    /// Get the const pointers to the values of `map`.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    std::vector<const V*> get_value_cptrs(const MapT<K, V, Ts...>& map)
    {
        return get_value_ptrs(map);
    }


    /// Get a value from `map` if it exsits or a default value otherwise.
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    V get_value_or_default(const MapT<K, V, Ts...>& map, const K& key, const V& default_value)
    {
        auto it = map.find(key);
        return it != map.end() ? it->second : default_value;
    }

    // overload for `std::string` to match `char[]`.
    template <class K, template<class...> class MapT = std::map, class...Ts>
    std::string get_value_or_default(const MapT<K, std::string, Ts...>& map, const K& key, const std::string& default_value)
    {
        auto it = map.find(key);
        return it != map.end() ? it->second : default_value;
    }

}



// Legacy definitions in old (general) namespace.
namespace simox
{
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    [[deprecated("Function has moved to namespace ::simox::alg.")]]
    std::vector<K> get_keys(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_keys(map);
    }
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    [[deprecated("Function has moved to namespace ::simox::alg.")]]
    std::set<K> get_keys_set(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_keys_set(map);
    }
    template <class K, class V, template<class...> class MapT = std::map, class...Ts>
    [[deprecated("Function has moved to namespace ::simox::alg.")]]
    std::vector<V> get_values(const MapT<K, V, Ts...>& map)
    {
        return simox::alg::get_values(map);
    }
    template <class R, class K, class V, template<class...> class MapT = std::map, class...Ts>
    [[deprecated("Function has moved to namespace ::simox::alg.")]]
    std::vector<V> get_values(const MapT<K, V, Ts...>& map, std::function<R(const V&)> value_fn)
    {
        return simox::alg::get_values(map, value_fn);
    }
}

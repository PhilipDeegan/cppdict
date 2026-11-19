#pragma once

/*
  Opt-in CBOR (RFC 8949) serialization of a cppdict::Dict, requires nlohmann/json.

  File layout, portable across endianness and ABIs:

    file  := { "format": "cppdict", "version": 1, "root": map | null }
    node  := map | leaf
    map   := { key: node, ... }
    leaf  := [ type_name, value ]

    value(bool)           := bool
    value(integral)       := integer      range checked on load
    value(floating point) := float
    value(std::string)    := text string
    value(sequence)       := [ value, ... ]  e.g. std::vector<double>, std::array<int, 3>,
                                             std::vector<std::string>; elements range checked

  User types are not supported: flatten them into maps beforehand (e.g. a static S FROM(Dict)).

  type_name is derived from the type itself, not from its position in Dict<Ts...>:
    "bool", "i8".."i64", "u8".."u64", "f32", "f64", "str", "vec<E>", "arr<E,N>"
  Prefer fixed width integers (std::int64_t, ...) as leaf types: `long` is "i64" on Linux but
  "i32" on Windows.
*/

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <nlohmann/json.hpp>

#include "../dict.hpp"

namespace cppdict::cbor
{
using json = nlohmann::json;

inline constexpr int format_version = 1;


template<typename Config>
std::vector<std::uint8_t> to_bytes(Config const& config);

template<typename Config>
Config from_bytes(std::vector<std::uint8_t> const& bytes);

template<typename Config>
void save_config(std::filesystem::path const& file, Config const& config);

template<typename Config>
Config load_config(std::filesystem::path const& file);

} // namespace cppdict::cbor


namespace cppdict::cbor::detail
{
template<typename T>
concept CharLike = std::same_as<T, char> or std::same_as<T, wchar_t> or std::same_as<T, char8_t>
                   or std::same_as<T, char16_t> or std::same_as<T, char32_t>;

// arithmetic types with a platform independent meaning
template<typename T>
concept PortableArithmetic
    = std::same_as<T, bool> or (std::integral<T> and !CharLike<T> and sizeof(T) <= 8)
      or (std::floating_point<T> and std::numeric_limits<T>::is_iec559
          and (sizeof(T) == 4 or sizeof(T) == 8));

template<typename T>
concept Scalar = PortableArithmetic<T> or std::same_as<T, std::string>;

// owning sized range of scalars, e.g. std::vector<double>, std::array<int, 3>
template<typename T>
concept Sequence = std::ranges::forward_range<T> and std::ranges::sized_range<T>
                   and !std::ranges::view<T> and std::default_initializable<T>
                   and !std::same_as<T, std::string> and Scalar<std::ranges::range_value_t<T>>;

template<typename T>
concept Resizable = requires(T& t, std::size_t n) { t.resize(n); };
} // namespace cppdict::cbor::detail


namespace cppdict::cbor
{
template<typename T>
concept CborSerializable = detail::Scalar<T> or detail::Sequence<T>;
} // namespace cppdict::cbor


namespace cppdict::cbor::detail
{
[[noreturn]] inline void fail(std::string const& what, std::string const& path);

template<typename T>
std::string type_name();

template<typename T>
std::string sequence_type_name();

template<typename T>
T decode_value(json const& j, std::string const& path);

template<typename T>
T decode_sequence(json const& j, std::string const& path);

template<typename T>
T decode_integral(json const& j, std::string const& path);

template<typename T>
T decode_floating(json const& j, std::string const& path);

// everything that needs the Ts... pack of a Dict
template<typename Dict>
struct LeafTypes;

template<typename... Ts>
struct LeafTypes<cppdict::Dict<Ts...>>
{
    static_assert(
        (CborSerializable<Ts> and ...),
        "cppdict: every leaf type must be bool, a fixed size integer, an IEEE float/double, "
        "std::string, or a sequence of those (std::vector, std::array)");

    static void check_unique_names();

    // calls f.template operator()<T>() for the leaf type T named `name`, false if none matches
    template<typename F>
    static bool dispatch(std::string const& name, F&& f);
};


template<typename Dict>
struct Codec
{
    using node_t       = typename Dict::node_t;
    using empty_leaf_t = typename Dict::empty_leaf_t;

    static json encode_node(Dict const& node, std::string const& path);
    static void decode_node(json const& j, Dict& node, std::string const& path);

private:
    static json encode_alternative(empty_leaf_t const&, std::string const& path);
    static json encode_alternative(node_t const& map, std::string const& path);
    template<typename T>
    static json encode_alternative(T const& leaf, std::string const& path);

    static void decode_leaf(std::string const& name, json const& value, Dict& node,
                            std::string const& path);
};


// ---------------------------------------------------------------- helpers

[[noreturn]] inline void fail(std::string const& what, std::string const& path)
{
    throw std::runtime_error("cppdict: " + what + " at '" + path + "'");
}

template<typename T>
std::string type_name()
{
    if constexpr (std::same_as<T, bool>)
        return "bool";
    else if constexpr (std::integral<T>)
        return (std::is_signed_v<T> ? "i" : "u") + std::to_string(8 * sizeof(T));
    else if constexpr (std::floating_point<T>)
        return "f" + std::to_string(8 * sizeof(T));
    else if constexpr (std::same_as<T, std::string>)
        return "str";
    else
        return sequence_type_name<T>();
}

template<typename T>
std::string sequence_type_name()
{
    auto const element = type_name<std::ranges::range_value_t<T>>();
    if constexpr (Resizable<T>)
        return "vec<" + element + ">";
    return "arr<" + element + "," + std::to_string(std::ranges::size(T{})) + ">";
}


template<typename... Ts>
void LeafTypes<cppdict::Dict<Ts...>>::check_unique_names()
{
    static auto const duplicate = []() -> std::string {
        std::vector<std::string> names{type_name<Ts>()...};
        std::ranges::sort(names);
        auto const it = std::ranges::adjacent_find(names);
        return it == names.end() ? "" : *it;
    }();

    if (!duplicate.empty())
        throw std::logic_error("cppdict: duplicate leaf type name '" + duplicate
                               + "', use distinct fixed width types");
}

template<typename... Ts>
template<typename F>
bool LeafTypes<cppdict::Dict<Ts...>>::dispatch(std::string const& name, F&& f)
{
    return ((name == type_name<Ts>() ? (f.template operator()<Ts>(), true) : false) or ...);
}


// ---------------------------------------------------------------- encode

template<typename Dict>
json Codec<Dict>::encode_node(Dict const& node, std::string const& path)
{
    return std::visit([&](auto const& value) { return encode_alternative(value, path); },
                      node.data);
}

template<typename Dict>
json Codec<Dict>::encode_alternative(empty_leaf_t const&, std::string const& path)
{
    fail("cannot serialize empty leaf", path);
}

template<typename Dict>
json Codec<Dict>::encode_alternative(node_t const& map, std::string const& path)
{
    json obj = json::object();
    for (auto const& [key, child] : map)
        obj[key] = encode_node(*child, path + "/" + key);
    return obj;
}

template<typename Dict>
template<typename T>
json Codec<Dict>::encode_alternative(T const& leaf, std::string const&)
{
    return json::array({type_name<T>(), json(leaf)});
}


// ---------------------------------------------------------------- decode

template<typename Dict>
void Codec<Dict>::decode_node(json const& j, Dict& node, std::string const& path)
{
    if (j.is_array() and j.size() == 2 and j[0].is_string())
        return decode_leaf(j[0].get_ref<std::string const&>(), j[1], node, path);

    if (!j.is_object())
        fail("expected a map or a [type, value] leaf", path);

    node.data = node_t{}; // keep empty maps as maps
    for (auto const& [key, value] : j.items())
        decode_node(value, node[key], path + "/" + key);
}

template<typename Dict>
void Codec<Dict>::decode_leaf(std::string const& name, json const& value, Dict& node,
                              std::string const& path)
{
    auto const decode = [&]<typename T>() { node = decode_value<T>(value, path); };
    if (!LeafTypes<Dict>::dispatch(name, decode))
        fail("unknown leaf type '" + name + "'", path);
}

template<typename T>
T decode_value(json const& j, std::string const& path)
{
    if constexpr (Sequence<T>)
        return decode_sequence<T>(j, path);
    else if constexpr (std::same_as<T, std::string>)
        return j.is_string() ? j.get<std::string>() : (fail("expected a string", path), T{});
    else if constexpr (std::same_as<T, bool>)
        return j.is_boolean() ? j.get<bool>() : (fail("expected a bool", path), false);
    else if constexpr (std::integral<T>)
        return decode_integral<T>(j, path);
    else
        return decode_floating<T>(j, path);
}

template<typename T>
T decode_sequence(json const& j, std::string const& path)
{
    if (!j.is_array())
        fail("expected an array", path);

    T t{};
    if constexpr (Resizable<T>)
        t.resize(j.size());
    else if (std::ranges::size(t) != j.size())
        fail("fixed size container length mismatch, file has " + std::to_string(j.size())
                 + " expected " + std::to_string(std::ranges::size(t)),
             path);

    auto it = std::ranges::begin(t);
    for (auto const& e : j)
        *it++ = decode_value<std::ranges::range_value_t<T>>(e, path);
    return t;
}

template<typename T>
T decode_integral(json const& j, std::string const& path)
{
    auto const narrow = [&](auto const v) {
        if (!std::in_range<T>(v))
            fail("integer " + std::to_string(v) + " out of range for " + type_name<T>(), path);
        return static_cast<T>(v);
    };

    if (j.is_number_unsigned())
        return narrow(j.get<std::uint64_t>());
    if (j.is_number_integer())
        return narrow(j.get<std::int64_t>());
    fail("expected an integer", path);
}

template<typename T>
T decode_floating(json const& j, std::string const& path)
{
    if (!j.is_number_float())
        fail("expected a float", path);
    auto const v = j.get<double>();
    if (std::isfinite(v) and std::abs(v) > std::numeric_limits<T>::max())
        fail("float " + std::to_string(v) + " out of range for " + type_name<T>(), path);
    return static_cast<T>(v);
}

} // namespace cppdict::cbor::detail


namespace cppdict::cbor
{


// ---------------------------------------------------------------- public API

template<typename Config>
std::vector<std::uint8_t> to_bytes(Config const& config)
{
    detail::LeafTypes<Config>::check_unique_names();
    if (!config.isNode() and !config.isEmpty())
        throw std::runtime_error("cppdict: root of a config must be a map");

    auto const root = config.isNode() ? detail::Codec<Config>::encode_node(config, "") : json{};
    return json::to_cbor(json{{"format", "cppdict"}, {"version", format_version}, {"root", root}});
}

template<typename Config>
Config from_bytes(std::vector<std::uint8_t> const& bytes)
{
    detail::LeafTypes<Config>::check_unique_names();

    auto const doc = json::from_cbor(bytes);
    if (!doc.is_object() or doc.value("format", "") != "cppdict")
        throw std::runtime_error("cppdict: not a cppdict CBOR document");
    if (!doc.contains("version") or doc["version"] != format_version)
        throw std::runtime_error("cppdict: unsupported cppdict CBOR format version");

    auto const& root = doc.value("root", json{});
    if (!root.is_object() and !root.is_null())
        throw std::runtime_error("cppdict: root of a config must be a map");

    Config config;
    if (root.is_object())
        detail::Codec<Config>::decode_node(root, config, "");
    return config;
}

template<typename Config>
void save_config(std::filesystem::path const& file, Config const& config)
{
    auto const bytes = to_bytes(config);
    std::ofstream out{file, std::ios::binary};
    if (!out)
        throw std::runtime_error("cppdict: cannot open file for writing: " + file.string());
    out.write(reinterpret_cast<char const*>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
    if (!out.flush())
        throw std::runtime_error("cppdict: write failed: " + file.string());
}

template<typename Config>
Config load_config(std::filesystem::path const& file)
{
    std::ifstream in{file, std::ios::binary};
    if (!in)
        throw std::runtime_error("cppdict: cannot open file for reading: " + file.string());
    std::vector<std::uint8_t> const bytes{std::istreambuf_iterator<char>{in}, {}};
    if (in.bad())
        throw std::runtime_error("cppdict: read failed: " + file.string());
    return from_bytes<Config>(bytes);
}

} // namespace cppdict::cbor

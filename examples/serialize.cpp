#include "dict.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>

using MyDict = cppdict::Dict<int, double, std::string>;


#define PRINT(x) std::cout << __LINE__ << " " << (x) << std::endl;


template<typename T>
void _read_data(T data, std::size_t size, std::ifstream& in)
{
    in.read(reinterpret_cast<char*>(data), size);
}

template<typename T>
void _write_data(T data, std::size_t size, std::ofstream& out)
{
    out.write(reinterpret_cast<char const* const>(data), size);
}



template<typename... Types>
struct DictSerializer
{
    using Dict                           = cppdict::Dict<Types...>;
    using data_t                         = typename Dict::data_t;
    using Tuple                          = std::tuple<Types...>;
    std::size_t constexpr static n_types = std::tuple_size_v<Tuple>;
    std::size_t constexpr static base    = 2; // 0 = map, 1 = map_key_string


    void _serialize_data(std::string const& s)
    {
        std::size_t const size = s.size();
        _write_data(&size, sizeof(size), out);
        _write_data(s.data(), size, out);
    }

    template<typename T>
    void _serialize_data(T const& data)
    {
        std::size_t const size = sizeof(T);
        _write_data(&data, size, out);
    }

    template<std::size_t I>
    void __serialize_variant(data_t const& data, bool& b)
    {
        using El = std::tuple_element_t<I, Tuple>;

        if (!b and std::holds_alternative<El>(data))
        {
            std::size_t const type = I + base;
            _write_data(&type, sizeof(type), out);
            _serialize_data(std::get<El>(data));
            b = 1;
        }
    }

    template<std::size_t... Is>
    void _serialize_variant(data_t const& data, std::integer_sequence<std::size_t, Is...>)
    {
        bool b = 0;
        (__serialize_variant<Is>(data, b), ...);
    }

    template<typename T, std::size_t I>
    void write_type_id(T const&, bool& b)
    {
        if (b)
            return;
        if constexpr (std::is_same_v<T, std::tuple_element_t<I, Tuple>>)
        {
            std::size_t const type = I + base;
            _write_data(&type, sizeof(type), out);
            b = 1;
        }
    }

    template<typename T, std::size_t... Is>
    void write_type_id(T const& t, std::integer_sequence<std::size_t, Is...>)
    {
        bool b = 0;
        (write_type_id<T, Is>(t, b), ...);
    }

    void operator()(Dict const& dict)
    {
        if (dict.isNode())
        {
            dict.visit(
                cppdict::visit_all_nodes,
                [&](const std::string& key, const typename Dict::node_t& v) {
                    {
                        std::size_t const type = 0;
                        std::size_t const keys = v.size();
                        std::size_t const size = key.size();
                        _write_data(&type, sizeof(type), out);
                        _write_data(&size, sizeof(type), out);
                        _write_data(&keys, sizeof(type), out);
                        _write_data(key.data(), size, out);
                    }

                    for (const auto& [key, node] : v)
                    {
                        std::size_t const type = 1;
                        std::size_t const size = key.size();
                        _write_data(&type, sizeof(type), out);
                        _write_data(&size, sizeof(type), out);
                        _write_data(key.data(), size, out);

                        (*this)(*node);
                    }
                },
                [&](const std::string&, const typename Dict::empty_leaf_t&) { PRINT(""); },
                [&](const std::string& key, const auto& v) {
                    std::size_t const type = 1;
                    std::size_t const size = key.size();
                    _write_data(&type, sizeof(type), out);
                    _write_data(&size, sizeof(type), out);
                    _write_data(key.data(), size, out);

                    write_type_id(v, std::make_integer_sequence<std::size_t, n_types>{});
                    _serialize_data(v);
                });
        }
        else if (dict.isValue())
        {
            _serialize_variant(dict.data, std::make_integer_sequence<std::size_t, n_types>{});
        }
    }

    std::string const filename;
    std::ofstream out{filename, std::ios::binary};
};


template<typename... Types>
struct DictDeSerializer
{
    using Dict                           = cppdict::Dict<Types...>;
    using data_t                         = typename Dict::data_t;
    using Tuple                          = std::tuple<Types...>;
    std::size_t constexpr static n_types = std::tuple_size_v<Tuple>;
    std::size_t constexpr static base    = 2; // 0 = map, 1 = map_key_string


    void _deserialize_string(Dict& node)
    {
        std::size_t size = 0;

        in.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

        std::string s;
        s.resize(size);
        _read_data(s.data(), size, in);

        node = s;
    }

    template<typename T>
    void _deserialize_data(Dict& node)
    {
        T d = 0;
        in.read(reinterpret_cast<char*>(&d), sizeof(T));
        node = d;
    }

    template<std::size_t I>
    void _read(Dict& node, std::size_t const& type)
    {
        using El = std::tuple_element_t<I, Tuple>;

        if (I + base == type)
        {
            if constexpr (std::is_same_v<std::string, El>)
            {
                _deserialize_string(node);
            }
            else
                _deserialize_data<El>(node);
        }
    }

    template<std::size_t... Is>
    void read(Dict& node, std::size_t const& type, std::integer_sequence<std::size_t, Is...>)
    {
        (_read<Is>(node, type), ...);
    }


    auto operator()()
    {
        Dict dict;

        std::function<void(Dict&)> read_chunk = [&](auto& node) {
            std::size_t type = 0;

            in.read(reinterpret_cast<char*>(&type), sizeof(std::size_t));

            if (type == 0)
            {
                std::size_t size = 0;
                in.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

                std::string s;
                s.resize(size);

                std::size_t keys = 0;
                in.read(reinterpret_cast<char*>(&keys), sizeof(std::size_t));

                _read_data(s.data(), size, in);

                node[s] = {};

                for (std::size_t i = 0; i < keys; ++i)
                    read_chunk(node[s]);
            }
            else if (type == 1)
            {
                std::size_t size = 0;
                in.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

                std::string s;
                s.resize(size);
                _read_data(s.data(), size, in);

                read_chunk(node[s]);
            }
            else
            {
                read(node, type, std::make_integer_sequence<std::size_t, n_types>{});
            }
        };

        while (!in.eof())
        {
            read_chunk(dict);
            in.peek();
        }

        return dict;
    }

    std::string const filename;
    std::ifstream in{filename, std::ios::binary};
};


int main()
{
    {
        MyDict md;
        md["test"]["super"] = 2;
        md["PI"]            = 3.14;
        md["Key"]           = std::string{"Value"};
        md["key2"]          = 2;

        DictSerializer<int, double, std::string>{"outfile.dat"}(md);
    }

    auto md = DictDeSerializer<int, double, std::string>{"outfile.dat"}();
    PRINT(md["test"]["super"].to<int>());
    PRINT(md["PI"].to<double>());
    PRINT(md["Key"].to<std::string>());
    PRINT(md["key2"].to<int>());

    return 0;
}

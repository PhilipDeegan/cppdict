#ifndef CPPDICT_DICT_HPP
#error // include dict first
#endif

#ifndef CPPDICT_DICT_EXP_SERIALZIER_HPP
#define CPPDICT_DICT_EXP_SERIALZIER_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <type_traits>
#include <variant>



namespace cppdict
{


template<typename T>
void _read_data(T data, std::size_t const size, std::ifstream& in)
{
    in.read(reinterpret_cast<char*>(data), size);
}

template<typename T>
void _write_data(T const data, std::size_t const size, std::ofstream& out)
{
    out.write(reinterpret_cast<char const* const>(data), size);
}

template<typename...>
using tryToInstanciate = void;

template<typename T, typename data = void, typename size = void>
struct is_spannable : std::false_type
{
};

template<typename T>
struct is_spannable<T, tryToInstanciate<decltype(std::declval<T>().data())>,
                    tryToInstanciate<decltype(std::declval<T>().size())>> : std::true_type
{
};

template<typename T>
bool constexpr static is_spannable_v = is_spannable<T>::value;


template<typename... Types>
class DictSerializer
{
    using Dict                           = cppdict::Dict<Types...>;
    using data_t                         = typename Dict::data_t;
    using node_t                         = typename Dict::node_t;
    using Tuple                          = std::tuple<Types...>;
    std::size_t constexpr static n_types = std::tuple_size_v<Tuple>;
    std::size_t constexpr static base    = 2; // 0 = map, 1 = map_key_string


public:
    DictSerializer(std::string const& filename_)
        : filename{filename_}
    {
    }

    void operator()(Dict const& dict)
    {
        std::function<void(std::string const&, node_t const&)> const node_visitor
            = [&](std::string const& key, node_t const& v) {
                  std::size_t keys = 0;
                  for (const auto& [key, node] : v)
                      if (is_node_or_serializable(*node))
                          ++keys;

                  {
                      std::size_t const type = 0;
                      _write_data(&type, sizeof(type), out);
                      std::size_t const size = key.size();
                      _write_data(&size, sizeof(size), out);
                      _write_data(&keys, sizeof(keys), out);
                      _write_data(key.data(), size, out);
                  }

                  for (const auto& [key, node] : v)
                  {
                      if (!is_node_or_serializable(*node))
                          continue;

                      if (node->isNode())
                          node_visitor(key, std::get<node_t>(node->data));
                      else
                      {
                          std::size_t const type = 1;
                          _write_data(&type, sizeof(type), out);
                          std::size_t const size = key.size();
                          _write_data(&size, sizeof(size), out);
                          _write_data(key.data(), size, out);

                          (*this)(*node);
                      }
                  }
              };


        if (dict.isNode())
            dict.visit(
                cppdict::visit_all_nodes, node_visitor,
                [&](const std::string&, const typename Dict::empty_leaf_t&) {},
                [&](const std::string& key, const auto& v) {
                    using El = std::decay_t<decltype(v)>;
                    if constexpr (is_spannable_v<El> || std::is_fundamental<El>::value)
                    {
                        std::size_t const type = 1;
                        _write_data(&type, sizeof(type), out);
                        std::size_t const size = key.size();
                        _write_data(&size, sizeof(size), out);
                        _write_data(key.data(), size, out);

                        write_type_id(v, std::make_integer_sequence<std::size_t, n_types>{});
                        _serialize_data(v);
                    }
                });
        else if (dict.isValue())
            _serialize_variant(dict.data, std::make_integer_sequence<std::size_t, n_types>{});
    }


private:
    template<std::size_t I>
    bool _holds_serializable(data_t const& data) const
    {
        using El = std::tuple_element_t<I, Tuple>;

        if constexpr (is_spannable_v<El> || std::is_fundamental<El>::value)
            return std::holds_alternative<El>(data);
        return false;
    }

    template<std::size_t... Is>
    bool holds_serializable(data_t const& data, std::integer_sequence<std::size_t, Is...>) const
    {
        int any = 0;
        ((any += _holds_serializable<Is>(data)), ...);
        return any > 0;
    }

    bool is_node_or_serializable(Dict const& dict) const
    {
        return dict.isNode()
               || holds_serializable(dict.data, std::make_integer_sequence<std::size_t, n_types>{});
    };


    template<typename T>
    void _serialize_data(T const& data)
    {
        if constexpr (is_spannable_v<T>)
        {
            std::size_t const size = data.size();
            _write_data(&size, sizeof(size), out);
            _write_data(data.data(), size * sizeof(typename T::value_type), out);
        }
        else if (std::is_fundamental<T>::value)
        {
            std::size_t const size = sizeof(T);
            _write_data(&data, size, out);
        }
        // else
        //     std::cerr << "type not serializable!" << std::endl;
    }

    template<std::size_t I>
    void __serialize_variant(data_t const& data, bool& b)
    {
        using El = std::tuple_element_t<I, Tuple>;

        if constexpr (is_spannable_v<El> || std::is_fundamental<El>::value)
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

    std::string const filename;
    std::ofstream out{filename, std::ios::binary};
};


template<typename... Types>
class DictDeSerializer
{
    using Dict                           = cppdict::Dict<Types...>;
    using data_t                         = typename Dict::data_t;
    using Tuple                          = std::tuple<Types...>;
    std::size_t constexpr static n_types = std::tuple_size_v<Tuple>;
    std::size_t constexpr static base    = 2; // 0 = map, 1 = map_key_string

public:
    DictDeSerializer(std::string const& filename_)
        : filename{filename_}
    {
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

private:
    template<typename T>
    void _deserialize_data(Dict& node)
    {
        if constexpr (is_spannable_v<T>)
        {
            std::size_t size = 0;

            in.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

            T s;
            s.resize(size);
            _read_data(s.data(), size * sizeof(typename T::value_type), in);

            node = s;
        }
        else if (std::is_fundamental<T>::value)
        {
            T d{};
            in.read(reinterpret_cast<char*>(&d), sizeof(T));
            node = d;
        }
    }

    template<std::size_t I>
    void _read(Dict& node, std::size_t const& type)
    {
        using El = std::tuple_element_t<I, Tuple>;

        if constexpr (is_spannable_v<El> || std::is_fundamental<El>::value)
            if (I + base == type)
                _deserialize_data<El>(node);
    }

    template<std::size_t... Is>
    void read(Dict& node, std::size_t const& type, std::integer_sequence<std::size_t, Is...>)
    {
        (_read<Is>(node, type), ...);
    }


    std::string const filename;
    std::ifstream in{filename, std::ios::binary};
};



} // namespace cppdict


#endif // CPPDICT_DICT_EXP_SERIALZIER_HPP

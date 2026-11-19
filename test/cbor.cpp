


#include <catch2/catch_test_macros.hpp>

#include "dict/cbor.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

using namespace cppdict::cbor;


TEST_CASE("can read/write", "[cppdict]")
{
    using MyDict = cppdict::Dict<int, double, std::string>;

    std::string const file("outfile.cbor");

    {
        MyDict md;
        md["PI"]            = 3.14;
        md["test"]["super"] = 2;
        md["Key"]           = std::string{"Value"};
        md["key2"]          = 2;

        save_config(file, md);
    }

    auto const md = load_config<MyDict>(file);

    REQUIRE(md["Key"].to<std::string>() == "Value");
    REQUIRE(md["key2"].to<int>() == 2);
    REQUIRE(md["PI"].to<double>() == 3.14);
    REQUIRE(md["test"]["super"].to<int>() == 2);
}


TEST_CASE("file is plain CBOR", "[cppdict]")
{
    using MyDict = cppdict::Dict<std::int32_t, double, std::string>;
    MyDict md;
    md["a"]["b"] = 1;
    md["s"]      = std::string{"x"};
    md["d"]      = 1.5;

    auto const doc = json::from_cbor(to_bytes(md));
    REQUIRE(doc["format"] == "cppdict");
    REQUIRE(doc["version"] == format_version);
    REQUIRE(doc["root"]["a"]["b"] == json::array({"i32", 1}));
    REQUIRE(doc["root"]["s"] == json::array({"str", "x"}));
    REQUIRE(doc["root"]["d"] == json::array({"f64", 1.5}));
}


TEST_CASE("basic types round trip", "[cppdict]")
{
    using MyDict
        = cppdict::Dict<bool, std::int8_t, std::int16_t, std::int32_t, std::int64_t, std::uint8_t,
                        std::uint16_t, std::uint32_t, std::uint64_t, float, double, std::string>;

    MyDict md;
    md["b"]   = true;
    md["i8"]  = std::numeric_limits<std::int8_t>::min();
    md["i16"] = std::numeric_limits<std::int16_t>::min();
    md["i32"] = std::numeric_limits<std::int32_t>::min();
    md["i64"] = std::numeric_limits<std::int64_t>::min();
    md["u8"]  = std::numeric_limits<std::uint8_t>::max();
    md["u16"] = std::numeric_limits<std::uint16_t>::max();
    md["u32"] = std::numeric_limits<std::uint32_t>::max();
    md["u64"] = std::numeric_limits<std::uint64_t>::max();
    md["f"]   = 1.5f;
    md["d"]   = 0.1;
    md["s"]   = std::string{"s"};

    auto const loaded = from_bytes<MyDict>(to_bytes(md));
    REQUIRE(loaded["b"].to<bool>() == true);
    REQUIRE(loaded["i8"].to<std::int8_t>() == std::numeric_limits<std::int8_t>::min());
    REQUIRE(loaded["i16"].to<std::int16_t>() == std::numeric_limits<std::int16_t>::min());
    REQUIRE(loaded["i32"].to<std::int32_t>() == std::numeric_limits<std::int32_t>::min());
    REQUIRE(loaded["i64"].to<std::int64_t>() == std::numeric_limits<std::int64_t>::min());
    REQUIRE(loaded["u8"].to<std::uint8_t>() == std::numeric_limits<std::uint8_t>::max());
    REQUIRE(loaded["u16"].to<std::uint16_t>() == std::numeric_limits<std::uint16_t>::max());
    REQUIRE(loaded["u32"].to<std::uint32_t>() == std::numeric_limits<std::uint32_t>::max());
    REQUIRE(loaded["u64"].to<std::uint64_t>() == std::numeric_limits<std::uint64_t>::max());
    REQUIRE(loaded["f"].to<float>() == 1.5f);
    REQUIRE(loaded["d"].to<double>() == 0.1);
    REQUIRE(loaded["s"].to<std::string>() == "s");
}


TEST_CASE("sequences round trip", "[cppdict]")
{
    using MyDict = cppdict::Dict<int, std::vector<int>, std::vector<double>,
                                 std::vector<std::string>, std::array<std::uint32_t, 3>>;

    MyDict md;
    md["vi"]  = std::vector<int>{1, -2, 3};
    md["vd"]  = std::vector<double>{};
    md["vs"]  = std::vector<std::string>{"a", "b"};
    md["arr"] = std::array<std::uint32_t, 3>{1, 2, 3};

    auto const bytes = to_bytes(md);
    auto const doc   = json::from_cbor(bytes);
    REQUIRE(doc["root"]["vi"] == json::array({"vec<i32>", {1, -2, 3}}));
    REQUIRE(doc["root"]["arr"][0] == "arr<u32,3>");

    auto const loaded = from_bytes<MyDict>(bytes);
    REQUIRE(loaded["vi"].to<std::vector<int>>() == std::vector<int>{1, -2, 3});
    REQUIRE(loaded["vd"].to<std::vector<double>>().empty());
    REQUIRE(loaded["vs"].to<std::vector<std::string>>() == std::vector<std::string>{"a", "b"});
    REQUIRE(loaded["arr"].to<std::array<std::uint32_t, 3>>()
            == std::array<std::uint32_t, 3>{1, 2, 3});
}


TEST_CASE("cbor errors fail loudly", "[cppdict]")
{
    using MyDict = cppdict::Dict<int, double, std::string>;

    SECTION("missing file")
    {
        REQUIRE_THROWS(load_config<MyDict>("does/not/exist.cbor"));
    }

    SECTION("empty leaf")
    {
        MyDict md;
        md["a"]["b"];
        REQUIRE_THROWS(save_config("outfile.empty_leaf.cbor", md));
    }

    SECTION("empty config round trips")
    {
        save_config("outfile.empty.cbor", MyDict{});
        REQUIRE(load_config<MyDict>("outfile.empty.cbor").isEmpty());
    }

    SECTION("empty map round trips as a map")
    {
        MyDict md;
        md["a"]["b"].data = MyDict::node_t{};
        md["c"]           = 1;
        save_config("outfile.empty_map.cbor", md);
        auto const loaded = load_config<MyDict>("outfile.empty_map.cbor");
        REQUIRE(loaded["a"]["b"].isNode());
        REQUIRE(loaded["c"].to<int>() == 1);
    }

    SECTION("leaf type missing from the reading Dict")
    {
        cppdict::Dict<int, float> md;
        md["f"] = 1.5f;
        REQUIRE_THROWS(from_bytes<MyDict>(to_bytes(md)));
    }

    SECTION("integer out of range")
    {
        auto const doc = json{{"format", "cppdict"},
                              {"version", format_version},
                              {"root", {{"x", json::array({"i8", 300})}}}};
        REQUIRE_THROWS(from_bytes<cppdict::Dict<std::int8_t>>(json::to_cbor(doc)));
    }

    SECTION("sequence element out of range")
    {
        auto const doc = json{{"format", "cppdict"},
                              {"version", format_version},
                              {"root", {{"v", json::array({"vec<i8>", {1, 300}})}}}};
        REQUIRE_THROWS(from_bytes<cppdict::Dict<std::vector<std::int8_t>>>(json::to_cbor(doc)));
    }

    SECTION("std::array length mismatch")
    {
        auto const doc = json{{"format", "cppdict"},
                              {"version", format_version},
                              {"root", {{"a", json::array({"arr<f64,2>", {1.0, 2.0, 3.0}})}}}};
        REQUIRE_THROWS(from_bytes<cppdict::Dict<std::array<double, 2>>>(json::to_cbor(doc)));
    }

    SECTION("not a cppdict document")
    {
        REQUIRE_THROWS(from_bytes<MyDict>(json::to_cbor(json{{"a", 1}})));
    }

    SECTION("truncated file")
    {
        std::string const file("outfile.truncated.cbor");
        MyDict md;
        md["key"] = std::string{"Value"};
        save_config(file, md);
        {
            std::ifstream in{file, std::ios::binary};
            std::string bytes{std::istreambuf_iterator<char>(in), {}};
            std::ofstream{file, std::ios::binary}.write(bytes.data(), bytes.size() - 2);
        }
        REQUIRE_THROWS(load_config<MyDict>(file));
    }
}

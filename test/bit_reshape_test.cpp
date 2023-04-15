
#include "jn/bit_reshape.hpp"

#include <boost/ut.hpp>

#include <array>
#include <format>
#include <numeric>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

namespace ut = boost::ut;
using namespace std::literals;
using namespace ut::literals;
using ut::operators::operator==;

static_assert([]() {
  constexpr static std::array input = {
      0x12345678u,
      0x90abcdefu,
      0xdeadbeefu,
      0xbabecafeu,
  };
  constexpr auto shaper = jn::views::bit_reshape<32, 24>(input);
  return std::ranges::begin(shaper) != std::ranges::end(shaper);
}());

static_assert(std::ranges::range<decltype(jn::views::bit_reshape<32, 24>(
                  std::declval<const std::array<unsigned int, 4> &>()))>);

template <std::ranges::range Range, typename... Adaptors>
void test_roundtrip(Range &&input, Adaptors... adaptors) {
  for (auto k : std::views::iota(0u, std::ranges::size(input))) {
    std::vector<std::ranges::range_value_t<Range>> actual{};
    const auto expected{input | std::views::take(k)};
    std::ranges::copy(input | std::views::take(k) | adaptors...,
                      std::back_inserter(actual));
    // There is no way to determine the amount of
    // padding bits in the encoded stream with the current implementation.
    // Check that we only have zero bits trailing
    ut::expect((actual | std::views::drop(k) |
                std::views::drop_while([](auto &&value) {
                  return value == std::decay_t<decltype(value)>{};
                })).empty());
    // Check that the actual output without padding matches the input
    ut::expect(std::ranges::equal(actual | std::views::take(k),
                                  expected));
  }
}

template <std::size_t from_bits, std::size_t to_bits>
inline constexpr auto make_test_bit_reshape_chain() {
  return jn::views::bit_reshape<from_bits, to_bits>;
}

template <std::size_t from_bits,
          std::size_t to_bits,
          std::size_t next,
          std::size_t... steps>
inline constexpr auto make_test_bit_reshape_chain() {
  return jn::views::bit_reshape<from_bits, to_bits> |
         make_test_bit_reshape_chain<to_bits, next, steps...>();
}

template <std::size_t... steps>
void test_bit_reshape_roundtrip(std::ranges::range auto &&input) {
  constexpr static auto from_bits = std::numeric_limits<std::make_unsigned_t<
      std::ranges::range_value_t<decltype(input)>>>::digits;
  const auto title = [] {
    auto title = std::format("roundtrip {} ->", from_bits);
    for (auto step : {steps...}) {
      title += std::format(" {} ->", step);
    }
    return title += std::to_string(from_bits);
  }();
  ut::test(title) = [&input] {
    test_roundtrip(
        input, make_test_bit_reshape_chain<from_bits, steps..., from_bits>());
  };
}

template <std::ranges::range Range, typename T, T... steps>
void test_bit_reshape_roundtrip(Range &&input, std::index_sequence<steps...>) {
  test_bit_reshape_roundtrip<steps...>(std::forward<Range>(input));
}

static ut::suite<"bit_reshape strings"> suite = [] {
  "empty string"_test = [] {
    std::string actual{};
    constexpr static std::string input{};
    constexpr static std::string expected{};
    std::ranges::copy(input | jn::views::bit_reshape<8, 6>,
                      std::back_inserter(actual));
    ut::expect(actual.empty());
    ut::expect(actual == expected);
  };
  "small string roundtrip"_test = [] {
    std::string actual{};
    constexpr static auto input = "abcdefgh12345678"sv;
    constexpr static auto expected{input};
    std::string encoded{};
    std::ranges::copy(input | jn::views::bit_reshape<8, 5>,
                      std::back_inserter(encoded));
    std::ranges::copy(
        encoded | jn::views::bit_reshape<5, 8> | std::views::take(input.size()),
        std::back_inserter(actual));
    std::cout << "Actual: <" << actual << ">" << std::endl;
    ut::expect(actual == expected);
  };
  "small string different lengths"_test = [] {
    test_bit_reshape_roundtrip<1, 2, 3, 4>("abcdefgh12345678"sv);
  };
};

static ut::suite<"bit_reshape integers"> bit_reshape_ints = [] {
  "short input"_test = [] {
    std::vector<std::uint32_t> actual{};
    constexpr static auto input = {
        1u,
    };
    constexpr static auto expected = 2_u;
    std::ranges::copy(input | jn::views::bit_reshape<1, 2>,
                      std::back_inserter(actual));
    ut::expect(actual.size() == 1_u);
    ut::expect(actual.front() == expected);
  };
  "trailing bits"_test = [] {
    std::vector<std::uint32_t> actual{};
    constexpr static auto input = {
        0x1234u,
        0xdeafu,
    };
    constexpr static auto expected = ut::_u(0x2fu << 18);
    std::ranges::copy(input | jn::views::bit_reshape<16, 25>,
                      std::back_inserter(actual));
    ut::expect(actual.size() == 2_u);
    ut::expect(actual.back() == expected);
  };
};

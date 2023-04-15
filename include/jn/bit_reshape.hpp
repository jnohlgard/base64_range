#pragma once
#include "jn/range_adaptor.hpp"

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <ranges>
#include <utility>

namespace jn::ranges {

template <std::size_t bits_in,
          std::size_t bits_out,
          std::ranges::view View,
          std::integral OutType = std::ranges::range_value_t<View>>
  requires(bits_in <= 32) &&
          (bits_out <= 32) && std::ranges::input_range<View> &&
          std::integral<std::ranges::range_value_t<View>>
class bit_reshape_view
    : std::ranges::view_interface<
          bit_reshape_view<bits_in, bits_out, View, OutType>> {
  View base;

 public:
  class Iterator {
    using Parent = bit_reshape_view;
    using BaseIterator = std::ranges::iterator_t<View>;
    using BaseSentinel = std::ranges::sentinel_t<View>;
    BaseIterator inner_it{};
    BaseSentinel end{};

   public:
    using value_type = OutType;
//    using iterator_concept =
//        std::iterator_traits<BaseIterator>::iterator_concept;
    using iterator_category =
        std::iterator_traits<BaseIterator>::iterator_category;
    using difference_type = std::ranges::range_difference_t<View>;

    constexpr Iterator(const Parent &parent,
                       std::ranges::iterator_t<View> inner_it)
        : inner_it(std::move(inner_it)), end(std::ranges::end(parent.base)) {
      while (bit_count < bits_out && this->inner_it != end) {
        fill_one();
      }
    }

    constexpr Iterator()
      requires std::default_initializable<std::ranges::iterator_t<View>>
    = default;
    constexpr Iterator(const Iterator &other) = default;
    constexpr Iterator(Iterator &&other) noexcept = default;
    Iterator &operator=(const Iterator &other) & = default;
    Iterator &operator=(Iterator &&other) & noexcept = default;

    constexpr value_type operator*() const {
      static constexpr auto out_shift = bit_buffer_width - bits_out;
      return {
          static_cast<value_type>((bit_buffer >> out_shift) & bits_out_mask)};
    }

    constexpr Iterator &operator++() {
      drain_one();
      while (bit_count < bits_out && inner_it != end) {
        fill_one();
      }
      return *this;
    }

    constexpr void operator++(int) { ++*this; }

    constexpr Iterator operator++(int)
      requires std::ranges::forward_range<View>
    {
      auto ret = *this;
      ++*this;
      return ret;
    }

    constexpr bool operator==(const Iterator &other) const {
      bool has_buffered_value = bit_count > 0;
      bool other_has_buffered_value = other.bit_count > 0;
      return has_buffered_value == other_has_buffered_value &&
             inner_it == other.inner_it;
    }
    template <typename OtherIterator>
      requires std::sentinel_for<OtherIterator, std::ranges::iterator_t<View>>
    friend constexpr bool operator==(const Iterator &it,
                                     OtherIterator &&other) {
      return it.bit_count == 0 && it.inner_it == other;
    }
    friend constexpr bool operator==(const Iterator &it,
                                     std::ranges::iterator_t<View> other) {
      return it.bit_count == 0 && it.inner_it == other;
    }
    friend constexpr bool operator==(const Iterator &it,
                                     std::default_sentinel_t) {
      return it.bit_count == 0 && it.inner_it == it.end;
    }

   private:
    constexpr void drain_one() {
      if (bit_count > bits_out) {
        bit_count -= bits_out;
      } else {
        bit_count = 0;
      }
      bit_buffer = (bit_buffer << bits_out);
    }
    constexpr void fill_one() {
      static constexpr auto in_shift = bit_buffer_width - bits_in;
      bit_buffer |=
          ((static_cast<bit_buffer_type>(*inner_it) << in_shift) >> bit_count);
      bit_count += bits_in;
      ++inner_it;
    }

    // Determine the most suitable type for the internal buffer variable
    constexpr static auto _buffer_type() {
      constexpr auto bit_buffer_req =
          std::max(2 * bits_in, bits_out + bits_in - 1);
      if constexpr (bit_buffer_req <= 32) {
        return std::uint32_t{};
      } else {
        return std::uint64_t{};
      }
    }

    using bit_buffer_type = decltype(_buffer_type());
    constexpr static value_type bits_out_mask = ((1u << bits_out) - 1u);

    bit_buffer_type bit_buffer{};
    constexpr static auto bit_buffer_width =
        std::numeric_limits<bit_buffer_type>::digits;
    unsigned int bit_count{};
  };

 public:
  constexpr explicit bit_reshape_view(View base) : base(std::move(base)) {}
  constexpr auto begin [[nodiscard]] () const {
    return Iterator(*this, std::ranges::begin(base));
  }
  constexpr auto end [[nodiscard]] () const {
    return std::default_sentinel_t{};
  }
  constexpr auto size [[nodiscard]] () const
    requires std::ranges::sized_range<View>
  {
    return std::ranges::size(base) * bits_in / bits_out;
  }
};
}  // namespace jn::ranges

namespace jn::views {

namespace detail {
template <std::size_t bits_in, std::size_t bits_out, typename Range>
concept can_bit_reshape_view = requires {
  jn::ranges::
      bit_reshape_view<bits_in, bits_out, std::ranges::views::all_t<Range>>(
          std::declval<Range>());
};

}  // namespace detail

template <std::size_t bits_in, std::size_t bits_out>
struct BitReshapeAdaptor
    : jn::ranges::range_adaptor_closure<BitReshapeAdaptor<bits_in, bits_out>> {
  template <std::ranges::range Range>
    requires detail::can_bit_reshape_view<bits_in, bits_out, Range>
  constexpr auto operator() [[nodiscard]] (Range &&base) const {
    using OutType = decltype(bitreshape_default_out_type<
                             std::ranges::range_value_t<Range>>());
    return ranges::bit_reshape_view<bits_in,
                                    bits_out,
                                    std::ranges::views::all_t<Range>,
                                    OutType>{
        std::views::all(std::forward<Range>(base))};
  }

 private:
  template <typename InputValueType>
  [[nodiscard]] static constexpr auto bitreshape_default_out_type() {
    if constexpr (bits_out <= 8) {
      if constexpr (std::same_as<std::remove_cvref_t<InputValueType>,
                                 std::byte>) {
        return std::byte{};
      } else if constexpr (std::same_as<std::remove_cvref_t<InputValueType>,
                                        char>) {
        return char{};
      } else {
        return std::uint8_t{};
      }
    } else if constexpr (bits_out <= 16) {
      return std::uint16_t{};
    } else if constexpr (bits_out <= 32) {
      return std::uint32_t{};
    }
  }
};

template <std::size_t bits_in, std::size_t bits_out>
inline constexpr BitReshapeAdaptor<bits_in, bits_out> bit_reshape;

template <std::ranges::range Range, std::size_t bits_in, std::size_t bits_out>
constexpr auto operator|(
    Range &&range, const BitReshapeAdaptor<bits_in, bits_out> &bit_reshape) {
  return bit_reshape(std::forward<Range>(range));
}

}  // namespace jn::views

template <std::size_t bits_in,
          std::size_t bits_out,
          std::ranges::view Inner,
          std::unsigned_integral OutType>
inline constexpr bool std::ranges::enable_borrowed_range<
    jn::ranges::bit_reshape_view<bits_in, bits_out, Inner, OutType>> =
    enable_borrowed_range<Inner>;
template <std::size_t bits_in, std::size_t bits_out, std::ranges::view Inner>
inline constexpr bool std::ranges::enable_borrowed_range<
    jn::ranges::bit_reshape_view<bits_in, bits_out, Inner>> =
    enable_borrowed_range<Inner>;

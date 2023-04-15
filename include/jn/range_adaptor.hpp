//
// Copyright (c) 2023 jnohlgard. All rights reserved.
//

#pragma once

#include <concepts>
#include <ranges>
#include <utility>

namespace jn::ranges {
namespace detail {
// True if the range adaptor `Adaptor` can be applied with `Args`.
template <typename Adaptor, typename... Args>
concept adaptor_invocable =
    requires { std::declval<Adaptor>()(std::declval<Args>()...); };
// True if the arguments can be chained
template <typename Lhs, typename Rhs, typename Range>
concept pipe_invocable = requires {
  std::declval<Rhs>()(std::declval<Lhs>()(std::declval<Range>()));
};

template <typename Lhs, typename Rhs>
class Pipe;
}  // namespace detail
// [range.adaptor.object], range adaptor objects
template <class Derived>
  requires std::is_class_v<Derived> &&
           std::same_as<Derived, std::remove_cv_t<Derived>>
class range_adaptor_closure {
 public:
  // range | adaptor is equivalent to adaptor(range).
  template <typename Self, typename Range>
    requires std::derived_from<std::remove_cvref_t<Self>,
                               range_adaptor_closure> &&
             detail::adaptor_invocable<Self, Range>
  friend constexpr auto operator|(Range &&range, Self &&self) {
    return std::forward<Self>(self)(std::forward<Range>(range));
  }

  // Compose the adaptors lhs and rhs into a pipeline, returning
  // another range adaptor closure object.
  template <typename Lhs, typename Rhs>
    requires(std::derived_from<Lhs, range_adaptor_closure<Derived>> &&
             std::derived_from<Rhs, range_adaptor_closure<Rhs>>)
  friend constexpr auto operator|(Lhs lhs, Rhs rhs) {
    return detail::Pipe<Lhs, Rhs>{std::move(lhs), std::move(rhs)};
  }
};

namespace detail {
// A range adaptor closure that represents composition of the range
// adaptor closures Lhs and Rhs.
template <typename Lhs, typename Rhs>
class Pipe : public range_adaptor_closure<Pipe<Lhs, Rhs>> {
  [[no_unique_address]] Lhs lhs;
  [[no_unique_address]] Rhs rhs;

 public:
  constexpr Pipe(Lhs lhs, Rhs rhs) : lhs(std::move(lhs)), rhs(std::move(rhs)) {}

  // Invoke _M_rhs(_M_lhs(__r)) according to the value category of this
  // range adaptor closure object.
  template <typename Range>
    requires pipe_invocable<const Lhs &, const Rhs &, Range>
  constexpr auto operator()(Range &&range) const & {
    return rhs(lhs(std::forward<Range>(range)));
  }

  template <typename Range>
    requires pipe_invocable<Lhs, Rhs, Range>
  constexpr auto operator()(Range &&range) && {
    return std::move(rhs)(std::move(lhs)(std::forward<Range>(range)));
  }

  template <typename Range>
  constexpr auto operator()(Range &&range) const && = delete;
};

}  // namespace detail
}  // namespace jn::ranges

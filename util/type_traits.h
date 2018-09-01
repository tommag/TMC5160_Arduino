// From http://close2.github.io/alibvr/doxygen/html/df/d63/type__traits_8h_source.html

#pragma once

namespace std {

  template<class T, T v>
  struct integral_constant {
      static constexpr T value = v;
      typedef T value_type;
      typedef integral_constant type;
      constexpr operator value_type() const noexcept { return value; }
      constexpr value_type operator()() const noexcept { return value; }
  };

  typedef std::integral_constant<bool, true> true_type;
  typedef std::integral_constant<bool, false> false_type;

  template<class T, class U>
  struct is_same : false_type {};

  template<class T>
  struct is_same<T, T> : true_type {};


  template<bool B, class T, class F>
  struct conditional { typedef T type; };

  template<class T, class F>
  struct conditional<false, T, F> { typedef F type; };
}

// Author(s):         Inbae Jeong, Chansol Hong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_SPSC_BUFFER_HPP
#define H_SPSC_BUFFER_HPP
#pragma once

#include <array>
#include <atomic>
#include <utility>

#include <cstdint>

// TODO: give appropriate memory orders to atomic operations for performance

namespace aiwc {
  namespace detail {

    class flag
    {
    public:
      using rep_type = std::uint_fast8_t; // XNDD CCSS

      constexpr flag()
      : flag(false, 0, 1, 2)
      { }

      constexpr bool is_new_write() const { return rep_ & 0x40; }
      constexpr std::size_t dirty() const { return (rep_ >> 4) & 0x03; }
      constexpr std::size_t clean() const { return (rep_ >> 2) & 0x03; }
      constexpr std::size_t snap()  const { return (rep_     ) & 0x03; }

      static constexpr flag consume(const flag& f) // swap clean and snap
      { return flag{false, f.dirty(), f.snap(), f.clean()}; }

      static constexpr flag produce(const flag& f) // swap dirty and clean
      { return flag{true, f.clean(), f.dirty(), f.snap()}; }

    private:
      constexpr flag(bool is_new, std::size_t dirty, std::size_t clean, std::size_t snap)
      : rep_{static_cast<rep_type>((is_new ? 0x40u : 0x00) | (dirty << 4) | (clean << 2) | (snap << 0))}
      { }

    private:
      rep_type rep_;
    };

  } // namespace detail

  template <class T>
  class spsc_buffer
  {
  public:
    using value_type = T;

  public:
    spsc_buffer(const value_type& v = value_type{})
      : flag_{{}}
      , data_{v, v, v}
    { }

    // noncopyable but movable
    spsc_buffer(const spsc_buffer&) = delete;
    spsc_buffer& operator=(const spsc_buffer&) = delete;
    spsc_buffer(spsc_buffer&&) = default;
    spsc_buffer& operator=(spsc_buffer&&) = default;

    bool is_lock_free() const { return flag_.is_lock_free(); }

    value_type read() const
    {
      auto f = flag_.load();

      do {
        if(!f.is_new_write()) { // if it's not new, just return snap
          break;
        }
      } while(!flag_.compare_exchange_weak(f, detail::flag::consume(f)));

      f = flag_.load(); //reload

      return data_[f.snap()];
    }

    template <class U>
    void write(U&& u)
    {
      auto f = flag_.load();

      data_[f.dirty()] = std::forward<U>(u);

      while(!flag_.compare_exchange_weak(f, detail::flag::produce(f)));
    }

  private:
    mutable std::atomic<detail::flag> flag_;
    std::array<value_type, 3> data_;
  };

} // namespace aiwc

#endif // H_SPSC_BUFFER_HPP

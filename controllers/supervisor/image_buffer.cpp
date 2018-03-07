// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#include "image_buffer.hpp"

#include <algorithm>

namespace /* anonymous */ {

  static constexpr std::size_t BYTE_PER_PIXEL = 4;

  unsigned char* get_pixel_ptr(std::vector<unsigned char>& im, std::size_t stride, std::size_t px, std::size_t py)
  {
    return std::next(&im[0], (stride * py + px) * BYTE_PER_PIXEL);
  }

  void set_alpha(std::vector<unsigned char>& im, std::size_t x, std::size_t y, unsigned char a)
  {
    for(std::size_t i = 0; i < x * y; ++i) {
      im[i * BYTE_PER_PIXEL + 3] = a;
    }
  }

  constexpr unsigned char get_b(const unsigned char* pixel_ptr) { return pixel_ptr[0]; }
  constexpr unsigned char get_g(const unsigned char* pixel_ptr) { return pixel_ptr[1]; }
  constexpr unsigned char get_r(const unsigned char* pixel_ptr) { return pixel_ptr[2]; }

  template <class OutputIterator>
  OutputIterator encode_pixel(const unsigned char* pixel_ptr, OutputIterator it) {
    static constexpr char enc[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    const unsigned char b = get_b(pixel_ptr);
    const unsigned char g = get_g(pixel_ptr);
    const unsigned char r = get_r(pixel_ptr);

    const int indices[] = { b >> 2, (b << 4) | (g >> 4), (g << 2) | (r >> 6), r };

    for(const auto& index : indices) {
      *it++ = enc[static_cast<unsigned char>(index)];
    }

    return it;
  }

} // namespace /* anonymous */

image_buffer::image_buffer(std::size_t x, std::size_t y, std::size_t nx, std::size_t ny)
  : x(x), y(y), nx(nx), ny(ny)
  , old(std::make_unique<std::vector<unsigned char> >(x * y * BYTE_PER_PIXEL))
  , cur(std::make_unique<std::vector<unsigned char> >(x * y * BYTE_PER_PIXEL))
{
  set_alpha(*cur, x, y, 0xff);
}

void image_buffer::reset()
{
  std::fill(std::begin(*cur), std::end(*cur), 0);

  set_alpha(*cur, x, y, 0xff);
}

std::deque<subimage> image_buffer::update_image(const unsigned char* ptr)
{
  using std::swap;

  std::deque<subimage> ret;

  swap(old, cur);
  std::copy_n(ptr, x * y * BYTE_PER_PIXEL, std::begin(*cur));

  for(std::size_t iy = 0; iy < nx; ++iy) {
    for(std::size_t ix = 0; ix < nx; ++ix) {
      std::size_t bx = ix * (x / nx); // begin x
      std::size_t by = iy * (y / ny); // begin y
      std::size_t ex = (ix == nx - 1) ? x : (bx + (x / nx));
      std::size_t ey = (iy == ny - 1) ? y : (by + (y / ny));

      // compare
      for(std::size_t py = by; py < ey; ++py) {
        if(!std::equal(get_pixel_ptr(*old, x, bx, py), get_pixel_ptr(*old, x, ex, py),
                       get_pixel_ptr(*cur, x, bx, py))) {
          // encode subimage and break
          std::string b64_encoded;
          b64_encoded.reserve((by - bx) * (ey - ex) * 4); // 3 bytes (BGR) are encoded into 4 bytes base64

          for(std::size_t py = by; py < ey; ++py) {
            for(std::size_t px = bx; px < ex; ++px) {
              encode_pixel(get_pixel_ptr(*cur, x, px, py), std::back_inserter(b64_encoded));
            }
          }

          ret.emplace_back(subimage{bx, by, ex - bx, ey - by, std::move(b64_encoded)});
          break;
        }
      }
    }
  }

  return ret;
}

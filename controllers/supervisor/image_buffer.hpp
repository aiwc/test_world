// Author(s):         Inbae Jeong
// Maintainer:        Chansol Hong (cshong@rit.kaist.ac.kr)

#ifndef H_IMAGE_BUFFER_HPP
#define H_IMAGE_BUFFER_HPP
#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <cstddef>

struct subimage
{
  std::size_t x;
  std::size_t y;
  std::size_t w;
  std::size_t h;

  std::string base64;
};

class image_buffer
{
public:
  image_buffer(std::size_t x, std::size_t y, std::size_t nx, std::size_t ny);

  void reset();
  std::deque<subimage> update_image(const unsigned char* ptr);

  std::size_t get_x() const { return x; }
  std::size_t get_y() const { return y; }

private:
  std::size_t x;
  std::size_t y;
  std::size_t nx;
  std::size_t ny;
  std::unique_ptr<std::vector<unsigned char> > old;
  std::unique_ptr<std::vector<unsigned char> > cur;
};


#endif // H_IMAGE_BUFFER_HPP

#ifndef FRC971_ORIN_POINTS_H_
#define FRC971_ORIN_POINTS_H_

#include <stdint.h>
#include <stddef.h>

#include <cub/iterator/transform_input_iterator.cuh>
#include <cuda/std/tuple>
#include <iomanip>
#include <ostream>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"

namespace frc971::apriltag {

// Class to hold the 2 adjacent blob IDs, a point in decimated image space, the
// half pixel offset, and the gradient.
//
// rep0 and rep1 are the two blob ids, and are each allocated 15 bits.
// point is the base point and is allocated 13 bits for x and 13 bits for y.
// dx and dy are allocated 2 bits, and can only take on set values.
// black_to_white captures the direction of the gradient in 1 bit.
//
// This adds up to 63 bits so we can load this with one big load.
struct QuadBoundaryPoint {
  static constexpr size_t kRepEndBit = 30;
  static constexpr size_t kBitsInKey = 64;

  __forceinline__ __host__ __device__ QuadBoundaryPoint() : key(0) {}

  // Sets rep0, the 0th blob id.  This only respects the bottom 15 bits.
  __forceinline__ __host__ __device__ void set_rep0(uint32_t rep0) {
    key = (key & ~(((uint64_t)0x7fff) << 30)) |
          (static_cast<uint64_t>(rep0 & 0x7fff) << 30);
  }
  // Returns rep0.
  __forceinline__ __host__ __device__ uint32_t rep0() const {
    return ((key >> 30) & 0x7fff);
  }

  // Sets rep1, the 1st blob id.  This only respects the bottom 15 bits.
  __forceinline__ __host__ __device__ void set_rep1(uint32_t rep1) {
    key = (key & ~(((uint64_t)0x7fff) << 45)) |
          (static_cast<uint64_t>(rep1 & 0x7fff) << 45);
  }
  // Returns rep1.
  __forceinline__ __host__ __device__ uint32_t rep1() const {
    return ((key >> 45) & 0x7fff);
  }

  // Returns both rep0 and rep1 concatenated into a single 30 bit number.
  __forceinline__ __host__ __device__ uint64_t rep01() const {
    return ((key >> 30) & 0x3fffffff);
  }

  // Returns all the bits used to hold position and gradient information.
  __forceinline__ __host__ __device__ uint32_t point_bits() const {
    return key & 0x3fffffff;
  }

  // Sets the 13 bit x and y.
  __forceinline__ __host__ __device__ void set_base_xy(uint32_t x, uint32_t y) {
    key = (key & ~((((uint64_t)0x1fff) << 17) | (((uint64_t)0x1fff) << 4))) |
          (static_cast<uint64_t>(x & 0x1fff) << 17) |
          (static_cast<uint64_t>(y & 0x1fff) << 4);
  }

  // Returns the base 13 bit x and y.
  __forceinline__ __host__ __device__ uint32_t base_x() const {
    return ((key >> 17) & 0x1fff);
  }
  __forceinline__ __host__ __device__ uint32_t base_y() const {
    return ((key >> 4) & 0x1fff);
  }

  // Sets dxy, the integer representing which of the 4 search directions we
  // went.
  __forceinline__ __host__ __device__ void set_dxy(uint64_t dxy) {
    key = (key & 0xfffffffffffffffcull) | (static_cast<uint64_t>(dxy & 0x3));
  }

  // Returns the change in x derived from the search direction.
  __forceinline__ __host__ __device__ int32_t dx() const {
    switch (key & 0x3) {
    case 0:
      return 1;
    case 1:
      return 1;
    case 2:
      return 0;
    case 3:
      return -1;
    }
    return 0;
  }

  // Returns the change in y derived from the search direction.
  __forceinline__ __host__ __device__ int32_t dy() const {
    switch (key & 0x3) {
    case 0:
      return 0;
    case 1:
    case 2:
    case 3:
      return 1;
    }
    return 0;
  }

  // Returns the un-decimated x and y positions for the provided decimation
  // factor.
  __forceinline__ __host__ __device__ uint32_t x(size_t decimation) const {
    return static_cast<int32_t>(base_x() * decimation) + dx();
  }
  __forceinline__ __host__ __device__ uint32_t y(size_t decimation) const {
    return static_cast<int32_t>(base_y() * decimation) + dy();
  }

  // Returns the gradient that this point represents, taking into account which
  // direction the color transitioned.
  __forceinline__ __host__ __device__ int8_t gx() const {
    return black_to_white() ? dx() : -dx();
  }
  __forceinline__ __host__ __device__ int8_t gy() const {
    return black_to_white() ? dy() : -dy();
  }

  // Returns the black to white or white to black bit.
  __forceinline__ __host__ __device__ void
  set_black_to_white(bool black_to_white) {
    key = (key & 0xfffffffffffffff7ull) |
          (static_cast<uint64_t>(black_to_white) << 3);
  }
  __forceinline__ __host__ __device__ bool black_to_white() const {
    return (key & 0x8) != 0;
  }

  // Various operators to make it easy to compare points.
  __forceinline__ __host__ __device__ bool
  operator!=(const QuadBoundaryPoint other) const {
    return other.key != key;
  }
  __forceinline__ __host__ __device__ bool
  operator==(const QuadBoundaryPoint other) const {
    return other.key == key;
  }
  __forceinline__ __host__ __device__ bool
  operator<(const QuadBoundaryPoint other) const {
    return key < other.key;
  }

  // Returns true if this point has been set.  Zero is reserved for "invalid"
  __forceinline__ __host__ __device__ bool nonzero() const {
    return key != 0ull;
  }

  // Returns true if this point is about the other point.
  bool near(QuadBoundaryPoint other) const { return other == *this; }

  // The key.  This shouldn't be parsed directly.
  uint64_t key;
};

std::ostream &operator<<(std::ostream &os, const QuadBoundaryPoint &point);

// Holds a compacted blob index, the angle to the X axis from the center of the
// blob, and the coordinate of the point.
//
// The blob index is 11 bits, the angle is 23 bits, and the point is 30 bits.
struct IndexPoint {
  // Max number of blob IDs we can hold.
  static constexpr size_t kMaxBlobs = 2048;

  static constexpr size_t kRepEndBit = 30;
  static constexpr size_t kBitsInKey = 64;

  __forceinline__ __host__ __device__ IndexPoint() : key(0) {}

  // Constructor to build a point with just the blob index, and point bits.  The
  // point bits should be grabbed from a QuadBoundaryPoint rather than built up
  // by hand.
  __forceinline__ __host__ __device__ IndexPoint(uint32_t blob_index,
                                                 uint32_t point_bits)
      : key((static_cast<uint64_t>(blob_index & 0x7ff) << 53) |
            (static_cast<uint64_t>(point_bits & 0x3fffffff))) {}

  // Sets and gets the 11 bit blob index.
  __forceinline__ __host__ __device__ void set_blob_index(uint32_t blob_index) {
    key = (key & ~(((uint64_t)0x7ff) << 53)) |
          (static_cast<uint64_t>(blob_index & 0x7ff) << 53);
  }
  __forceinline__ __host__ __device__ uint32_t blob_index() const {
    return ((key >> 53) & 0x7ff);
  }

  // Sets and gets the 23 bit angle.
  __forceinline__ __host__ __device__ void set_theta(uint32_t theta) {
    key = (key & ~(((uint64_t)0x7fffff) << 30)) |
          (static_cast<uint64_t>(theta & 0x7fffff) << 30);
  }
  __forceinline__ __host__ __device__ uint32_t theta() const {
    return ((key >> 30) & 0x7fffff);
  }

  // See QuadBoundaryPoint for a description of the rest of these.
  // Sets the 13 bit x and y.
  __forceinline__ __host__ __device__ void set_base_xy(uint32_t x, uint32_t y) {
    key = (key & ~((((uint64_t)0x1fff) << 17) | (((uint64_t)0x1fff) << 4))) |
          (static_cast<uint64_t>(x & 0x1fff) << 17) |
          (static_cast<uint64_t>(y & 0x1fff) << 4);
  }

  __forceinline__ __host__ __device__ uint32_t base_x() const {
    return ((key >> 17) & 0x1fff);
  }

  __forceinline__ __host__ __device__ uint32_t base_y() const {
    return ((key >> 4) & 0x1fff);
  }

  __forceinline__ __host__ __device__ void set_dxy(uint64_t dxy) {
    key = (key & 0xfffffffffffffffcull) | (static_cast<uint64_t>(dxy & 0x3));
  }

  __forceinline__ __host__ __device__ int32_t dx() const {
    switch (key & 0x3) {
    case 0:
      return 1;
    case 1:
      return 1;
    case 2:
      return 0;
    case 3:
      return -1;
    }
    return 0;
  }

  __forceinline__ __host__ __device__ int32_t dy() const {
    switch (key & 0x3) {
    case 0:
      return 0;
    case 1:
    case 2:
    case 3:
      return 1;
    }
    return 0;
  }

  __forceinline__ __host__ __device__ uint32_t x(size_t decimation) const {
    return static_cast<int32_t>(base_x() * decimation) + dx();
  }
  __forceinline__ __host__ __device__ uint32_t y(size_t decimation) const {
    return static_cast<int32_t>(base_y() * decimation) + dy();
  }

  __forceinline__ __host__ __device__ int8_t gx() const {
    return black_to_white() ? dx() : -dx();
  }
  __forceinline__ __host__ __device__ int8_t gy() const {
    return black_to_white() ? dy() : -dy();
  }

  __forceinline__ __host__ __device__ uint32_t point_bits() const {
    return key & 0x3fffffff;
  }

  __forceinline__ __host__ __device__ void
  set_black_to_white(bool black_to_white) {
    key = (key & 0xfffffffffffffff7ull) |
          (static_cast<uint64_t>(black_to_white) << 3);
  }
  __forceinline__ __host__ __device__ bool black_to_white() const {
    return (key & 0x8) != 0;
  }

  // The key.  This shouldn't be parsed directly.
  uint64_t key;
};

std::ostream &operator<<(std::ostream &os, const IndexPoint &point);

// Decomposer for sorting which just returns the key.
struct QuadBoundaryPointDecomposer {
  __host__ __device__ ::cuda::std::tuple<uint64_t &>
  operator()(QuadBoundaryPoint &key) const {
    return {key.key};
  }
};

// Decomposer for sorting which just returns the key.
struct QuadIndexPointDecomposer {
  __host__ __device__ ::cuda::std::tuple<uint64_t &>
  operator()(IndexPoint &key) const {
    return {key.key};
  }
};

} // namespace frc971::apriltag

#endif // FRC971_ORIN_POINTS_H_

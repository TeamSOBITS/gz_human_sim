# lightsfm (vendored)

Header-only Social Force Model library from
[robotics-upo/lightsfm](https://github.com/robotics-upo/lightsfm),
BSD-3-Clause licensed (see `LICENSE`).

Vendored directly into `gz_human_sim` unmodified, aside from relocating the
headers one directory deeper (`include/*.hpp` -> `include/lightsfm/*.hpp`)
so consuming code can `#include <lightsfm/sfm.hpp>`. Used by
`src/sfm_crowd_system.cpp`.

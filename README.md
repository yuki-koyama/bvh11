# bvh11

[![Build Status](https://travis-ci.com/yuki-koyama/bvh11.svg?branch=master)](https://travis-ci.com/yuki-koyama/bvh11)
![GitHub](https://img.shields.io/github/license/yuki-koyama/bvh11)

A tiny C++11 library for reading (and writing) BVH motion capture data.

![](./docs/sample.gif)

## Dependencies

- C++11 standard library
- Eigen 3 <http://eigen.tuxfamily.org/> (`brew install eigen`)

### Additional Dependencies for Demos (Optional)

- three-dim-util <https://github.com/yuki-koyama/three-dim-util/>
  - Eigen 3 (`brew install eigen`)
  - Qt 5 (`brew install qt`)

## Usage

### Build and Install

```bash
git clone https://github.com/yuki-koyama/bvh11.git --recursive
mkdir build
cd build
cmake ../bvh11
make
make install
```

### Import (and Export) BVH Data

```cpp
#include <bvh11.hpp>

int main()
{
  // Import data
  auto bvh_object = bvh11::BvhObject("/path/to/bvh/data.bvh");

  // Do something (e.g., render the motion data, apply some processing, etc.)
  
  // Export data (if necessary)
  bvh_object.WriteBvhFile("/path/to/bvh/new_data.bvh");

  return 0;
}
```

## License

MIT License.

## Contributing

Contributions are welcome.

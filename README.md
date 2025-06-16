# FORGE: Formation of Optimized Robotic Groundtruth Examples

FORGE is a C++-based inverse kinematics and metaheuristic-gradient descent hybrid optimization framework that uses YAML-based configuration and Eigen for matrix computations. It supports multi-core execution and is designed for robotic systems defined via Denavit–Hartenberg (DH) parameters.

---

## 🚀 Features

- Load robot configuration from `.yaml` files
- Support for DH parameter models
- Inverse Kinematics solver using Gradient Descent
- Multi-core support using `std::thread`
- Clean and modular C++ struct-based design
- YAML and Eigen-based configuration and math

---

## 🛠️ Build Instructions

### ✅ Prerequisites

Make sure the following dependencies are installed:

#### Fedora:
```bash
sudo dnf install eigen3-devel yaml-cpp-devel
```

#### Ubuntu:
```bash
sudo apt install libeigen3-dev libyaml-cpp-dev
```

---

### 🔧 Build Using Make

```bash
make
```

Or manually:

```bash
g++ main.cpp -I/usr/include/eigen3 $(pkg-config --cflags --libs yaml-cpp) -std=c++17 -o FORGE_linux
```

---

### 🛠️ Build Using CMake (Recommended)

```bash
mkdir build
cd build
cmake ..
make
```

The compiled binary `FORGE_linux` will be located in the `build/` directory.

---

### 🧹 Clean Build Files
```bash
make clean
```

Or if using CMake:
```bash
rm -rf build/
```

---

## 📂 YAML Format Example

```yaml
dh_parameters:
  - a: 0
    d: 746
    alpha: 90
  - a: 1250
    d: 0
    alpha: 0
  - a: 250
    d: 0
    alpha: 90
  - a: 0
    d: 1000
    alpha: -90
  - a: 0
    d: 0
    alpha: 90
  - a: 0
    d: 200
    alpha: 0
```

---

## 🧪 Usage

```bash
./FORGE_linux config.yaml 4
```

- `config.yaml`: YAML file containing DH parameters
- `4`: Number of threads to use (multi-core optimization)

---

## 📁 Project Structure

```
FORGE/
├── main.cpp                  # Entry point
├── <metaheuristic algo>/     # Implementations of various metaheuristic algorithms
│   └── main.cpp
├── Makefile                  # Makefile build
├── CMakeLists.txt            # CMake build script
├── example_dh_parameters/    # Example YAML configurations
│   └── kawasaki_bx200l.yaml
└── README.md                 # You're reading it!
```

## 📄 Output CSV Format

After running an algorithm, FORGE generates a `.csv` file logging each inverse kinematics attempt with the following structure:

| ang1 | ang2 | ang3 | ang4 | ang5 | ang6 | pos_x | pos_y | pos_z | out_1 | out_2 | out_3 | out_4 | out_5 | out_6 |
|------|------|------|------|------|------|--------|--------|--------|--------|--------|--------|--------|--------|--------|
| 0.12 | 0.35 | 0.78 | -0.2 | 1.05 | 0.66 | 500.0  | 200.0  | 300.0  | 0.11   | 0.33   | 0.80   | -0.21  | 1.04   | 0.65   |

### Column Details:

- **ang1–ang6**: Initial joint angles provided as input to the solver.
- **pos_x, pos_y, pos_z**: Desired 3D end-effector position (target).
- **out_1–out_6**: Output joint angles predicted by the solver to achieve the target.

These CSV files are useful for:
- Evaluating solver accuracy (e.g., how close output angles reach the target)
- Debugging IK performance
- Training or benchmarking machine learning models

---

## 📜 License

This project is released under the [MIT License](LICENSE).

---

## 👨‍🔬 Author

**Rhiddhi Prasad Das**  
Built for research in robotic inverse kinematics and optimization.

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
├── metaheuristics/     
│   └── <metaheuristic algo>/ # Implementations of various metaheuristic algorithms
│       └── main.cpp
├── Makefile                  # Makefile build
├── CMakeLists.txt            # CMake build script
├── example_dh_parameters/    # Example YAML configurations
│   │── fanucm20ia.yaml
│   │── tm5_700.yaml
│   │── kawasaki_bx200l.yaml
│   │── kawasaki_bx200l_restricted_3dof.yaml
│   └── kuka_youbot.yaml
├── visualiser.py             # a basic visualiser to display a single row from generated csv
└── README.md                 # You're reading it!
```

## 📄 Output CSV Format

After running an algorithm, FORGE generates a `.csv` file logging each inverse kinematics attempt with the following structure:

| initial_ang_1 | initial_ang_2 | initial_ang_1 | initial_ang_4 | initial_ang_5 | initial_ang_6 | target_pos_x | target_pos_y | target_pos_z | final_ang_1 | final_ang_2 | final_ang_3 | final_ang_4 | final_ang_5 | final_ang_6 | algorithm |
|------|------|------|------|------|------|--------|--------|--------|--------|--------|--------|--------|--------|--------|--------|
| 0.12 | 0.35 | 0.78 | -0.2 | 1.05 | 0.66 | 500.0  | 200.0  | 300.0  | 0.11   | 0.33   | 0.80   | -0.21  | 1.04   | 0.65   |PSO_Adam|

### Column Details:

- **initial_ang_1–initial_ang_6**: Initial joint angles provided as input to the solver.
- **target_pos_x, target_pos_y, target_pos_z**: Desired 3D end-effector position (target).
- **final_ang_1-final_ang_6**: Output joint angles predicted by the solver to achieve the target.

These CSV files are useful for:
- Evaluating solver accuracy (e.g., how close output angles reach the target)
- Debugging IK performance
- Training or benchmarking machine learning models

---

## 📈 Visualizing a Single IK Output

To visualise a single row from the output CSV file, use the provided `visualiser.py` script.

### 🖥️ Usage

```bash
python visualise_fk_csv.py <config.yaml> <data_line>
```

### 📌 Example

```bash
python visualise_fk_csv.py example_dh_parameters/config.yaml "4.78,5.97,0.45,0.50,5.16,2.71,-657.6,1688.0,730.3,1.88,6.17,1.44,1.40,4.82,2.71,PSO_Adam"
```

- `<config.yaml>`: Path to the YAML file containing your robot's DH parameters  
- `<data_line>`: A comma-separated line copied from your output CSV (enclosed in double quotes)

This tool reconstructs and displays the forward kinematics output of the predicted joint angles, helping you verify accuracy visually.

---

## 📜 License

This project is released under the [MIT License](LICENSE).

---

## 📊 Dataset on Kaggle

You can explore and download the dataset used for training and evaluation here:

🔗 [FORGE Inverse Kinematics Dataset on Kaggle](https://www.kaggle.com/datasets/rhiddhiprasaddas/forge-ik-2025/)


## 👨‍🔬 Author

**Rhiddhi Prasad Das**  
Built for research in robotic inverse kinematics and optimization.

## 🙌 Testers
Thanks to the following people for helping test the program:

- @Ari-gay-tor
- @imon1207
- @tiya-513
- @SLSi14
- @Shivangid2904

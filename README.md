# AUTONOMOUS NAVIGATION RESEARCH WORKSPACE (ROS 2 JAZZY)
## Differential Drive Mobile Robot Navigation, Mapping, Coverage, & 3D Exploration

Repositori ini merupakan ruang kerja riset robotika komprehensif yang mengintegrasikan tiga pilar sistem navigasi otonom modern di atas **ROS 2 Jazzy** dan **Gazebo Harmonic**:
1. **SLAM (Simultaneous Localization and Mapping)**: Pemetaan 2D (SLAM Toolbox / Cartographer) dan 3D Voxel Octree (OctoMap).
2. **Nav2 (Navigation 2 Stack) & Fields2Cover**: Navigasi planar terkelola dengan lokalisasi Monte Carlo AMCL, pengontrol Regulated Pure Pursuit, dan cakupan area penuh (*Complete Coverage Path Planning*).
3. **CMU Autonomy Stack & TARE Exploration Planner**: Navigasi 3D berbasis manifold medan tanah (*terrain analysis*), pemilihan lintasan kisi pergerakan (*motion lattice* $v_y = 0$), dan penjelajahan otonom 3D penuh tanpa peta awal (*zero prior map*) menggunakan optimasi Dual-Layer TSP Google OR-Tools.

---

## DOKUMENTASI LENGKAP TIGA PILAR UTAMA

Setiap subsistem memiliki dokumentasi teknis mendalam yang mencakup formulasi matematis, alur integrasi rekayasa, daftar parameter lengkap, dan referensi akademik wajib:

| Subsistem | File Dokumentasi | Fokus Materi |
| :--- | :--- | :--- |
| 🗺️ **SLAM & Mapping** | [`docs/README_SLAM.md`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/docs/README_SLAM.md) | Log-Odds Occupancy Grid, Scan Matching (CSM & Ceres), Pose Graph Optimization (PGO), OctoMap 3D Voxel Tree, dan Frontier Auto-Mapping. |
| 🧭 **Nav2 & Coverage** | [`docs/README_NAV2.md`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/docs/README_NAV2.md) | KLD-Sampling AMCL, Kinematika Regulated Pure Pursuit, Inflasi Costmap Berlapis, Dekomposisi Boustrophedon, dan Swath Generation Fields2Cover. |
| ⛰️ **CMU & TARE** | [`docs/README_CMU.md`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/docs/README_CMU.md) | Analisis Medan 3D Kuantil, Deteksi Jurang/Tanjakan, Motion Lattice (343 kurva), Adaptasi Roda Diferensial, dan Dual-Layer TSP Google OR-Tools. |

Dokumentasi pelengkap lainnya:
- 📐 [Arsitektur Komprehensif & Teori Matematis](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/docs/DEVELOPMENT_ARCHITECTURE_AND_MATH.md)
- 🚀 [Panduan Operasional & Seluruh Fitur Sistem](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/docs/OPERATIONS_AND_FEATURES_GUIDE.md)

---

## RINGKASAN PERINTAH EKSEKUSI CEPAT (QUICK START)

Semua perintah dijalankan di dalam container Distrobox ROS 2 Jazzy:

```bash
distrobox enter ros2-jazzy
source /opt/ros/jazzy/setup.bash
source ~/robotics/delabo_itb/navigation_ros_ws/install/setup.bash
```

### 1. Menjalankan Mode SLAM 2D (Pemetaan Manual / Teleop)
```bash
ros2 launch robot_bringup sim_mapping.launch.py world:=turtlebot3_house.world
```

### 2. Menjalankan Mode Nav2 Complete Coverage (Fields2Cover)
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2 world:=turtlebot3_house.world
```

### 3. Menjalankan Mode CMU 3D Complete Coverage (Medan Kontur Garasi)
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu world:=garage.world lidar_mode:=3d
```

### 4. Menjalankan Mode CMU TARE 3D Autonomous Exploration (1-Klik)
```bash
ros2 launch robot_bringup sim_exploration.launch.py world:=garage.world
```

---

## STRUKTUR PAKET WORKSPACE

```
navigation_ros_ws/
├── docs/
│   ├── README_SLAM.md                  # Dokumentasi teknis lengkap SLAM 2D & OctoMap
│   ├── README_NAV2.md                  # Dokumentasi teknis lengkap Nav2 & Fields2Cover
│   ├── README_CMU.md                   # Dokumentasi teknis lengkap CMU Stack & TARE
│   ├── DEVELOPMENT_ARCHITECTURE_AND_MATH.md
│   └── OPERATIONS_AND_FEATURES_GUIDE.md
├── src/
│   ├── navigation_ros_research/
│   │   ├── robot_description/          # URDF model, sensor 2D & 3D VLP-16, worlds Gazebo
│   │   ├── robot_mapping/              # Konfigurasi SLAM Toolbox, Cartographer, OctoMap
│   │   ├── robot_navigation/           # Konfigurasi Nav2 BT, AMCL, Costmaps, RPP
│   │   ├── robot_coverage/             # Server Fields2Cover & Jembatan CMU Sim Bridge
│   │   └── robot_bringup/              # Launch file orkestrasi terpadu & konfigurasi RViz
│   └── cmu_autonomy_stack/
│       ├── terrain_analysis/           # Rekonstruksi medan kontur 3D waktu-nyata
│       ├── terrain_analysis_ext/       # Analisis konektivitas medan 3D
│       ├── sensor_scan_generation/     # Sinkronisasi odometri & pemindaian sensor
│       ├── local_planner/              # Motion lattice trajectory generator & follower
│       └── tare_planner/               # TARE 3D Autonomous Exploration (Google OR-Tools)
└── README.md
```

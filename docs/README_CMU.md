# DOKUMENTASI LENGKAP & PANDUAN IMPLEMENTASI: CMU AUTONOMY & TARE EXPLORATION
## (3D Terrain Analysis, Motion Lattice Local Planning, & TARE 3D Autonomous Exploration)

Dokumentasi ini menyajikan panduan mendalam untuk subsistem **CMU Autonomy Stack** dan **CMU TARE (Terrain-Aware Autonomous Exploration) Planner** pada repositori `navigation_ros_ws`. Dokumen ini mencakup landasan matematis analisis medan 3D, pemilihan trayektori kisi pergerakan (*motion lattice*), adaptasi kinematika roda diferensial ($v_y = 0$), formulasi hierarkis Dual-Layer TSP dengan Google OR-Tools, alur integrasi teknik ke ROS 2 Jazzy, serta rincian parameter secara komprehensif.

---

## 1. PENDAHULUAN & ARSITEKTUR TINGKAT TINGGI CMU STACK

CMU Autonomy Stack adalah ekosistem navigasi otonom 3D untuk lingkungan ekstrim yang dikembangkan oleh Field Robotics Center, Carnegie Mellon University. Berbeda dengan Nav2 yang meratakan dunia ke bidang 2D datar, CMU Autonomy Stack memperlakukan lingkungan sebagai manifold permukaan 3D berkontur, mampu mendeteksi tanjakan (*ramps*), rintangan negatif (*drop-offs/cliffs*), dan medan terjal secara waktu-nyata (*real-time*).

```
                      ┌────────────────────────────────────────┐
                      │   GAZEBO SIMULASI (3D LiDAR 16-ch)     │
                      └──────────────────┬─────────────────────┘
                                         │
                         /points 3D      │ /odom
                                         ▼
                      ┌────────────────────────────────────────┐
                      │             cmu_sim_bridge             │
                      │  • /odom -> /state_estimation (map)    │
                      │  • /points -> /registered_scan (map)   │
                      │  • Static TF (map->odom, base->vehicle)│
                      └──────────────────┬─────────────────────┘
                                         │
                                         ▼
                      ┌────────────────────────────────────────┐
                      │            terrain_analysis            │
                      │  • Two-Tier Planar Voxel Filter        │
                      │  • Quantile Surface Height & Slope     │
                      │  • Negative Obstacle (Drop-off) Det.   │
                      └──────────────────┬─────────────────────┘
                                         │
                                         │ /terrain_map (3D Elevation PointCloud)
                                         ▼
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                DUA PILAR PERENCANA OTONOM                              │
│                                                                                        │
│   PILAR A: COVERAGE 3D (Fields2Cover)            PILAR B: AUTO EXPLORATION (TARE)      │
│   • Input: Poligon batas di RViz2                • Input: Zero Prior Map               │
│   • Perencana: cmu_coverage_navigator            • Perencana: tare_planner_node        │
│   • Jalur: Sapuan baris berkontur                • Algoritma: Dual TSP (Google OR-Tools│
└───────────────────────────┬────────────────────────────────┬───────────────────────────┘
                            │                                │
                            └───────────────┬────────────────┘
                                            │ /way_point (geometry_msgs/PointStamped)
                                            ▼
                      ┌────────────────────────────────────────┐
                      │             local_planner              │
                      │  • 343 Precomputed Motion Trajectories │
                      │  • Terrain Obstacle Clearance Cost     │
                      │  • Path Follower: Diff-Drive Lock      │
                      │    (v_y = 0.0, vehicleYawRate Control) │
                      └──────────────────┬─────────────────────┘
                                         │
                                         ▼ /cmd_vel (Twist)
                                  [Robot Gazebo]
```

---

## 2. FORMULASI MATEMATIS & PRINSIP TEKNIS

### 2.1 Analisis Medan 3D (*Terrain Analysis*)
Modul `terrain_analysis` mengubah *point cloud* 3D mentah menjadi representasi medan berukuran $20\text{ m} \times 20\text{ m}$ di sekitar robot:
1. **Dua Tingkat Voxel Diskrit**:
   Data titik LiDAR diakumulasi ke dalam kisi planar horizontal dengan resolusi sel $\Delta x = \Delta y = 0.1\text{ m}$.
2. **Kuantil Ketinggian Permukaan Tanah**:
   Untuk setiap sel $(i, j)$ yang memuat $N$ titik dengan urutan ketinggian $z_1 \le z_2 \le \dots \le z_N$, estimasi ketinggian tanah lokal $z_{\text{ground}}$ diambil pada nilai kuantil persentase $q = 0.2$ (20% terbawah):
   $$z_{\text{ground}}(i, j) = z_{\lfloor q \cdot N \rfloor}$$
   Langkah ini secara matematis mengeliminasi rumput liar, semak, dan debu sensor tanpa keliru menganggapnya sebagai rintangan masif.
3. **Pemeriksaan Kemiringan Lereng (*Slope Calculation*)**:
   Kemiringan tanah dihitung dari gradien spasial ketinggian antar sel tetangga:
   $$\nabla z = \left[ \frac{\partial z}{\partial x}, \frac{\partial z}{\partial y} \right]^T \approx \left[ \frac{z_{i+1, j} - z_{i-1, j}}{2 \Delta x}, \frac{z_{i, j+1} - z_{i, j-1}}{2 \Delta y} \right]^T$$
   $$\tan \phi = \|\nabla z\|_2 = \sqrt{\left(\frac{\partial z}{\partial x}\right)^2 + \left(\frac{\partial z}{\partial y}\right)^2}$$
   Jika $\tan \phi > \tan \phi_{\text{max}}$ (di mana $\phi_{\text{max}} \approx 25^\circ$), sel tersebut ditandai sebagai sel tidak dapat dilalui (*non-traversable*).
4. **Deteksi Rintangan Negatif (*Negative Obstacle / Drop-off*)**:
   Jika terdapat rongga sel kosong di mana sel di depannya memiliki ketinggian $\Delta z_{\text{drop}} < -0.25\text{ m}$, modul mengidentifikasinya sebagai jurang/turunan tajam dan menandai tepi sel sebagai rintangan terlarang.

### 2.2 Perencana Lintasan Lokal Kisi Pergerakan (*Motion Lattice Planner*)
Modul `localPlanner` memilih lintasan terbaik dari sebuah perpustakaan berisi **343 kurva trayektori pra-komputasi** (*precomputed motion lattice*):
- Setiap kandidat trayektori $\tau_k(t) = (x_k(t), y_k(t), z_k(t))$ dibangkitkan dari kurva polinomial atau busur lingkar dengan sudut kemudi yang berbeda-beda.
- Untuk setiap trayektori $\tau_k$, dihitung total fungsi biaya (*cost function*):

$$\mathcal{J}(\tau_k) = w_{\text{goal}} \cdot \mathcal{C}_{\text{goal}}(\tau_k) + w_{\text{obs}} \cdot \mathcal{C}_{\text{obs}}(\tau_k) + w_{\text{smooth}} \cdot \mathcal{C}_{\text{smooth}}(\tau_k)$$

Di mana:
- Biaya Sasaran: $\mathcal{C}_{\text{goal}}(\tau_k) = \|\tau_k(T) - \mathbf{x}_{\text{goal}}\|_2$ (jarak titik akhir trayektori ke sasaran `/way_point`).
- Biaya Rintangan Medan:
  $$\mathcal{C}_{\text{obs}}(\tau_k) = \sum_{t=0}^T \max \left( 0, z_{\text{point}}(\tau_k(t)) - z_{\text{ground}}(\tau_k(t)) - h_{\text{clearance}} \right)$$
- Biaya Kelancaran: $\mathcal{C}_{\text{smooth}}(\tau_k) = |\kappa_k - \kappa_{\text{prev}}|$ (mencegah perubahan stir yang menyentak).

Trayektori dengan biaya minimum terpilih sebagai lintasan eksekusi: $\tau^* = \arg\min_k \mathcal{J}(\tau_k)$.

### 2.3 Pelacak Jalur Roda Diferensial (*Path Follower Kinematics*)
Di dalam `pathFollower.cpp`, sinyal kontrol kecepatan robot ditransformasikan ke format differential drive:
- Pada robot aslinya (Ackermann / Omni), CMU memancarkan kecepatan lateral $v_y$.
- Pada robot diferensial repositori ini, kecepatan lateral dikunci mati:
  $$v_y = 0.0$$
- Kecepatan sudut Yaw Rate dihitung dari kelengkungan target dan kesalahan lintasan lateral $e_y$:
  $$\omega = \frac{v \tan \delta}{L} + k_p \cdot e_y + k_d \cdot \dot{e}_y$$
- Komando akhir dipublikasikan sebagai `geometry_msgs/Twist` standar ke `/cmd_vel`.

---

## 3. FORMULASI MATEMATIS CMU TARE AUTONOMOUS EXPLORATION PLANNER

Algoritma **TARE (Terrain-Aware Autonomous Exploration) Planner** memungkinkan robot menjelajahi seluruh area 3D tanpa peta awal secara mandiri 100% dengan efisiensi tinggi melalui penyelesaian dua tingkat Travelling Salesman Problem (TSP).

### 3.1 Voxel Rolling Occupancy Grid & Ray-Tracing
Ruang penjelajahan dibagi menjadi kisi sel voxel dinamis berukuran $0.2\text{ m} \times 0.2\text{ m} \times 0.2\text{ m}$.
- Setiap kali data `/registered_scan` masuk, algoritma *Bresenham 3D Ray-Casting* menarik garis lurus dari pusat sensor menuju setiap titik pantulan.
- Seluruh voxel yang dilewati sinar diubah statusnya menjadi **Free**, sedangkan voxel ujung pantulan ditandai sebagai **Occupied**.
- Voxel yang belum pernah terjamah berkas sinar tetap berstatus **Unknown**.

### 3.2 Deteksi Perbatasan Frontier 3D
Frontier $\mathcal{F}$ adalah himpunan sel **Free** yang bersinggungan langsung dengan minimal satu sel **Unknown**:

$$\mathcal{F} = \{ V \in \text{VoxelGrid} \mid \text{State}(V) = \text{Free} \land \exists U \in \mathcal{N}_{26}(V) \text{ s.t. } \text{State}(U) = \text{Unknown} \}$$

Frontier yang terdeteksi dikelompokkan dengan *Euclidean Clustering* ($r = 1.0\text{ m}$, minimum 8 titik) untuk membuang titik-titik derau liar.

### 3.3 Sampling Titik Sudut Pandang Sensor (*Viewpoint Sampling*)
Untuk setiap klaster frontier, dibangkitkan sejumlah kandidat sudut pandang $\mathcal{V} = \{v_1, \dots, v_n\}$ pada elevasi medan $z = z_{\text{terrain}} + h_{\text{sensor}}$. Setiap kandidat diperiksa:
1. **Bebas Tabrakan (*Collision Margin*)**: Jarak minimum ke rintangan $\ge 0.35\text{ m}$.
2. **Konektivitas Graf (*Keypose Graph*)**: Harus terhubung ke graf posisi yang pernah dilalui robot tanpa terhalang dinding.

### 3.4 Optimasi Dual-Layer TSP Menggunakan Google OR-Tools
Komputasi rute TSP global yang melibatkan ratusan titik secara simultan memiliki kompleksitas $\mathcal{O}(N!)$ atau $\mathcal{O}(2^N N^2)$, yang mustahil diselesaikan secara real-time. TARE membaginya secara hierarkis:

#### Lapisan 1: Global Subspace TSP
Ruang eksplorasi dipetakan ke dalam makro-kubus ruang (*Grid World Subspaces*) berukuran $18\text{ m} \times 18\text{ m} \times 1.8\text{ m}$.
Rute kunjungan antar-kubus yang masih memiliki frontier aktif dioptimasi menggunakan TSP global melintasi *Keypose Graph*:

$$\min_{\pi \in \mathcal{S}_K} \sum_{k=1}^{K-1} D_{\text{graph}}(C_{\pi(k)}, C_{\pi(k+1)})$$

#### Lapisan 2: Local Coverage TSP
Di dalam ruang pandang lokal robot (*Local Planning Horizon*, radius $4.0\text{ m}$), terdapat sekumpulan titik kandidat sudut pandang $\{v_1, \dots, v_m\}$.
Rute lokal dioptimasi secara instan menggunakan solver **Google OR-Tools Routing Library**:

$$\min_{\sigma \in \mathcal{S}_m} \sum_{j=1}^{m-1} \left( d_{\text{geodesic}}(v_{\sigma(j)}, v_{\sigma(j+1)}) + \lambda_{\text{heading}} \cdot \Delta \theta \right)$$

Titik pertama dari rute optimal lokal dipublikasikan langsung ke `/way_point`, membimbing robot menyapu frontier terdekat secara kontinu.

---

## 4. ALUR TEKNIS INTEGRASI CMU & TARE DI REPOSITORI INI

### Langkah 1: Pembangunan Node Jembatan `cmu_sim_bridge.py`
Stack CMU aslinya membutuhkan topik odometri bernama `/state_estimation` dengan frame acuan `map` dan anak `vehicle`, serta point cloud di frame `map`. Di Gazebo, robot memancarkan `/odom` (frame `odom` $\to$ `base_footprint`) dan `/points` (frame sensor `lidar_3d_link`).
- Kami membangun node [`cmu_sim_bridge.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_coverage/robot_coverage/cmu_sim_bridge.py) yang:
  1. Menerjemahkan `/odom` menjadi `/state_estimation` dengan konversi orientasi quaternion matriks rotasi.
  2. Menerapkan matriks transformasi homogen terhadap seluruh titik `/points` secara live menjadi `/registered_scan` di frame `map`.
  3. Memfilter nilai `NaN` dan `Inf` secara ketat sebelum komputasi matriks NumPy.
  4. Memancarkan TF statis `map -> odom`, `base_footprint -> vehicle`, dan `vehicle -> sensor`.

### Langkah 2: Mengunci Kinematika Roda Diferensial di C++ CMU
Pada file [`pathFollower.cpp`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/cmu_autonomy_stack/src/base_autonomy/local_planner/src/pathFollower.cpp), implementasi asli menghitung kecepatan lateral holonomik:
```cpp
// KODE ASLI CMU (OMNI/ACKERMANN):
cmd_vel.twist.linear.x = cos(vehicleYawRate) * vehicleSpeed;
cmd_vel.twist.linear.y = sin(vehicleYawRate) * vehicleSpeed;
```
Kami memodifikasinya secara permanen untuk model roda diferensial:
```cpp
// MODIFIKASI KINEMATIKA RODA DIFERENSIAL (TURTLEBOT):
if (omniDirGoalThre <= 0) {
    cmd_vel.twist.linear.x = vehicleSpeed;
    cmd_vel.twist.linear.y = 0.0; // Terkunci nol mutlak
}
cmd_vel.twist.angular.z = vehicleYawRate;
pubSpeedTwist->publish(cmd_vel.twist);
```

### Langkah 3: Porting & Migrasi TARE Planner ke ROS 2 Jazzy
Paket `tare_planner` awalnya menggunakan pustaka statis eksternal bawaan `or-tools/lib/libortools.so` dan arsip statis Abseil lama yang menyebabkan tabrakan simbol (*ODR flag registration collision* pada `stderrthreshold`) dengan ROS 2 Jazzy.
- Kami memodifikasi [`CMakeLists.txt`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/cmu_autonomy_stack/src/exploration_planner/tare_planner/CMakeLists.txt) milik `tare_planner`:
  1. Menggunakan `find_package(ortools_vendor REQUIRED)` dan `find_package(ortools REQUIRED)` native bawaan ROS 2 Jazzy.
  2. Mengaitkan target `tsp_solver` dan `tare_planner_node` langsung ke `ortools::ortools`.
  3. Memasukkan header `ORTOOLS_INCLUDE_DIRS` (`/opt/ros/jazzy/opt/ortools_vendor/include`) secara eksplisit ke seluruh modul C++ dependent.
  4. Menghapus ketergantungan path statis lama `/opt/ros/humble/include`.

### Langkah 4: Pembuatan Launch Terpadu & RViz Otonom
1. Dibuat [`sim_exploration.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_bringup/launch/sim_exploration.launch.py) yang mengorkestrasi Gazebo Harmonic, 3D LiDAR VLP-16, `cmu_sim_bridge`, `terrain_analysis`, `sensor_scan_generation`, `local_planner`, dan `tare_planner_node` dengan pengaturan waktu *delay* aman.
2. Dibuat visualisasi khusus [`exploration.rviz`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_bringup/rviz/exploration.rviz) yang menampilkan model robot 3D, peta elevasi kontur medan pelangi, kubus ruang eksplorasi, titik frontier, dan jalur global TSP.

---

## 5. PENJELASAN PARAMETER RINCI & DETAIL

### 5.1 Parameter Kunci `terrain_analysis` (`terrain_analysis.launch`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `scanVoxelSize` | `0.05` | double (m) | Ukuran voxel penyaring pointcloud mentah ($5\text{ cm}$) sebelum dianalisis. |
| `decayTime` | `2.0` | double (s) | Durasi waktu ingatan titik medan lama sebelum dihapus dari memori. |
| `noDecayMinShifts` | `0.5` | double (m) | Jarak pergeseran robot minimum agar titik medan dipertahankan tanpa luruh. |
| `clearingDis` | `8.0` | double (m) | Radius pembersihan sel di luar jangkauan sensor aktif. |
| `minRelZ` | `-1.5` | double (m) | Batas bawah relatif ketinggian titik terhadap posisi robot yang diakui. |
| `maxRelZ` | `0.2` | double (m) | Batas atas relatif ketinggian titik (titik di atas $0.2\text{ m}$ dianggap rintangan penghalang bodi). |
| `quantileZ` | `0.25` | double | Fraksi kuantil ketinggian untuk menentukan permukaan tanah ($25\%$ terendah). |
| `considerDrop` | `true` | bool | Mengaktifkan algoritma deteksi rintangan negatif (jurang / celah lantai). |
| `limitGroundLift` | `false` | bool | Mencegah estimasi tanah melonjak tiba-tiba saat melewati undakan tajam. |

### 5.2 Parameter Kunci `local_planner` (`local_planner.launch`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `maxSpeed` | `0.6` | double (m/s) | Batas kecepatan linear maju maksimum robot. |
| `autonomySpeed` | `0.6` | double (m/s) | Kecepatan nominal navigasi otonom saat melacak lintasan kisi. |
| `maxAccel` | `1.0` | double (m/s²) | Batas akselerasi linear maksimum untuk mencegah sentakan motor. |
| `initSpeed` | `0.0` | double (m/s) | Kecepatan awal saat modul baru diaktifkan. |
| `omniDirGoalThre` | `-1.0` | double | Nilai $\le 0$ memaksa penguncian mode **Roda Diferensial Murni** ($v_y = 0.0$). |
| `twoWayDrive` | `false` | bool | Jika `false`, robot hanya boleh melaju maju menghadap sasaran (tidak mundur). |
| `goalClearRange` | `0.5` | double (m) | Jarak toleransi di sekitar sasaran di mana pemeriksaan tabrakan dilonggarkan. |
| `searchRadius` | `0.45` | double (m) | Radius zona bebas rintangan di sekitar setiap titik lintasan kandidat. |

### 5.3 Parameter Kunci `tare_planner` (`indoor_small.yaml`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `kAutoStart` | `true` | bool | Jika `true`, robot langsung memulai eksplorasi otomatis sesaat setelah node aktif tanpa perlu pemicu eksternal. |
| `kRushHome` | `true` | bool | Robot akan otomatis merencanakan rute kembali ke titik awal saat seluruh area selesai dieksplorasi. |
| `rolling_occupancy_grid/resolution_x` | `0.2` | double (m) | Resolusi kisi okupansi 3D TARE ($20\text{ cm}$). |
| `rolling_occupancy_grid/resolution_y` | `0.2` | double (m) | Resolusi voxel sumbu Y. |
| `rolling_occupancy_grid/resolution_z` | `0.2` | double (m) | Resolusi voxel sumbu Z. |
| `kSensorRange` | `3.0` | double (m) | Jangkauan sensor efektif yang diasumsikan untuk menghapus status unknown sel voxel. |
| `kLookAheadDistance` | `4.0` | double (m) | Radius jendela perencanaan lokal (*Local Planning Horizon*). |
| `kViewPointCollisionMargin` | `0.35` | double (m) | Margin jarak aman minimum antara titik kandidat sudut pandang dengan rintangan terdekat. |
| `kGridWorldCellHeight` | `3.0` | double (m) | Tinggi kubus sub-ruang global (*subspace cell*). |
| `kGridWorldNearbyGridNum` | `5` | int | Jumlah sel tetangga yang dipertimbangkan dalam kalkulasi TSP transisi global. |
| `kMinAddFrontierPointNum` | `8` | int | Jumlah sel frontier minimum agar sebuah klaster dianggap valid untuk dikunjungi. |

---

## 6. CARA MENJALANKAN FITUR CMU

### 6.1 Mode Eksplorasi Otonom 3D (CMU TARE Planner - 1 Klik)
Untuk menjelajahi lingkungan garasi 3D tanpa peta awal secara mandiri:
```bash
distrobox enter ros2-jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup sim_exploration.launch.py world:=garage.world
```

### 6.2 Mode Cakupan Penuh 3D (CMU Autonomy + Fields2Cover)
Untuk menyapu area medan berkontur yang dibatasi poligon tertentu:
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu world:=garage.world lidar_mode:=3d
```
1. RViz2 3D Orbit akan terbuka.
2. Gunakan tombol **"Publish Point"** untuk menggambar poligon batas area kerja di lantai garasi.
3. Picu komputasi rute:
   ```bash
   ros2 service call /finish_field_boundary std_srvs/srv/Trigger
   ```
4. Robot akan menyapu area berkontur secara rapi dengan kontrol lokal CMU tanpa selip!

---

## 7. REFERENSI ILMIAH & BACAAN WAJIB

1. **Paper Kunci TARE Planner**:
   - Shen, C., Zhang, J., & Scherer, S. (2021). *TARE: A Hierarchical Framework for Rapid and Complete 3D Exploration*. Science Robotics / Carnegie Mellon University Robotics Institute.
2. **Paper Kunci CMU Autonomous Exploration**:
   - Zhang, J., & Singh, S. (2014). *LOAM: Lidar Odometry and Mapping in Real-time*. Robotics: Science and Systems (RSS).
3. **Paper Motion Lattice Rough Terrain Navigation**:
   - Howard, T. M., Green, C. J., Kelly, A., & Ferguson, D. (2008). *State Space Sampling of Feasible Motions for High-Performance Mobile Robot Navigation in Complex Terrain*. Journal of Field Robotics, 25(6‐7), 325-345.
4. **Google OR-Tools Routing Documentation**:
   - Google Developers. *Vehicle Routing Problem (VRP) & Traveling Salesperson Problem (TSP) Solver in C++*. https://developers.google.com/optimization/routing.

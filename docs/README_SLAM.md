# DOKUMENTASI LENGKAP & PANDUAN IMPLEMENTASI: SLAM
## (Simultaneous Localization and Mapping - 2D & 3D OctoMap)

Dokumentasi ini menyajikan panduan komprehensif implementasi subsistem **SLAM (Simultaneous Localization and Mapping)** pada repositori `navigation_ros_ws`. Dokumen ini mencakup landasan matematis, arsitektur perangkat lunak, alur rekayasa modifikasi, referensi ilmiah wajib, serta rincian parameter tuning.

---

## 1. PENDAHULUAN & PRINSIP FUNDAMENTAL SLAM

### 1.1 Definisi Masalah SLAM
Masalah SLAM dirumuskan sebagai estimasi simultan terhadap posisi/pose robot $\mathbf{x}_{1:t}$ dan representasi peta lingkungan $\mathbf{m}$, berdasarkan riwayat pengukuran sensor $\mathbf{z}_{1:t}$ dan perintah kontrol odometri $\mathbf{u}_{1:t}$:

$$P(\mathbf{x}_{1:t}, \mathbf{m} \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t})$$

Di mana:
- $\mathbf{x}_t = [x_t, y_t, \theta_t]^T \in SE(2)$: Pose robot pada waktu $t$ (posisi Cartesian 2D dan orientasi Yaw).
- $\mathbf{u}_t = [v_t, \omega_t]^T$: Kecepatan translasi dan rotasi robot (odometri roda).
- $\mathbf{z}_t = \{z_{t,k}\}_{k=1}^K$: Sekumpulan berkas pembacaan sensor LiDAR (jarak $r$ dan sudut $\phi$).
- $\mathbf{m}$: Peta lingkungan (dapat berupa *2D Occupancy Grid* atau *3D OctoMap Voxel Tree*).

### 1.2 Model Probabilitas & Log-Odds Occupancy Grid
Dalam pemetaan berbasis kisi okupansi (*occupancy grid mapping*), peta $\mathbf{m}$ dipecah menjadi sel-sel biner independen $m_i \in \{\text{free}(0), \text{occupied}(1)\}$. Untuk menghindari ketidakstabilan numerik akibat perkalian probabilitas berulang ($[0, 1]$), digunakan transformasi **Log-Odds**:

$$l(m_i) = \ln \left( \frac{P(m_i = 1)}{1 - P(m_i = 1)} \right) = \ln \left( \frac{P(m_i = 1)}{P(m_i = 0)} \right)$$

Pembaruan rekursif Log-Odds saat menerima pengukuran baru $\mathbf{z}_t$:

$$l_t(m_i) = l_{t-1}(m_i) + \underbrace{\ln \left( \frac{P(m_i \mid \mathbf{z}_t, \mathbf{x}_t)}{1 - P(m_i \mid \mathbf{z}_t, \mathbf{x}_t)} \right)}_{\text{Inverse Sensor Model}} - \underbrace{l_0(m_i)}_{\text{Prior}}$$

Ketika nilai $l_t(m_i)$ dievaluasi, probabilitas okupansi dapat dikembalikan melalui fungsi sigmoid:

$$P(m_i = 1 \mid \mathbf{z}_{1:t}, \mathbf{x}_{1:t}) = 1 - \frac{1}{1 + \exp(l_t(m_i))}$$

### 1.3 Scan Matching & Korelasi Ruang
Ketika odometri mengalami selip roda (*wheel slip*), estimasi posisi robot dikoreksi menggunakan algoritma pencocokan pindaian (*scan matching*).
- **Correlative Scan Matching (CSM)**: Mencari pergeseran pose $\mathbf{x} = (\delta x, \delta y, \delta \theta)$ dalam jendela pencarian diskrit $\mathcal{W}$ yang memaksimumkan skor kecocokan sinar LiDAR terhadap peta sebelumnya:
  $$\mathbf{x}^* = \arg\max_{\mathbf{x} \in \mathcal{W}} \sum_{k=1}^K M(T_{\mathbf{x}} p_k)$$
  di mana $p_k$ adalah koordinat titik pantulan LiDAR lokal, $T_{\mathbf{x}}$ adalah matriks transformasi homogen 2D, dan $M(\cdot)$ adalah nilai okupansi peta.
- **Ceres Scan Matching (Non-Linear Least Squares)**: Menyempurnakan pencocokan pada tingkat sub-piksel dengan meminimalkan galat kuadrat menggunakan pengali Levenberg-Marquardt:
  $$\min_{\Delta \mathbf{x}} \frac{1}{2} \sum_{k=1}^K \left( 1 - M(T_{\mathbf{x}_0 + \Delta \mathbf{x}} p_k) \right)^2$$

### 1.4 Optimasi Graf Pose (Pose Graph Optimization - PGO)
Untuk mengatasi akumulasi pergeseran jangka panjang (*drift*), sistem membangun graf pose:
- **Node**: Pose robot $\mathbf{x}_i \in SE(2)$ pada waktu ke-$i$.
- **Edge**: Kendala relatif $\mathbf{z}_{ij}$ (dari odometri atau deteksi *Loop Closure*).

Fungsi objektif optimasi graf meminimalkan galat residual Mahalanobis:

$$F(\mathbf{x}) = \sum_{(i,j) \in \mathcal{E}} \mathbf{e}_{ij}^T \mathbf{\Omega}_{ij} \mathbf{e}_{ij}$$

Di mana:
$$\mathbf{e}_{ij} = \mathbf{z}_{ij}^{-1} \cdot (\mathbf{x}_i^{-1} \mathbf{x}_j)$$
$$\mathbf{\Omega}_{ij} = \mathbf{\Sigma}_{ij}^{-1} \quad (\text{Matriks Informasi / Kovariansi Terbalik})$$

Saat robot kembali ke tempat yang pernah dikunjungi (*Loop Closure Detection* menggunakan *Branch-and-Bound*), tepi baru ditambahkan antara node lampau dan node saat ini, memicu relaksasi graf (*graph relaxation*) yang mengoreksi seluruh lintasan peta secara global.

### 1.5 Pemetaan 3D Probabilistik Berbasis Octree (OctoMap)
Untuk data 3D point cloud dari LiDAR 16-channel:
- Ruang 3D dimodelkan sebagai pohon oktal bersarang (*octree*), di mana setiap node kubus membelah menjadi 8 anak (*octants*).
- Voxel yang tidak memuat informasi dapat dipangkas (*pruned*) jika semua anak memiliki status probabilitas identik, menghasilkan efisiensi memori ekstrem.
- Pembaruan probabilitas menggunakan model sensor sinar (*ray casting*):
  $$L(n \mid z_{1:t}) = \max \left( \min \left( L(n \mid z_{1:t-1}) + \Delta l(z_t), l_{\text{max}} \right), l_{\text{min}} \right)$$
  - $\Delta l = l_{\text{hit}} > 0$ untuk sel ujung berkas.
  - $\Delta l = l_{\text{miss}} < 0$ untuk sel sepanjang lintasan berkas.

---

## 2. ARSITEKTUR PERANGKAT LUNAK SLAM DI REPO

Implementasi SLAM di repositori ini berada di bawah paket `src/navigation_ros_research/robot_mapping` dan diorkestrasi melalui `robot_bringup`:

```
┌────────────────────────────────────────────────────────────────────────┐
│                        GAZEBO SIMULASI HARMONIC                        │
│                                                                        │
│   [Differential Drive Robot] ──► /odom (nav_msgs/Odometry)             │
│   [LiDAR 2D / 3D VLP-16]     ──► /scan (LaserScan)                     │
│                                  /points (sensor_msgs/PointCloud2)     │
└──────────────────┬─────────────────────────────┬───────────────────────┘
                   │                             │
                   ▼                             ▼
       ┌──────────────────────┐      ┌─────────────────────────┐
       │     SLAM TOOLBOX     │      │         OCTOMAP         │
       │    (Mode 2D SLAM)    │      │     (Mode 3D Voxel)     │
       │                      │      │                         │
       │ • Scan Matcher       │      │ • 3D Ray Casting        │
       │ • Loop Closer (Karto)│      │ • Octree Compression    │
       │ • Pose Graph Solver  │      │ • Collision Volume      │
       └──────────┬───────────┘      └───────────┬─────────────┘
                  │                              │
                  ▼                              ▼
          [/map] (2D Grid)            [/octomap_binary]
          [TF: map -> odom]           [/projected_map]
```

### File-File Kunci Subsistem SLAM:
1. [`robot_mapping/launch/slam.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/launch/slam.launch.py): Menjalankan SLAM Toolbox dalam mode sinkron online.
2. [`robot_mapping/launch/octomap_mapping.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/launch/octomap_mapping.launch.py): Menjalankan server OctoMap 3D yang berlangganan ke `/points`.
3. [`robot_mapping/config/mapper_params_online_sync.yaml`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/config/mapper_params_online_sync.yaml): Konfigurasi parameter inti SLAM Toolbox.
4. [`robot_bringup/launch/sim_mapping.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_bringup/launch/sim_mapping.launch.py): Launch terpadu simulasi + SLAM + teleoperasi.
5. [`robot_bringup/launch/sim_auto_mapping.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_bringup/launch/sim_auto_mapping.launch.py): Pemetaan otonom menggunakan Frontier Explorer.

---

## 3. ALUR TEKNIS MEMBUAT FITUR SLAM BERJALAN DI REPO INI

Berikut adalah tahapan rekayasa langkah demi langkah yang kami lakukan untuk mengintegrasikan SLAM di repositori ini:

### Langkah 1: Standardisasi Pohon Transformasi Koordinat (TF Tree)
Agar SLAM Toolbox dapat beroperasi tanpa kehilangan orientasi, rantai TF harus terdefinisi tanpa celah:
$$\text{map} \xrightarrow{\text{SLAM}} \text{odom} \xrightarrow{\text{Gazebo Diff-Drive}} \text{base_footprint} \xrightarrow{\text{URDF}} \text{base_link} \xrightarrow{\text{URDF}} \text{lidar_link}$$
- Jika `map -> odom` belum dipublikasikan, robot tidak dapat meletakkan scan ke koordinat absolut.
- SLAM Toolbox disetel untuk mengambil alih otoritas publikasi transformasi dinamis `map -> odom`.

### Langkah 2: Sinkronisasi Waktu Simulasi (`use_sim_time`)
Pada simulasi Gazebo, waktu komputasi fisik tidak selalu berjalan 1:1 dengan jam dinding (*wall clock*).
- Kami menambahkan deklarasi `use_sim_time = LaunchConfiguration('use_sim_time', default='true')` pada setiap node SLAM.
- Seluruh node membaca waktu dari topik `/clock` untuk mencegah galat `TF_OLD_DATA` atau `ExtrapolationException`.

### Langkah 3: Ekstraksi LaserScan 2D dari 3D VLP-16 (Dual Mode)
Agar robot dapat memetakan menggunakan sensor 2D planar standar maupun mengekstraksi bidang horizontal dari sensor 3D VLP-16:
- Pada URDF [`lidar_3d.xacro`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_description/urdf/lidar_3d.xacro), plugin Gazebo dikonfigurasi untuk memancarkan `/points` (3D PointCloud2) sekaligus `/pointcloud/scan` (proyeksi 2D horizontal layer).
- Jembatan ROS-Gazebo menerjemahkan kedua topik ini dengan QoS `sensor_data` (Best Effort) berlatensi rendah.

### Langkah 4: Integrasi Penyimpanan Peta Otomatis
Peta 2D yang dihasilkan SLAM disimpan ke disk dalam format `.yaml` dan `.pgm` agar dapat dimuat oleh Nav2 map server:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/map/map_baru --ros-args -p use_sim_time:=true
```

---

## 4. PENJELASAN PARAMETER RINCI & DETAIL

### 4.1 Parameter Kunci SLAM Toolbox (`mapper_params_online_sync.yaml`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `resolution` | `0.05` | double (m) | Ukuran sisi sel kisi okupansi ($5\text{ cm}$). Resolusi lebih kecil menghasilkan peta lebih detail tetapi memperbesar konsumsi memori secara kuadratik ($\mathcal{O}(W \cdot H)$). |
| `max_laser_range` | `12.0` | double (m) | Jarak pancaran maksimum sinar LiDAR yang diakui untuk membangun rintangan. Sinar di atas jarak ini dianggap bebas pantulan. |
| `minimum_travel_distance` | `0.3` | double (m) | Jarak linear translasi minimum yang harus ditempuh robot sebelum pose baru ditambahkan ke Pose Graph. |
| `minimum_travel_heading` | `0.4` | double (rad) | Perubahan sudut rotasi minimum ($\approx 23^\circ$) yang memicu penambahan node baru pada graf. Mencegah graf membengkak saat robot diam. |
| `scan_buffer_size` | `10` | int | Jumlah pindaian laser yang disimpan dalam penyangga sirkular untuk pemrosesan paralel pencocokan pindaian. |
| `link_match_minimum_response_fine` | `0.1` | double | Ambang batas korelasi minimum pencocokan halus agar dua scan dianggap membentuk tautan valid. |
| `loop_search_maximum_distance` | `3.0` | double (m) | Radius lingkaran di sekitar pose robot saat ini untuk mencari calon *loop closure* dengan lintasan masa lalu. |
| `do_loop_closing` | `true` | bool | Mengaktifkan optimasi graf penutup simpul (*loop closure*). Wajib `true` untuk memetakan ruangan berputar/sirkular. |
| `loop_match_minimum_chain_size` | `10` | int | Jumlah node berturut-turut minimum yang harus cocok sebelum deformasi graf dieksekusi. |
| `correlation_search_space_dimension` | `0.5` | double (m) | Jendela pencarian linear translasi $(\pm 0.5\text{ m})$ untuk mencocokkan scan saat pencarian simpul. |
| `correlation_search_space_resolution` | `0.01` | double (m) | Resolusi kisi diskrit pencarian translasi ($1\text{ cm}$). |

### 4.2 Parameter Kunci OctoMap (`octomap_mapping.launch.py`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `resolution` | `0.1` | double (m) | Resolusi voxel daun terkecil pohon oktal ($10\text{ cm}$). |
| `frame_id` | `map` | string | Sistem koordinat acuan absolut penyimpanan voxel 3D. |
| `sensor_model/max_range` | `15.0` | double (m) | Batas jangkauan pemindaian berkas sinar 3D. |
| `sensor_model/hit` | `0.7` | double | Probabilitas kenaikan okupansi $P(\text{hit})$ saat sinar memantul. Dalam log-odds: $l_{\text{hit}} = \ln(0.7 / 0.3) \approx +0.847$. |
| `sensor_model/miss` | `0.4` | double | Probabilitas penurunan okupansi $P(\text{miss})$ sepanjang berkas tembus: $l_{\text{miss}} = \ln(0.4 / 0.6) \approx -0.405$. |
| `sensor_model/min` | `0.12` | double | Batas bawah penjepitan probabilitas ($l_{\text{min}}$) agar sel kosong dapat cepat berubah jika ada objek baru. |
| `sensor_model/max` | `0.97` | double | Batas atas penjepitan probabilitas ($l_{\text{max}}$). |
| `pointcloud_min_z` | `0.05` | double (m) | Batas bawah ketinggian titik pointcloud yang diproses (menyingkirkan pantulan lantai datar agar tidak menjadi rintangan semu). |
| `pointcloud_max_z` | `2.0` | double (m) | Batas atas ketinggian titik yang diproses (mengabaikan langit-langit/atap). |

---

## 5. CARA MENJALANKAN FITUR SLAM

### 5.1 Mode Pemetaan Manual (Teleop Keyboard)
1. Jalankan simulasi dan SLAM Toolbox:
   ```bash
   distrobox enter ros2-jazzy
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch robot_bringup sim_mapping.launch.py world:=turtlebot3_house.world
   ```
2. Pada terminal baru, jalankan pengontrol keyboard:
   ```bash
   distrobox enter ros2-jazzy
   source /opt/ros/jazzy/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. Kemudikan robot menyusuri seluruh ruangan hingga peta terbentuk sempurna di RViz.
4. Simpan peta:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/map/peta_house --ros-args -p use_sim_time:=true
   ```

### 5.2 Mode Pemetaan Otomatis (Autonomous Frontier Mapping)
```bash
ros2 launch robot_bringup sim_auto_mapping.launch.py world:=turtlebot3_house.world
```
Robot akan secara otomatis mendeteksi batas terluar peta (*frontier exploration*) dan menjelajahi rumah secara mandiri hingga pemetaan selesai.

---

## 6. REFERENSI ILMIAH & BACAAN WAJIB

1. **Buku Fundamental**:
   - Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. The MIT Press. (Wajib dibaca: Bab 9 "Occupancy Grid Mapping", Bab 10 "Simultaneous Localization and Mapping", Bab 11 "GraphSLAM").
2. **Paper Kunci SLAM Toolbox**:
   - Macenski, S., & Jambrecic, I. (2021). *SLAM Toolbox: SLAM for the dynamic and beyond*. Journal of Open Source Software (JOSS), 6(61), 2783.
3. **Paper Kunci Google Cartographer**:
   - Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). *Real-Time Loop Closure in 2D LIDAR SLAM*. IEEE International Conference on Robotics and Automation (ICRA).
4. **Paper Kunci OctoMap 3D**:
   - Hornung, A., Wurm, K. M., Bennewitz, M., Stachniss, C., & Burgard, W. (2013). *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees*. Autonomous Robots, 34(3), 189-206.

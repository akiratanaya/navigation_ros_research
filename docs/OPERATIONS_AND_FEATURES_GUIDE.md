# PANDUAN OPERASIONAL & CARA MENJALANKAN FITUR SISTEM
## Panduan Langkah Demi Langkah Menjalankan Seluruh Mode Navigasi (Nav2, CMU Stack, Coverage Planning)

---

## 1. PERSIAPAN SISTEM & WORKSPACE

Seluruh sistem berjalan di dalam container **ROS 2 Jazzy** melalui **Distrobox**.

### 1.1 Membuka Terminal dan Masuk Container
Buka terminal host Anda, lalu jalankan:
```bash
distrobox enter ros2-jazzy
```

### 1.2 Masuk ke Direktori Workspace & Source Environment
```bash
cd ~/robotics/delabo_itb/navigation_ros_ws
source /opt/ros/jazzy/setup.bash
```

### 1.3 Kompilasi Workspace (Colcon Build)
Jika Anda baru pertama kali menjalankan atau baru melakukan perubahan kode:
```bash
colcon build --symlink-install
source install/setup.bash
```
> **Catatan**: Workspace terdiri dari 11 paket aktif dan terkompilasi bersih dalam waktu $\approx 3$ detik tanpa error.

---

## 2. FITUR 1: SIMULASI ROBOT & WORLD GAZEBO

Anda dapat menjalankan robot secara mandiri di berbagai jenis dunia simulasi (Gazebo Harmonic).

### 2.1 Simulasi di Lingkungan Indoor Rumah Datar (`turtlebot3_house.world`)
Mode sensor 2D LiDAR planar:
```bash
ros2 launch robot_description sim.launch.py world:=turtlebot3_house.world lidar_mode:=2d
```

### 2.2 Simulasi di Lingkungan Garasi Medan 3D (`garage.world`)
Mode sensor 3D LiDAR 16-channel:
```bash
ros2 launch robot_description sim.launch.py world:=garage.world lidar_mode:=3d x:=0.0 y:=0.0 z:=0.1
```

### 2.3 Kontrol Manual Robot (Teleoperasi Keyboard)
Buka terminal baru di dalam container `ros2-jazzy`:
```bash
distrobox enter ros2-jazzy
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 3. FITUR 2: PEMETAAN LINGKUNGAN 2D (MAPPING / SLAM)

Untuk membuat peta 2D baru (`map.yaml` dan `map.pgm`) menggunakan Cartographer atau SLAM Toolbox:

### 3.1 Menjalankan Simulasi + Modul SLAM
```bash
# Terminal 1: Jalankan simulasi Gazebo
ros2 launch robot_description sim.launch.py world:=turtlebot3_house.world lidar_mode:=2d

# Terminal 2: Jalankan modul SLAM
ros2 launch robot_mapping cartographer.launch.py use_sim_time:=true
```

### 3.2 Menggerakkan Robot Menyusuri Ruangan
Gunakan `teleop_twist_keyboard` di terminal 3 untuk menjelajah seluruh ruangan hingga peta pada RViz terbentuk sempurna.

### 3.3 Menyimpan Peta
Setelah seluruh ruangan terpetakan:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_mapping/map/my_new_map
```

---

## 4. FITUR 3: NAVIGASI POINT-TO-POINT STANDAR DENGAN NAV2

Mode ini digunakan untuk memindahkan robot dari satu titik ke titik tujuan tertentu pada peta 2D dengan menghindari rintangan statis/dinamis.

### 4.1 Menjalankan Navigasi Penuh Nav2
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2 auto_navigate:=false
```

### 4.2 Langkah Operasi di RViz:
1. Tunggu $\sim 10$ detik hingga siklus hidup (*lifecycle*) Nav2 mencapai status `ACTIVE`.
2. Klik tombol **"2D Goal Pose"** pada toolbar atas RViz.
3. Klik pada titik tujuan di peta dan tarik panah untuk menentukan arah hadap (*heading*) akhir yang diinginkan.
4. **Respon Robot**:
   - `NavfnPlanner` akan menghitung jalur terpendek bebas tabrakan (garis hijau).
   - `RegulatedPurePursuitController` akan mengarahkan robot menuju tujuan dengan mematuhi limit akselerasi dan deselerasi.

---

## 5. FITUR 4: COVERAGE NAVIGATION DENGAN NAV2 BACKEND (2D DATAR)

Fitur ini menjalankan sapuan pembersihan lahan menyeluruh (*Complete Coverage Path Planning*) menggunakan **Fields2Cover** yang dieksekusi oleh tumpukan **ROS 2 Nav2**.

### 5.1 Perintah Eksekusi Satu Baris
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2 world:=turtlebot3_house.world lidar_mode:=2d
```

### 5.2 Langkah Operasional di RViz:
1. Tunggu Gazebo dan RViz terbuka secara otomatis.
2. Klik tombol **"Publish Point"** pada toolbar atas RViz.
3. Klik minimal 3 atau 4 titik di lantai ruangan untuk membentuk poligon batas area kerja (*field boundary*). Titik-titik ini akan dihubungkan dengan garis kuning.
4. Buka terminal baru (atau jalankan perintah) untuk memicu kalkulasi:
   ```bash
   distrobox enter ros2-jazzy -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 service call /finish_field_boundary std_srvs/srv/Trigger"
   ```

### 5.3 Apa yang Terjadi Secara Otomatis:
1. **Ekstraksi Rintangan**: Node `coverage_server` mendeteksi rintangan interior (kaki meja) dari `/map`.
2. **Pembangkitan Jalur 2-Fase**:
   - **Fase 1 (Cyan Path)**: Robot terlebih dahulu melakukan *Perimeter Tour* melingkari batas dinding ruangan.
   - **Fase 2 (Yellow Path)**: Robot melanjutkan sapuan *Infill Boustrophedon* bolak-balik dengan *smooth circular bypass* di sekitar kaki meja.
3. **Eksekusi Navigasi**: `coverage_navigator` mengirimkan rute ke Nav2 action server `NavigateThroughPoses`.
4. **Visualisasi Jejak Sapuan**: Node `footprint_trail_visualizer` akan mewarnai lantai dengan jejak hijau transparan sesuai lebar sapuan robot ($0.24\text{ m}$) dan menampilkan persentase area yang telah dibersihkan.

---

## 6. FITUR 5: NAVIGASI MEDAN 3D DENGAN CMU AUTONOMY STACK

Pada dunia berkontur seperti `garage.world` yang memiliki tanjakan dan rintangan 3D, CMU Autonomy Stack mengabaikan peta 2D statis dan mengandalkan analisis real-time elevasi point cloud 3D LiDAR.

### 6.1 Menjalankan Sistem Navigasi CMU
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu world:=garage.world lidar_mode:=3d x:=0.0 y:=0.0 z:=0.1 auto_navigate:=false
```

### 6.2 Navigasi Titik ke Titik (Point-to-Point 3D)
Untuk mengirimkan titik target manual ke CMU Local Planner:
```bash
distrobox enter ros2-jazzy -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic pub --once /way_point geometry_msgs/msg/PointStamped '{header: {frame_id: \"map\"}, point: {x: 4.0, y: 2.0, z: 0.0}}'"
```
- **Respon Robot**:
  - `terrainAnalysis` memindai kontur permukaan tanah di depan robot dan mengukur kelerengan tanjakan.
  - `localPlanner` memilih salah satu dari 343 lintasan kisi (*motion primitive lattice*) yang paling mulus dan bebas benturan.
  - `pathFollower` menggerakkan roda diferensial robot mendaki tanjakan atau mengitari rintangan 3D.

---

## 7. FITUR 6: COVERAGE NAVIGATION DENGAN CMU BACKEND DI MEDAN 3D

Ini adalah fitur terlengkap: menggabungkan kecerdasan cakupan area **Fields2Cover** dengan ketangguhan penjelajahan medan 3D **CMU Autonomy Stack**.

### 7.1 Perintah Eksekusi Utama
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu world:=garage.world lidar_mode:=3d x:=0.0 y:=0.0 z:=0.1
```

### 7.2 Langkah Pengoperasian:
1. RViz khusus CMU (`coverage_cmu.rviz`) akan terbuka dalam mode perspektif 3D Orbit.
2. Gunakan tombol **"Publish Point"** di RViz untuk menentukan batas area kerja di lantai garasi (misalnya mengelilingi area tanjakan).
3. Panggil service pemicu:
   ```bash
   distrobox enter ros2-jazzy -- bash -c "source /opt/ros/jazzy/setup.bash && ros2 service call /finish_field_boundary std_srvs/srv/Trigger"
   ```
4. **Aliran Kerja Lapangan**:
   - `coverage_server` menghitung jalur sapuan global `/coverage_path`.
   - `cmu_coverage_navigator` membaca rute dan mulai mendistribusikan waypoint satu per satu ke topik `/way_point`.
   - `local_planner` CMU secara real-time memastikan robot bergerak aman melintasi permukaan berkontur 3D tanpa terjebak atau terbalik.
   - Progres kunjungan waypoint dapat dipantau langsung pada log terminal atau topik `/state_estimation`.

---

## 8. FITUR 7: EKSPLORASI OTONOM 3D (CMU TARE PLANNER - 1-KLIK)

Fitur ini menggunakan algoritma **CMU TARE (Terrain-Aware Autonomous Exploration) Planner** untuk melakukan penjelajahan dan pemetaan otonom penuh secara 3D di lingkungan yang belum diketahui tanpa memerlukan peta awal (*zero prior map*). Robot secara aktif mendeteksi perbatasan area yang belum diketahui (*frontiers*), membagi ruang ke dalam sub-ruang hierarkis, merencanakan urutan kunjungan optimal melalui *Traveling Salesman Problem* (TSP) dengan Google OR-Tools, dan memandu robot mengelilingi seluruh area hingga tuntas 100%.

### 8.1 Menjalankan Eksplorasi Otonom di Lingkungan Garasi 3D (`garage.world`)
Jalankan satu perintah berikut di dalam container:
```bash
ros2 launch robot_bringup sim_exploration.launch.py world:=garage.world
```

### 8.2 Menjalankan di Lingkungan Rumah (`turtlebot3_house.world`)
```bash
ros2 launch robot_bringup sim_exploration.launch.py world:=turtlebot3_house.world x:=-2.0 y:=1.0 z:=0.05
```

### 8.3 Apa yang Terjadi Secara Otomatis:
1. **Simulasi Gazebo**: Robot di-spawn lengkap dengan sensor 3D LiDAR 16-channel.
2. **CMU Base Autonomy Stack**:
   - `cmu_sim_bridge` menjembatani odometri dan pointcloud ke frame TF koordinat global `map`.
   - `terrain_analysis` merekonstruksi kontur medan jalan secara live (`/terrain_map`).
   - `local_planner` mengendalikan dinamika gerak roda diferensial ($v_y = 0$) menghindari rintangan tajam.
3. **TARE Planner Node**:
   - Berjalan otomatis (`auto_start:=true`) menghitung voxel rolling occupancy grid.
   - Menghasilkan rute eksplorasi global dan lokal tercepat, lalu mengirimkan target ke `/way_point`.
4. **Visualisasi RViz2 Otonom (`exploration.rviz`)**:
   - **Frontier Surfaces (Magenta)**: Permukaan batas antara area yang sudah terdeteksi dan yang belum terjamah.
   - **Exploring Subspaces (Kubus Gradien)**: Kotak-kotak ruang penjelajahan aktif yang dievaluasi algoritma TARE.
   - **Global Exploration Path (Cyan)**: Garis rute global hasil optimasi TSP OR-Tools.
   - **Local Trajectory Path (Hijau)**: Lintasan gerak dinamis aktual yang dilalui robot.
   - **Target Waypoint (Bola Oranye)**: Titik tujuan aktif yang sedang dituju robot.

---

## 9. PANDUAN KUSTOMISASI PARAMETER (TUNING GUIDE)

Anda dapat menyesuaikan parameter sistem saat menjalankan launch file tanpa perlu mengompilasi ulang kode:

### 8.1 Parameter Kecepatan & Toleransi
| Nama Parameter | Default | Keterangan & Rekomendasi |
| :--- | :---: | :--- |
| `max_speed` | `0.6` | Batas kecepatan linear maksimum robot ($\text{m/s}$). |
| `autonomy_speed` | `0.6` | Kecepatan jelajah otonom CMU Local Planner ($\text{m/s}$). |
| `waypoint_tolerance` | `0.35` | Toleransi jarak robot ke waypoint sebelum maju ke titik berikutnya ($\text{m}$). |
| `auto_navigate` | `true` | Jika `true`, robot langsung bergerak otomatis begitu rute coverage selesai dihitung. |

Contoh pemanggilan:
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu world:=garage.world lidar_mode:=3d max_speed:=0.8 waypoint_tolerance:=0.25
```

### 8.2 Parameter Pola Rute & Dekomposisi Fields2Cover
| Nama Parameter | Pilihan Nilai | Keterangan |
| :--- | :--- | :--- |
| `decomp_method` | `boustrophedon`, `trapezoidal`, `none` | Metode pemecahan poligon non-cembung menjadi sub-sel. |
| `route_pattern` | `boustrophedon`, `snake`, `spiral` | Pola urutan penyapuan baris swaths. |

Contoh pemanggilan:
```bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2 decomp_method:=boustrophedon route_pattern:=spiral
```

---

## 10. PANDUAN PEMECAHAN MASALAH (TROUBLESHOOTING & FAQS)

### Q1: Perintah `colcon build` atau `ros2` tidak ditemukan di terminal?
**Penyebab**: Anda berada di terminal OS host, bukan di dalam container Ubuntu Jazzy.  
**Solusi**: Jalankan `distrobox enter ros2-jazzy`, lalu `source /opt/ros/jazzy/setup.bash`.

### Q2: Robot Nav2 berputar-putar di tempat dan tidak mulai melacak rute?
**Penyebab**: Sudut hadap awal robot berselisih $> 45^\circ$ dengan arah lintasan pertama.  
**Solusi**: Ini adalah fitur *Rotate-to-Heading* bawaan Regulated Pure Pursuit untuk keamanan roda diferensial. Robot akan berputar di tempat dengan aman hingga searah lintasan sebelum melaju.

### Q3: Pada CMU Stack, robot tidak bergerak meskipun `/way_point` sudah dipublikasikan?
**Penyebab**: 
1. Sensor PointCloud 3D belum aktif (pastikan `lidar_mode:=3d`).
2. Node `cmu_sim_bridge` belum berjalan (periksa dengan `ros2 node list`).
3. Titik target berada di dalam zona terlarang rintangan ($\Delta Z > 0.2\text{ m}$). Coba berikan koordinat titik di lantai terbuka.

### Q4: Apakah saya bisa berganti kembali ke Nav2 kapan saja?
**Jawaban**: Tentu saja! Anda cukup mengganti argumen `nav_backend:=nav2` saat meluncurkan launch file. Kedua tumpukan navigasi terisolasi sempurna dan tidak saling menginterferensi satu sama lain.

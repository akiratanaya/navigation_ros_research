# DOKUMENTASI LENGKAP & PANDUAN IMPLEMENTASI: NAV2 & COVERAGE
## (ROS 2 Navigation Stack & Fields2Cover Complete Path Planning)

Dokumentasi ini menyajikan panduan mendalam untuk subsistem **Nav2 (Navigation 2 Stack)** serta integrasi **Fields2Cover Coverage Path Planning** pada repositori `navigation_ros_ws`. Dokumen ini merinci arsitektur kendali, lokalisasi adaptif AMCL, kinematika pengontrol Pure Pursuit teregulasi, formulasi dekomposisi poligon, alur integrasi rekayasa, dan tabel parameter lengkap.

---

## 1. PENDAHULUAN & PRINSIP FUNDAMENTAL NAV2

Nav2 adalah tumpukan navigasi otonom generasi terbaru untuk ROS 2. Arsitektur Nav2 dibangun di atas konsep **State Machine Bersarang Berbasis Behavior Trees (BT)** dan arsitektur siklus hidup node (*Lifecycle Nodes*).

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          BEHAVIOR TREE NAVIGATOR                            │
│                     (NavigateThroughPoses / NavigateToPose)                 │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
            ┌──────────────────────────┼──────────────────────────┐
            ▼                          ▼                          ▼
┌───────────────────────┐  ┌───────────────────────┐  ┌───────────────────────┐
│     GLOBAL PLANNER    │  │    CONTROLLER SERVER  │  │   RECOVERY BEHAVIOR   │
│  (Navfn / Smac 2D)    │  │ (Regulated Pure       │  │ (Spin, BackUp, Wait,  │
│                       │  │  Pursuit Controller)  │  │  ClearCostmaps)       │
│ • Dijkstra / A* / Dubins│• Arc Curvature Control │  │                       │
│ • Global Costmap      │  │ • Local Costmap       │  │                       │
└───────────────────────┘  └───────────────────────┘  └───────────────────────┘
```

### 1.1 Lokalisasi Monte Carlo Adaptif (AMCL)
AMCL mengestimasi pose robot $\mathbf{x}_t = (x, y, \theta)$ dalam peta statis yang sudah diketahui menggunakan **Filter Partikel KLD (Kullback-Leibler Divergence)**.
Setiap partikel $p^{[m]} = (\mathbf{x}_t^{[m]}, w_t^{[m]})$ merepresentasikan hipotesis pose robot dengan bobot $w$.

#### KLD-Sampling (Ukuran Partikel Dinamis)
Jumlah partikel $M$ dihitung secara dinamis pada setiap siklus pembaruan untuk menjamin galat aproksimasi probabilistik antara distribusi empiris dan distribusi kontinu tidak melebihi $\epsilon$ dengan keyakinan $1 - \delta$:

$$M = \frac{k - 1}{2\epsilon} \left( 1 - \frac{2}{9(k - 1)} + \sqrt{\frac{2}{9(k - 1)}} z_{1-\delta} \right)^3$$

Di mana $k$ adalah jumlah bin histogram yang terisi oleh partikel, dan $z_{1-\delta}$ adalah kuantil dari distribusi normal baku.

#### Model Pengukuran Sensor: Likelihood Field
Untuk setiap titik laser $z_{t,k}$ yang diproyeksikan ke koordinat global $(x_{z_k}, y_{z_k})$, probabilitas pengukuran dihitung berdasarkan jarak terdekat $d$ ke rintangan terdekat dalam peta:

$$P(z_{t,k} \mid \mathbf{x}_t, \mathbf{m}) = z_{\text{hit}} \cdot \frac{1}{\sqrt{2\pi \sigma_{\text{hit}}^2}} \exp\left( -\frac{d^2}{2\sigma_{\text{hit}}^2} \right) + \frac{z_{\text{rand}}}{z_{\text{max}}}$$

Di mana $z_{\text{hit}} + z_{\text{rand}} = 1$ adalah koefisien pencampuran model sensor.

### 1.2 Kinematika Pengontrol: Regulated Pure Pursuit
Robot beroda diferensial memiliki kendala non-holonomik lateral ($\dot{y}_r = 0$). Algoritma **Regulated Pure Pursuit (RPP)** menghitung kelengkungan lintasan busur lingkaran $\kappa$ menuju titik target pandang depan (*lookahead point*) $(x_L, y_L)$ pada jarak $L$:

$$\kappa = \frac{2 y_L}{L^2}$$

Kecepatan sudut rotasi $\omega$ dihitung dari kecepatan linear $v$:

$$\omega = v \cdot \kappa = v \cdot \frac{2 y_L}{L^2}$$

#### Regulasi Kecepatan Canggih RPP
Untuk mencegah selip, rollover, atau manuver tajam yang membahayakan robot di tikungan:
1. **Regulasi Berdasarkan Kelengkungan (*Curvature Regulation*)**:
   $$v_{\text{curv}} = v_{\text{max}} \cdot \frac{r_{\text{min\_radius}}}{\max(r_{\text{min\_radius}}, |\kappa|^{-1})}$$
2. **Regulasi Berdasarkan Kedekatan Rintangan (*Costmap Proximity Regulation*)**:
   $$v_{\text{prox}} = v_{\text{max}} \cdot \left( 1.0 - \frac{\text{cost}_{\text{local}}}{254} \right)^{\gamma}$$
3. **Regulasi Akselerasi Lateral**:
   $$v_{\text{accel}} = \sqrt{\frac{a_{\text{lat,max}}}{|\kappa|}}$$
4. **Kecepatan Komando Akhir**:
   $$v_{\text{cmd}} = \min(v_{\text{max}}, v_{\text{curv}}, v_{\text{prox}}, v_{\text{accel}})$$

### 1.3 Lapisan Biaya Costmap 2D (Layered Costmaps)
Costmap menggabungkan informasi spasial dalam format berlapis:
- **Static Layer**: Peta biner dari SLAM ($0 = \text{free}$, $100 = \text{occupied}$).
- **Obstacle Layer**: Pembacaan sensor LiDAR live untuk mendeteksi rintangan dinamis dengan proses *Bresenham Raytracing* untuk menghapus sel bebas.
- **Inflation Layer**: Memperluas rintangan dengan fungsi peluruhan eksponensial guna memastikan jarak aman robot radius $r_{\text{robot}}$:

$$\text{Cost}(d) = 254 \cdot \exp\left( -k \cdot (d - r_{\text{inscribed}}) \right)$$

Di mana $d$ adalah jarak Euclidean sel ke rintangan terdekat, $r_{\text{inscribed}}$ adalah radius jari-jari lingkaran bodi robot, dan $k$ adalah `cost_scaling_factor`.

---

## 2. FORMULASI MATEMATIS FIELDS2COVER (COMPLETE COVERAGE)

Cakupan area penuh (*Complete Coverage Path Planning - CCPP*) bertujuan memandu robot menyapu seluruh area tertutup $\mathcal{P} \subset \mathbb{R}^2$ dengan persentase sapuan $100\%$ dan tumpang tindih (*overlap*) minimum.

### 2.1 Dekomposisi Sel Cembung (Cell Decomposition)
Jika poligon batas area kerja $\mathcal{P}$ bersifat non-cembung (*non-convex*) atau memiliki lubang (*holes* berupa pilar rintangan):
1. **Boustrophedon Decomposition**: Menembakkan garis sapu vertikal melintasi seluruh titik verteks kritis (*inward/outward reflex vertices*) dan membelah $\mathcal{P}$ menjadi sekumpulan sub-poligon cembung $\mathcal{C} = \{c_1, c_2, \dots, c_m\}$.
2. Pada setiap sub-sel $c_j$, fungsi penyapuan garis paralel (*swaths*) dapat dibangkitkan tanpa terputus.

### 2.2 Pembangkitan Jalur Sapuan (Swaths Generation)
Untuk setiap sub-sel $c_j$, ditentukan vektor arah optimal $\vec{u} = (\cos \alpha, \sin \alpha)$. Garis sapu ke-$i$ dibentuk dengan jarak antar baris selebar sapuan alat robot $W_{\text{tool}}$ dikurangi faktor tumpang tindih $\Delta$:

$$\text{Swath}_i = \{ p \in c_j \mid \vec{u}_{\perp} \cdot p = i \cdot (W_{\text{tool}} - \Delta) \}$$

### 2.3 Putaran Antar-Baris (Headland Turns)
Untuk menghubungkan ujung baris $\text{Swath}_i$ ke pangkal baris $\text{Swath}_{i+1}$, sistem membangkitkan kurva putar non-holonomik yang sesuai dengan radius putar minimum robot diferensial $R_{\text{min}}$:
- **Dubins Curves**: Jalur terpendek $CSC$ (*Circle-Straight-Circle*) untuk gerak maju terus.
- **Spiral / Boustrophedon Sequence**: Urutan traversal baris diatur untuk meminimalkan jumlah putaran balik $180^\circ$ tajam.

---

## 3. ARSITEKTUR INTEGRASI NAV2 & FIELDS2COVER DI REPO

```
┌────────────────────────────────────────────────────────────────────────┐
│                          ROBOT_COVERAGE PACKAGE                        │
│                                                                        │
│  [RViz2: Publish Point] ──► /clicked_point                             │
│                                   │                                    │
│                                   ▼                                    │
│                     [field_boundary_collector]                         │
│                                   │                                    │
│                             /field_boundary                            │
│                                   │                                    │
│                                   ▼                                    │
│                           [coverage_server]                            │
│                 (Fields2Cover Boustrophedon Planner)                   │
│                                   │                                    │
│                            /coverage_path                              │
│                                   │                                    │
│                                   ▼                                    │
│                         [coverage_navigator]                           │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼ Action Client: NavigateThroughPoses
┌────────────────────────────────────────────────────────────────────────┐
│                              NAV2 STACK                                │
│                                                                        │
│  • amcl: Lokalisasi global robot terhadap map.yaml                     │
│  • controller_server: Regulated Pure Pursuit melacak pose list        │
│  • local_costmap: Menghindari rintangan dinamis secara live            │
│  • cmd_vel ──► Gazebo Diff-Drive Robot                                 │
└────────────────────────────────────────────────────────────────────────┘
```

### File-File Kunci Subsistem Nav2 & Coverage:
1. [`robot_navigation/config/nav2_params.yaml`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_navigation/config/nav2_params.yaml): Parameter konfigurasi terpadu AMCL, BT Navigator, Controller Server, Costmaps, dan Recovery.
2. [`robot_navigation/launch/navigation.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_navigation/launch/navigation.launch.py): Menjalankan seluruh server Nav2 terkelola (*managed lifecycle*).
3. [`robot_coverage/launch/coverage_pipeline.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_coverage/launch/coverage_pipeline.launch.py): Pipeline lengkap pengumpul poligon, generator jalur Fields2Cover, dan navigator Nav2.
4. [`robot_bringup/launch/sim_coverage.launch.py`](file:///home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/navigation_ros_research/robot_bringup/launch/sim_coverage.launch.py): Launch terpadu simulasi + Nav2 + Coverage.

---

## 4. ALUR TEKNIS MEMBUAT FITUR NAV2 BERJALAN DI REPO INI

### Langkah 1: Penataan Lifecycle Nodes
Nav2 menggunakan model siklus hidup ROS 2 (`Unconfigured` $\to$ `Inactive` $\to$ `Active`). Kami membuat konfigurasi `nav2_lifecycle_manager` di launch file untuk mengikat node:
`amcl`, `map_server`, `controller_server`, `planner_server`, `recoveries_server`, dan `bt_navigator`. Pengikatan terorkestrasi mencegah `bt_navigator` dieksekusi sebelum controller siap.

### Langkah 2: Konfigurasi Parameter Roda Diferensial (TurtleBot)
Parameter default Nav2 sering kali disetel untuk platform omnidirectional atau Ackermann besar. Kami menyesuaikan:
- Mengunci `holonomic: false` pada controller.
- Mengaktifkan `use_rotate_to_heading: true` pada Regulated Pure Pursuit. Jika orientasi awal robot berbeda $> 45^\circ$ terhadap lintasan, robot akan berputar di tempat terlebih dahulu sebelum melaju, mencegah osilasi belok liar.

### Langkah 3: Menghilangkan Crash Pemulihan / Inflasi Costmap
Pada lingkungan indoor sempit, radius inflasi default ($0.55\text{ m}$) menyebabkan jalur terputus (*no valid path found*).
- Kami menyetel `inflation_radius: 0.45` dan `cost_scaling_factor: 3.0` sehingga robot berani melewati koridor selebar $0.6\text{ m}$ dengan aman.

### Langkah 4: Jembatan Fields2Cover ke Nav2 Action Server
Alih-alih mengirimkan titik satu per satu melalui topik manual, `coverage_navigator.py` memaketkan seluruh jalur sapuan menjadi `std_msgs/Header` + list `geometry_msgs/PoseStamped`, lalu mengirimkannya langsung ke Action Server `NavigateThroughPoses` milik Nav2. Ini memungkinkan Nav2 menangani seluruh pemulihan (*recovery actions*), deteksi tabrakan dinamis, dan interpolasi kurva secara native.

---

## 5. PENJELASAN PARAMETER RINCI & DETAIL

### 5.1 Parameter Kunci AMCL (`nav2_params.yaml`)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `min_particles` | `500` | int | Jumlah partikel minimum saat estimasi posisi robot memiliki keyakinan tinggi. |
| `max_particles` | `3000` | int | Batas atas jumlah partikel saat robot mengalami ketidakpastian tinggi (misal setelah teleportasi/kehilangan jejak). |
| `update_min_d` | `0.1` | double (m) | Jarak translasi minimum yang harus ditempuh robot sebelum partikel di-resample. |
| `update_min_a` | `0.15` | double (rad) | Rotasi minimum ($\approx 8.6^\circ$) sebelum filter partikel di-update. |
| `alpha1` | `0.2` | double | Parameter derau odometri: variansi rotasi akibat rotasi. |
| `alpha2` | `0.2` | double | Parameter derau odometri: variansi rotasi akibat translasi. |
| `alpha3` | `0.2` | double | Parameter derau odometri: variansi translasi akibat translasi. |
| `alpha4` | `0.2` | double | Parameter derau odometri: variansi translasi akibat rotasi. |
| `laser_model_type` | `likelihood_field` | string | Model sensor pengukuran jarak: `likelihood_field` lebih cepat dan halus dibanding `beam`. |
| `z_hit` | `0.8` | double | Bobot komponen gaussian sensor model (kemungkinan sinar memantul dari objek nyata). |
| `z_rand` | `0.2` | double | Bobot komponen derau acak sensor (pantulan palsu / cermin). |

### 5.2 Parameter Kunci Regulated Pure Pursuit Controller

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `desired_linear_vel` | `0.5` | double (m/s) | Kecepatan jelajah nominal robot di lintasan lurus. |
| `lookahead_dist` | `0.4` | double (m) | Jarak titik pandang depan nominal ($L$). Nilai terlalu besar membuat robot memotong tikungan (*corner cutting*); nilai terlalu kecil membuat gerakan bergoyang (*wobbling*). |
| `min_lookahead_dist` | `0.25` | double (m) | Batas bawah jarak titik pandang depan pada kecepatan rendah. |
| `max_lookahead_dist` | `0.8` | double (m) | Batas atas jarak titik pandang depan pada kecepatan tinggi. |
| `use_velocity_scaled_lookahead_dist` | `true` | bool | Jika `true`, jarak pandang diatur proporsional terhadap kecepatan: $L = \text{clamp}(v \cdot t_{\text{lookahead}})$. |
| `lookahead_time` | `1.5` | double (s) | Waktu pandang proyeksi robot ke depan ($t_{\text{lookahead}}$). |
| `use_regulated_linear_velocity_scaling` | `true` | bool | Mengaktifkan perlambatan otomatis saat melewati kurva tajam. |
| `regulated_linear_scaling_min_radius` | `0.6` | double (m) | Radius tikungan minimum di mana kecepatan robot mulai diturunkan secara proporsional. |
| `use_rotate_to_heading` | `true` | bool | Memaksa robot roda diferensial menyelaraskan sudut hadapnya dengan target sebelum melaju maju. |
| `rotate_to_heading_min_angle` | `0.785` | double (rad) | Ambang selisih orientasi ($45^\circ$) yang memicu mode putar di tempat. |

### 5.3 Parameter Kunci Costmap (Global & Local)

| Nama Parameter | Nilai Default | Tipe Data | Penjelasan Matematis & Pengaruh Fisik |
| :--- | :---: | :---: | :--- |
| `robot_radius` | `0.22` | double (m) | Radius lingkaran batas luar fisik bodi robot TurtleBot. Digunakan untuk memeriksa tabrakan sel costmap. |
| `inflation_radius` | `0.45` | double (m) | Jarak perluasan biaya rintangan. Seluruh sel dalam radius ini memiliki biaya $> 0$. |
| `cost_scaling_factor` | `3.0` | double | Konstanta eksponensial pembusukan biaya $k$. Nilai lebih tinggi membuat biaya turun drastis menjauhi rintangan; nilai lebih rendah membuat rintangan terasa "lebih tebal". |
| `obstacle_range` | `2.5` | double (m) | Jarak sensor maksimum yang diizinkan untuk menandai sel sebagai occupied. |
| `raytrace_range` | `3.0` | double (m) | Jarak sensor maksimum yang digunakan untuk membersihkan rintangan bebas melalui raytracing. |

---

## 6. CARA MENJALANKAN FITUR NAV2 & COVERAGE

Jalankan satu perintah berikut di dalam container:

```bash
distrobox enter ros2-jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2 world:=turtlebot3_house.world
```

### Prosedur Operasi di RViz2:
1. RViz2 akan terbuka secara otomatis menampilkan peta `map_turtlehouse.yaml` dan partikel AMCL.
2. Klik tombol **"Publish Point"** di bilah atas RViz, lalu klik minimal 3–4 titik di area lantai ruangan untuk membentuk poligon batas area penyapuan.
3. Setelah titik terakhir diklik, buka terminal kedua dan picu proses komputasi:
   ```bash
   ros2 service call /finish_field_boundary std_srvs/srv/Trigger
   ```
4. Jalur coverage berwarna cyan akan seketika digenerate oleh Fields2Cover, dan Nav2 akan langsung memandu robot menyapu seluruh ruangan secara otonom!

---

## 7. REFERENSI ILMIAH & BACAAN WAJIB

1. **Paper Arsitektur Nav2**:
   - Macenski, S., Martín, F., White, R., & Clavero, J. (2020). *The Marathon 2: A Navigation System*. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS).
2. **Paper Regulated Pure Pursuit Controller**:
   - Macenski, S., Singh, S., Martin, F., & Ganganath, N. (2023). *Regulated Pure Pursuit for Robot Path Tracking*. Autonomous Robots.
3. **Paper Fields2Cover Complete Coverage**:
   - Mier, G., Valente, J., & de Bruin, S. (2023). *Fields2Cover: An open-source coverage path planning library for unmanned agricultural vehicles*. IEEE Robotics and Automation Letters (RA-L), 8(4), 2166-2172.
4. **Paper Dekomposisi Boustrophedon**:
   - Choset, H., & Pignon, P. (1998). *Coverage Path Planning: The Boustrophedon Cellular Decomposition*. International Conference on Field and Service Robotics.

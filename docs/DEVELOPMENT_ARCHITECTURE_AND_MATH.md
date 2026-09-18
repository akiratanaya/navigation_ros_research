# DOKUMENTASI LENGKAP: ARSITEKTUR SISTEM, PENGEMBANGAN DARI AWAL, DAN FONDASI MATEMATIS
## Integrasi Navigasi Otonom Nav2 & CMU Autonomy Stack dengan Fields2Cover pada Robot Roda Diferensial

---

## 1. PENDAHULUAN DAN LATAR BELAKANG

Sistem ini dirancang untuk menyelesaikan permasalahan **Complete Coverage Path Planning (CCPP)** dan navigasi otonom pada robot beroda diferensial (*differential drive mobile robot*) di dua domain operasional yang berbeda:
1. **Domain 2D Datar (Indoor/Structured Environment)**: Lingkungan rumah tinggal atau kantor lantai datar (*flat floor*), menggunakan representasi *2D Occupancy Grid Map*, algoritma lokalisasi partikel probabilistik (AMCL), dan framework navigasi **ROS 2 Nav2** dengan *Regulated Pure Pursuit Controller*.
2. **Domain 3D Kompleks (Unstructured/Terrain Environment)**: Lingkungan garasi atau semi-outdoor dengan tanjakan (*ramps*), rintangan 3 dimensi dengan elevasi bervariasi, serta lubang atau turunan curam (*negative obstacles / drop-offs*), di mana representasi peta 2D mengalami kegagalan (*false obstacles* pada tanjakan dan kebutaan terhadap lubang). Domain ini diatasi dengan integrasi **CMU Autonomy Stack** (*terrain_analysis* dan *local_planner*).

Sistem ini mengimplementasikan mekanisme **Dual-Backend Architecture**, di mana modul *Coverage Path Planning* (**Fields2Cover**) dapat mengeksekusi jalurnya ke backend Nav2 ataupun backend CMU tanpa modifikasi kode aplikasi tingkat atas.

---

## 2. SPESIFIKASI FISIK & PEMODELAN ROBOT (URDF & XACRO)

Robot dimodelkan menggunakan format URDF (*Unified Robot Description Format*) berbasis Xacro yang modular di paket `robot_description`.

```
                  ┌──────────────────────┐
                  │    3D LiDAR / 2D     │ (sensor_link / lidar_link)
                  └──────────┬───────────┘
                             │ z = 0.165 m
                  ┌──────────┴───────────┐
                  │   Top Plate + IMU    │
                  └──────────┬───────────┘
                             │ Standoffs (h = 0.108 m)
       ┌──────────┬──────────┴───────────┬──────────┐
       │ Roda     │      Base Chassis    │ Roda     │
       │ Kiri     │  (0.281 x 0.306 m)   │ Kanan    │
       └──────────┴──────────┬───────────┴──────────┘
                             │
                      Caster Wheels
                   (Front & Rear Ball)
```

### 2.1 Parameter Fisik Kunci
- **Radius Roda Penggerak ($r$)**: $0.033\text{ m}$ ($33\text{ mm}$)
- **Jarak Antar Roda / Track Gauge ($L$)**: $0.287\text{ m}$ ($287\text{ mm}$)
- **Dimensi Chassis Utama**: Panjang $0.281\text{ m}$, Lebar $0.306\text{ m}$, Tinggi $0.070\text{ m}$
- **Massa Chassis ($m_{\text{base}}$)**: $1.80\text{ kg}$
- **Massa Total Sistem**: $\approx 2.45\text{ kg}$
- **Lebar Efektif Robot ($W_{\text{robot}}$)**: $0.15\text{ m}$
- **Lebar Jalur Sapuan Pembersih/Coverage ($W_{\text{cov}}$)**: $0.24\text{ m}$
- **Radius Putar Minimum Kinematik ($R_{\text{min}}$)**: $0.05\text{ m}$ (dapat berputar di tempat $R=0$)

### 2.2 Konfigurasi Sensor Modular
URDF mendukung peralihan mode sensor melalui parameter `lidar_mode`:
- **`lidar_mode == "2d"`**: Mengaktifkan emulasi sensor LiDAR 2D planar (*sensor_msgs/msg/LaserScan*) pada topik `/scan` dengan 360 sampel horizontal, range $0.12\text{ m} - 12.0\text{ m}$, frekuensi update $10\text{ Hz}$.
- **`lidar_mode == "3d"`**: Mengaktifkan emulasi sensor LiDAR 3D 16-channel (*sensor_msgs/msg/PointCloud2*) pada topik `/points` menyerupai Velodyne VLP-16, dengan field of view vertikal $[-15^\circ, +15^\circ]$ dan horizontal $360^\circ$, menghasilkan $\sim 300.000$ titik per detik.

---

## 3. FONDASI MATEMATIS KINEMATIKA RODA DIFERENSIAL

Robot menggunakan konfigurasi penggerak diferensial dengan 2 roda traksi koaksial dan 2 roda *caster* pasif (depan dan belakang) untuk menjaga keseimbangan statis.

```
                  ^ Y_robot
                  │
                  │       [ v_x ]
                  │          ▲
                  │          │
        [Wheel L] ├─── L/2 ──┼─── L/2 ───┤ [Wheel R]
        (omega_L) │          │           │ (omega_R)
                  │          O ───────> X_robot
                  │
                  │   Rotasi yaw: omega_z
```

### 3.1 Kinematika Maju (Forward Kinematics)
Diberikan kecepatan sudut roda kanan $\omega_R = \dot{\theta}_R$ dan roda kiri $\omega_L = \dot{\theta}_L$ (dalam satuan $\text{rad/s}$), kecepatan linear tangensial masing-masing roda adalah:
$$v_R = r \cdot \omega_R$$
$$v_L = r \cdot \omega_L$$

Kecepatan linear robot pada sumbu longitudinal $v_x$ dan kecepatan sudut rotasi robot $\omega_z = \dot{\theta}$ terhadap titik pusat aksial $O$ didefinisikan sebagai:
$$\begin{bmatrix} v_x \\ \omega_z \end{bmatrix} = \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{r}{L} & -\frac{r}{L} \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

Karena robot merupakan sistem non-holonomik (*non-holonomic constraint*), robot tidak dapat bergerak menyamping seketika pada sumbu lateral:
$$v_y = 0$$

### 3.2 Kinematika Balik (Inverse Kinematics)
Ketika modul pengendali lintasan (*path follower* atau *controller*) mengeluarkan perintah kendali kecepatan $\mathbf{u} = [v_x, \omega_z]^T$, kecepatan putar roda dihitung melalui invers matriks kinematika:
$$\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix} = \begin{bmatrix} \frac{1}{r} & \frac{L}{2r} \\ \frac{1}{r} & -\frac{L}{2r} \end{bmatrix} \begin{bmatrix} v_x \\ \omega_z \end{bmatrix}$$

Secara eksplisit:
$$\omega_R = \frac{v_x + \frac{L}{2}\omega_z}{r}$$
$$\omega_L = \frac{v_x - \frac{L}{2}\omega_z}{r}$$

### 3.3 Integrasi Odometri Dead Reckoning
Posisi robot dalam frame referensi global $(\text{odom})$ pada selang waktu sampling $\Delta t = t_{k+1} - t_k$ dihitung menggunakan aproksimasi busur lingkaran (*exact integration of constant curvature*):

Jika $|\omega_z| > 10^{-5}\text{ rad/s}$:
$$\Delta \theta = \omega_z \cdot \Delta t$$
$$\Delta x = \frac{v_x}{\omega_z} \left( \sin(\theta_k + \Delta \theta) - \sin(\theta_k) \right)$$
$$\Delta y = -\frac{v_x}{\omega_z} \left( \cos(\theta_k + \Delta \theta) - \cos(\theta_k) \right)$$

Jika $\omega_z \approx 0$ (gerak lurus murni):
$$\Delta \theta = 0$$
$$\Delta x = v_x \Delta t \cos(\theta_k)$$
$$\Delta y = v_x \Delta t \sin(\theta_k)$$

Pembaruan state posisi global:
$$\mathbf{p}_{k+1} = \begin{bmatrix} x_{k+1} \\ y_{k+1} \\ \theta_{k+1} \end{bmatrix} = \begin{bmatrix} x_k + \Delta x \\ y_k + \Delta y \\ \theta_k + \Delta \theta \end{bmatrix}$$

---

## 4. INTEGRASI PIPELINE NAVIGASI 2D (NAV2)

Pada lingkungan datar, tumpukan navigasi standar **ROS 2 Nav2** dieksekusi dengan konfigurasi optimal pada `nav2_params.yaml`.

```
   /scan ──────> [ AMCL Particle Filter ] ───> TF (map -> odom)
                       │
   /map ───────> [ Global Costmap ] ───> [ Navfn Planner (A*) ]
                       │                         │
                       ▼                         ▼
   /scan ──────> [ Local Costmap ]  ───> [ Regulated Pure Pursuit ] ──> /cmd_vel
```

### 4.1 Lokalisasi: AMCL (Adaptive Monte Carlo Localization)
AMCL menggunakan filter partikel berbasis KLD-sampling (*Kullback-Leibler Divergence*) untuk mengestimasi distribusi probabilitas pose robot $p(x_t | z_{1:t}, u_{1:t})$.
1. **Model Pergerakan Probabilistik**:
   Menggunakan `nav2_amcl::DifferentialMotionModel` dengan 5 koefisien noise $\alpha_1, \alpha_2, \alpha_3, \alpha_4, \alpha_5 = 0.2$ untuk memodelkan slip putaran dan translasi.
2. **Model Pengukuran Sensor**:
   Menggunakan `likelihood_field` di mana setiap pancaran berkas LiDAR dievaluasi jarak terdekatnya ke rintangan pada peta grid:
   $$p(z_t^k | x_t, m) = z_{\text{hit}} \cdot \frac{1}{\sqrt{2\pi \sigma_{\text{hit}}^2}} \exp\left(-\frac{\text{dist}^2}{2\sigma_{\text{hit}}^2}\right) + \frac{z_{\text{rand}}}{z_{\text{max}}}$$
   dengan $z_{\text{hit}} = 0.5$, $\sigma_{\text{hit}} = 0.2\text{ m}$, $z_{\text{rand}} = 0.5$.

### 4.2 Pemodelan Costmap 2D Multi-Layer
Costmap merepresentasikan probabilitas ruang rintangan ke dalam grid diskret beresolusi $0.05\text{ m}$ ($5\text{ cm}$):
1. **Static Layer**: Memuat data peta biner dari `map_server`.
2. **Obstacle/Voxel Layer**: Melakukan *raycasting* berkas sensor LiDAR menggunakan algoritma garis Bresenham 2D/3D untuk membersihkan ruang bebas (*clearing*) dan menandai rintangan (*marking*).
3. **Inflation Layer**: Memperluas rintangan dengan fungsi peluruhan eksponensial matematis:
   $$\text{cost}(d) = \begin{cases} 254 & \text{jika } d \le r_{\text{inscribed}} \\ 253 \cdot \exp\left(-\alpha \cdot (d - r_{\text{inscribed}})\right) & \text{jika } r_{\text{inscribed}} < d \le r_{\text{inflation}} \\ 0 & \text{jika } d > r_{\text{inflation}} \end{cases}$$
   Dalam sistem kita: $r_{\text{inscribed}} = 0.155\text{ m}$, $r_{\text{inflation}} = 0.18\text{ m}$, dan faktor skala biaya $\alpha = 25.0$.

### 4.3 Pelacak Jalur: Regulated Pure Pursuit Controller (RPP)
RPP bertanggung jawab mengendalikan robot agar melacak rute secara presisi dengan mematuhi dinamika gerak.

```
                           Target Waypoint P (x_t, y_t)
                                 ●
                                /│
                               / │
                          L_d /  │ y_t
                             /   │
                            / α  │
            Robot Pose O ──┴─────┴─────────> Heading
                           x_t
            Lingkaran Busur dengan Radius R
```

1. **Geometri Pure Pursuit**:
   Jarak pandang ke depan (*lookahead distance*) dinamis:
   $$L_d = \text{clamp}(v_x \cdot t_{\text{lookahead}}, L_{\text{min}}, L_{\text{max}})$$
   dengan $t_{\text{lookahead}} = 0.6\text{ s}$, $L_{\text{min}} = 0.18\text{ m}$, $L_{\text{max}} = 0.28\text{ m}$.
   
   Sudut relatif ke titik target adalah $\alpha = \text{atan2}(y_t, x_t)$.
   Kelengkungan lintasan busur lingkaran ($\kappa$) adalah:
   $$\kappa = \frac{2 \sin \alpha}{L_d}$$
   Radius kelengkungan: $R = \frac{1}{\kappa}$.

2. **Regulasi Kecepatan Adaptif (Curvature Regulation)**:
   Kecepatan linear diperlambat secara otomatis saat berbelok tajam untuk mencegah gaya sentrifugal berlebih dan selip roda:
   $$v_{\text{regulated}} = v_{\text{des}} \cdot \min\left(1.0, \frac{|R|}{R_{\text{min\_scale}}}\right)$$
   dengan $v_{\text{des}} = 0.30\text{ m/s}$ dan $R_{\text{min\_scale}} = 0.15\text{ m}$.

3. **Perintah Kontrol**:
   $$v_x = v_{\text{regulated}}$$
   $$\omega_z = \kappa \cdot v_x = \frac{2 v_x \sin \alpha}{L_d}$$

4. **Rotate-to-Heading Logic**:
   Jika selisih sudut heading robot dengan arah rute $|\alpha| > 45^\circ$ ($0.785\text{ rad}$), robot berhenti bergerak linear ($v_x = 0$) dan berputar murni di tempat dengan kecepatan $\omega_z = 0.85\text{ rad/s}$ hingga robot menghadap jalur.

---

## 5. ALGORITMA COVERAGE PATH PLANNING (FIELDS2COVER)

Modul coverage diimplementasikan pada paket `robot_coverage` menggunakan pustaka **Fields2Cover** yang diperkaya dengan filter rintangan adaptif dan strategi 2-fase komersial.

```
  [ /field_boundary ] ────────┐
                              ▼
  [ /map ] ──────> [ Ekstraksi Rintangan Interior (CV2) ]
                              │
                              ▼
               [ Fields2Cover Cells Decomposition ]
                              │
               ┌──────────────┴──────────────┐
               ▼                             ▼
      [ Fase 1: Perimeter Tour ]    [ Fase 2: Infill Boustrophedon ]
               │                             │
               └──────────────┬──────────────┘
                              ▼
               [ Smooth Obstacle Arc Bypass ]
                              │
               [ Filter Obstacle Costmap ]
                              │
                              ▼
             Topik /coverage_path (nav_msgs/Path)
```

### 5.1 Ekstraksi Rintangan Interior Berbasis Morfologi
Peta rintangan grid dari `/map` diekstraksi secara otomatis untuk mengidentifikasi tiang rintangan interior (seperti kaki meja) yang berada di dalam area poligon boundary:
1. **Masking Poligon**: Mengonversi poligon koordinat dunia $(X, Y)$ ke koordinat piksel $(u, v)$ menggunakan resolusi grid $r = 0.05\text{ m/px}$ dan origin $(x_0, y_0)$:
   $$u = \left\lfloor \frac{X - x_0}{r} \right\rfloor, \quad v = \left\lfloor \frac{Y - y_0}{r} \right\rfloor$$
2. **Erosi Dinding Luar**: Operasi erosi morfologi dengan kernel persegi ukuran $k_{\text{erode}} = 2 \cdot \lceil \frac{0.15}{r} \rceil + 1$ untuk memisahkan dinding batas ruangan dari rintangan tengah.
3. **Dilasi Rintangan**: Operasi dilasi dengan kernel elips untuk merekatkan kluster piksel satu tiang.
4. **Convex Hull & Minimum Area Bounding Box**: Menghitung titik pusat $(c_x, c_y)$ dan jari-jari aman:
   $$R_{\text{safe}} = \max\left(0.18\text{ m}, R_{\text{raw}} + 0.12\text{ m}\right)$$

### 5.2 Strategi Coverage 2-Fase Komersial
1. **Fase 1 (Perimeter Tour)**:
   Robot pertama kali menyapu keliling tepi terluar batas poligon sebanyak 1 putaran penuh ($360^\circ$). Hal ini menjamin pembersihan sudut dan sisi ruangan secara bersih tuntas.
   - Pembangkitan headland swath: $HG\_Const\_gen$ dengan offset $d = W_{\text{cov}} = 0.24\text{ m}$.
   - Pemilihan titik start perimeter ditentukan oleh jarak Euclidean terdekat dari pose aktual robot saat ini:
     $$i_{\text{start}} = \arg\min_i \sqrt{(x_i - x_{\text{robot}})^2 + (y_i - y_{\text{robot}})^2}$$
2. **Fase 2 (Infill Sweeping Boustrophedon)**:
   Area interior yang tersisa diisi dengan barisan garis sapuan paralel (*swaths*) teratur.
   - Pembangkitan swaths: $SG\_BruteForce$ pada sudut sapuan optimal $\theta_{\text{swath}} = 90^\circ$ ($\frac{\pi}{2}\text{ rad}$).
   - Jarak antar swath (*swath width*): $W_{\text{cov}} = 0.24\text{ m}$.
   - Pengurutan rute: $RP\_Boustrophedon$ membentuk pola bolak-balik (serpentine) yang meminimalkan total jarak putar balik non-kerja (*turning cost*).

### 5.3 Matematika Smooth Circular Obstacle Arc Bypass
Jika garis sapuan infill memotong lingkaran proteksi rintangan $(c_x, c_y, R_{\text{safe}})$, titik-titik lintasan diproyeksikan secara kontinu melingkari rintangan sehingga jalur tetap mulus tanpa terpotong diagonal:

Untuk setiap titik $\mathbf{p} = [p_x, p_y]^T$ pada garis lurus dengan vektor arah satuan $\mathbf{u} = [u_x, u_y]^T$ dan vektor normal $\mathbf{n} = [-u_y, u_x]^T$:
1. Vektor relatif terhadap pusat rintangan: $\mathbf{v} = \mathbf{p} - \mathbf{c}$.
2. Jarak proyeksi sepanjang garis: $s = \mathbf{v} \cdot \mathbf{u}$.
3. Jarak tegak lurus ke garis: $d_{\perp} = \mathbf{v} \cdot \mathbf{n}$.
4. Jika $|s| < R_{\text{safe}}$ dan $|d_{\perp}| < \sqrt{R_{\text{safe}}^2 - s^2}$:
   $$r_{\text{perp}} = \sqrt{R_{\text{safe}}^2 - s^2}$$
   $$\mathbf{p}_{\text{bypass}} = \mathbf{p} + \text{sgn}(d_{\perp}) \cdot (r_{\text{perp}} - |d_{\perp}|) \cdot \mathbf{n}$$
Formula ini menghasilkan busur sirkular tangensial yang menjamin robot menyapu sedekat mungkin dengan rintangan tanpa risiko tabrakan.

---

## 6. ARSITEKTUR CMU AUTONOMY STACK & FONDASI MATEMATIS 3D

Ketika robot berpindah ke lingkungan 3D (seperti `garage.world` yang memiliki tanjakan, elevasi bertingkat, dan obstacle 3D), pendekatan 2D costmap runtuh. CMU Autonomy Stack memproses PointCloud 3D secara langsung melalui geometri diferensial medan.

```
   Gazebo /points (lidar_link) ───┐
   Gazebo /odom               ───┼─> [ cmu_sim_bridge ]
                                 │          │
   /state_estimation <───────────┘          ▼
   /registered_scan  <──────────────────────┘
         │
         ├──────────────────────────────┐
         ▼                              ▼
   [ terrain_analysis ]        [ sensor_scan_generation ]
         │                              │
         ▼ /terrain_map                 │
   [ terrain_analysis_ext ]             │
         │                              │
         └──────────────┬───────────────┘
                        ▼
   /way_point ───> [ local_planner ] (343 Trajectory Lattice)
                        │
                        ▼ /path
                 [ pathFollower ] (Pure Pursuit & Diff-Drive Twist)
                        │
                        ▼
                     /cmd_vel
```

### 6.1 Modul Jembatan: `cmu_sim_bridge`
CMU Stack mensyaratkan dua topik utama dalam koordinat dunia (`map`):
1. `/state_estimation` (`nav_msgs/msg/Odometry`): Posisi dan kecepatan robot dalam frame `map` dengan `child_frame_id = 'vehicle'`.
2. `/registered_scan` (`sensor_msgs/msg/PointCloud2`): PointCloud LiDAR 3D yang telah ditransformasikan ke frame `map` lengkap dengan koordinat $(x, y, z)$ dan intensitas.

#### Transformasi Koordinat Homogen PointCloud 3D:
Diberikan titik LiDAR pada frame sensor $\mathbf{p}_s = [x_s, y_s, z_s]^T$ dan pose sensor relatif terhadap map $[\mathbf{t}, \mathbf{q}]$ di mana $\mathbf{q} = [q_x, q_y, q_z, q_w]^T$:

Matriks Rotasi $R(\mathbf{q}) \in SO(3)$ dihitung sebagai:
$$R = \begin{bmatrix} 1 - 2(q_y^2 + q_z^2) & 2(q_x q_y - q_z q_w) & 2(q_x q_z + q_y q_w) \\ 2(q_x q_y + q_z q_w) & 1 - 2(q_x^2 + q_z^2) & 2(q_y q_z - q_x q_w) \\ 2(q_x q_z - q_y q_w) & 2(q_y q_z + q_x q_w) & 1 - 2(q_x^2 + q_y^2) \end{bmatrix}$$

Transformasi affine vectorized ke koordinat dunia (`map`):
$$\mathbf{p}_{\text{map}} = R \cdot \mathbf{p}_s + \mathbf{t}$$

Pesan PointCloud yang dipublikasikan memiliki field `[x, y, z, intensity]` bertipe `FLOAT32`.

### 6.2 Analisis Medan 3D: `terrain_analysis`
Modul ini bertugas mengekstraksi bidang permukaan tanah (*ground estimation*) dan mengukur deviasi elevasi rintangan.
1. **Struktur Data Voxel Bertingkat**:
   - **Terrain Voxel Ring**: Grid horizontal berukuran $21 \times 21$ voxel (lebar voxel $1.0\text{ m}$) yang selalu berpusat mengikuti pergerakan posisi robot saat ini $(x_{\text{vehicle}}, y_{\text{vehicle}})$.
   - **Planar Local Voxel**: Grid planar lokal berukuran $51 \times 51$ sub-voxel (resolusi tinggi $0.2\text{ m}$).
2. **Estimasi Elevasi Permukaan Tanah (Quantile Ground Level)**:
   Pada setiap sub-voxel planar $V(i, j)$, kumpulan elevasi titik $\{z_k\}$ diurutkan dari terendah ke tertinggi.
   Ketinggian tanah lokal $Z_{\text{ground}}$ diambil pada persentil quantile $Q = 0.25$ ($25\%$ terendah):
   $$Z_{\text{ground}} = z_{(\lfloor 0.25 \cdot N \rfloor)}$$
   Pendekatan persentil ini sangat kokoh (*robust*) terhadap noise sensor dan rumput/dedaunan tipis.
3. **Ketinggian Relatif Rintangan (Displacement Height)**:
   Untuk setiap titik laser $\mathbf{p}$:
   $$\Delta Z = z - Z_{\text{ground}}$$
4. **Deteksi Rintangan Positif vs Negatif (Negative Obstacle / Lubang)**:
   - **Rintangan Positif (Batu, Dinding, Rintangan Tinggi)**:
     $$\Delta Z > h_{\text{obstacle\_thre}} \quad (0.2\text{ m})$$
   - **Rintangan Negatif (Lubang / Jurang / Drop-off)**:
     Jika pada jarak horizontal $d_{xy} < d_{\text{neg\_dis}}$ ditemukan titik dengan penurunan curam:
     $$\Delta Z < h_{\text{neg\_thre}} \quad (-0.2\text{ m})$$
     Maka nilai ketinggian relatif titik tersebut dipaksa bernilai maksimum $\Delta Z = H_{\text{vehicle}} = 1.5\text{ m}$. Dengan cara ini, lubang diperlakukan setara dengan dinding vertikal tebal, sehingga perencana lintasan tidak akan pernah memilih jalur yang melintasi lubang!
5. **Output**: Peta titik medan yang terdisinfeksi dipublikasikan ke topik `/terrain_map`, di mana nilai intensitas (*intensity*) setiap titik menyimpan nilai $\Delta Z$.

### 6.3 Evaluasi Lintasan: `local_planner`
`localPlanner` tidak menggunakan optimasi numerik kontinu yang lambat, melainkan menggunakan **Precomputed Motion Primitive Lattice** yang terdiri dari $343$ lintasan kandidat yang terbagi ke dalam $7$ grup arah gerak (*directional groups*).

1. **Pemeriksaan Tabrakan Voxel**:
   Setiap lintasan kandidat $P_k$ dipetakan ke dalam tabel korespondensi sel volume ruang kerja (*workspace boundary envelope*). Jika terdapat titik pada `/terrain_map` yang jatuh di dalam sel lintasan dengan intensitas $\Delta Z > h_{\text{thre}}$, lintasan tersebut langsung dicoret (*blocked*).
2. **Fungsi Biaya Multi-Objektif (Trajectory Cost Function)**:
   Untuk seluruh lintasan yang bebas tabrakan, dievaluasi skor penalti total:
   $$J(P_k) = w_{\text{goal}} \cdot D_{\text{goal}}(P_k) + w_{\text{dir}} \cdot |\Delta \theta_k| + w_{\text{roughness}} \cdot \text{Cost}_{\text{terrain}}(P_k)$$
   di mana:
   - $D_{\text{goal}}(P_k)$: Jarak Euclidean dari ujung lintasan $P_k$ ke titik target `/way_point`.
   - $\Delta \theta_k$: Deviasi orientasi heading akhir lintasan terhadap arah vektor sasaran.
   - $\text{Cost}_{\text{terrain}}$: Akumulasi kekasaran medan (*terrain penalty*) sepanjang lintasan.
3. Lintasan dengan skor penalti $J$ terkecil dipilih dan dipublikasikan ke topik `/path`.

### 6.4 Modifikasi Pengendali Lintasan (`pathFollower`) untuk Roda Diferensial
Secara *default*, CMU Autonomy Stack dirancang untuk kendaraan omnidirectional atau mobil Ackerman yang dapat menghasilkan kecepatan lateral $v_y$. Untuk robot roda diferensial, kita telah memodifikasi implementasi logika di `pathFollower.cpp`:

```cpp
// Konfigurasi omniDirGoalThre = -1.0 mengunci robot dalam mode unicycle/differential
if (omniDirGoalThre > 0) {
    cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
    cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
} else {
    // Mode Roda Diferensial Murni: Kecepatan lateral terkunci 0
    cmd_vel.twist.linear.x = vehicleSpeed;
    cmd_vel.twist.linear.y = 0.0;
}
cmd_vel.twist.angular.z = vehicleYawRate;

// Publikasi Twist langsung ke Gazebo
pubSpeedTwist->publish(cmd_vel.twist);
```

### 6.5 Pengatur Sekuensial Waypoint: `cmu_coverage_navigator`
Node `cmu_coverage_navigator.py` bertindak sebagai jembatan antara perencana cakupan area tingkat tinggi (Fields2Cover) dengan pengendali lokal CMU:
1. Menerima array waypoint dari topik `/coverage_path` ($N$ buah waypoint).
2. Memublikasikan waypoint aktif ke-k ke topik `/way_point` (`geometry_msgs/PointStamped`).
3. Pada loop kontrol $10\text{ Hz}$, membaca posisi aktual dari `/state_estimation` $(x_{\text{rob}}, y_{\text{rob}})$ dan menghitung jarak Euclidean ke waypoint aktif target $(x_k^*, y_k^*)$:
   $$d_k = \sqrt{(x_{\text{rob}} - x_k^*)^2 + (y_{\text{rob}} - y_k^*)^2}$$
4. Jika $d_k \le \epsilon_{\text{tol}}$ (default $\epsilon_{\text{tol}} = 0.35\text{ m}$):
   $$k \leftarrow k + 1$$
   Node mengirimkan waypoint berikutnya ke CMU Stack.
5. Saat $k = N$, node memublikasikan sinyal henti `/stop` (`std_msgs/Int8 = 1`).

---

## 7. ARSITEKTUR & FORMULASI MATEMATIS CMU TARE PLANNER (3D AUTONOMOUS EXPLORATION)

Untuk mode penjelajahan otonom tanpa peta awal (*zero prior map*), sistem menggunakan algoritma **TARE (Terrain-Aware Autonomous Exploration) Planner** yang dikembangkan oleh tim Carnegie Mellon University (CMU). Algoritma ini dirancang khusus untuk menjelajahi lingkungan 3D yang kompleks dan luas dengan efisiensi tinggi melalui dekomposisi hierarkis dan optimasi jalur ganda (*dual-layer Traveling Salesman Problem*).

```
   [3D LiDAR /points] ──► [cmu_sim_bridge] ──► [/registered_scan] ──► [terrain_analysis]
                                                                             │
                                                                       [/terrain_map]
                                                                             │
                                                                             ▼
┌─────────────────────────────────── TARE PLANNER NODE ───────────────────────────────┐
│                                                                                     │
│  1. Rolling Occupancy Grid (Voxel Update & Ray Casting)                             │
│     P(Voxel = Occupied | z_t) & P(Voxel = Unknown)                                  │
│                                │                                                    │
│  2. Deteksi Frontier 3D & Sampling Sudut Pandang (Viewpoint Generator)              │
│     {v_i} ϵ V_cand dihitung berdasarkan cakupan visibilitas sensor & jarak aman     │
│                                │                                                    │
│  3. Formulasi Hierarkis Dual TSP (Google OR-Tools):                                 │
│     - Global Subspace TSP: Menentukan urutan penyapuan blok ruang (Grid World)      │
│     - Local Coverage TSP: Menentukan tur lintasan lokal di sekitar robot            │
│                                │                                                    │
│  4. Optimal Waypoint Generator:                                                     │
│     x_target* = argmin Cost(v_i, x_robot) ──► Publish ke [/way_point]                │
└──────────────────────────────────────────────────┬──────────────────────────────────┘
                                                   │
                                                   ▼
                       [CMU local_planner (Differential Drive: v_y = 0)]
                                                   │
                                                   ▼
                                         [/cmd_vel] ──► [Gazebo]
```

### 7.1 Representasi Voxel: Rolling Occupancy Grid & Ray-Tracing
Ruang 3D di sekitar robot direpresentasikan dalam bentuk kisi okupansi berjalan (*rolling occupancy grid*) berukuran dinamis dengan resolusi sel $\delta = 0.2\text{ m}$. Setiap voxel $V_{i,j,k}$ menyimpan status:
$$\text{State}(V) \in \{\text{Unknown}, \text{Free}, \text{Occupied}\}$$

Ketika pointcloud sensor $P = \{p_m\}_{m=1}^M$ diterima:
1. **Ray Casting**: Untuk setiap titik pantulan $p_m = (x_m, y_m, z_m)$, ditarik garis lurus dari pusat sensor $s = (x_s, y_s, z_s)$ menuju $p_m$.
2. Seluruh voxel yang dilalui berkas sinar dinyatakan berstatus **Free**:
   $$\forall V \in \text{Ray}(s, p_m), \quad \text{State}(V) \leftarrow \text{Free}$$
3. Voxel tempat titik pantulan $p_m$ berada dinyatakan berstatus **Occupied**:
   $$\text{State}(V(p_m)) \leftarrow \text{Occupied}$$

### 7.2 Deteksi Perbatasan Frontier 3D
Voxel frontier $F$ didefinisikan sebagai sel **Free** yang bertetangga langsung dengan setidaknya satu sel **Unknown**:
$$\mathcal{F} = \{V \in \text{Grid} \mid \text{State}(V) = \text{Free} \land \exists U \in \mathcal{N}_{26}(V) \text{ s.t. } \text{State}(U) = \text{Unknown}\}$$

Frontier yang berdekatan dikelompokkan (*clustering*) menggunakan *Euclidean distance clustering* dengan radius toleransi $r_{\text{cluster}} = 1.0\text{ m}$ dan batas ukuran minimum $N_{\text{min}} = 8$ titik guna mengeliminasi derau sensor (*sensor noise*).

### 7.3 Pembangkitan Kandidat Sudut Pandang (*Viewpoint Sampling*)
Di sekitar setiap klaster frontier dan permukaan objek yang belum terjamah, algoritma membangkitkan himpunan titik sudut pandang $\mathcal{V} = \{v_1, v_2, \dots, v_n\}$ pada ketinggian robot terhadap medan tanah:
$$z(v_i) = z_{\text{terrain}}(x_i, y_i) + h_{\text{sensor}}$$

Setiap kandidat $v_i$ dievaluasi kelayakannya:
1. **Pemeriksaan Tabrakan Medan (*Terrain Collision Check*)**:
   Jarak minimum ke voxel *Occupied* harus memenuhi margin keselamatan robot beroda diferensial:
   $$\min_{p \in \text{Occupied}} \|v_i - p\| \ge R_{\text{margin}} \quad (R_{\text{margin}} = 0.35\text{ m})$$
2. **Konektivitas Graf (*Keypose Graph*)**:
   Titik $v_i$ harus dapat dihubungkan ke graf pergerakan yang telah dilalui robot tanpa terhalang rintangan.

### 7.4 Formulasi Dual-Layer TSP dengan Google OR-Tools
Kelemahan perencana eksplorasi konvensional adalah komputasi TSP skala penuh yang berbiaya eksponensial $\mathcal{O}(2^n \cdot n^2)$ saat jumlah sudut pandang $n$ mencapai ratusan titik. TARE memecahkan persoalan ini dengan pendekatan **Hierarkis Dua Tingkat (Dual-Layer)**:

#### Tingkat 1: Global Subspace TSP
Ruang eksplorasi global dibagi menjadi kubus-kubus ruang penjelajahan (*exploring subspaces* atau *cells*) berukuran $L_{\text{subspace}} = 18\text{ m} \times 18\text{ m} \times 1.8\text{ m}$.
Misalkan terdapat $K$ sel aktif yang masih memiliki frontier belum tercover. Rute kunjungan antar-sel dioptimasi sebagai Travelling Salesman Problem global:
$$\min_{\pi \in \mathcal{S}_K} \sum_{k=1}^{K-1} D_{\text{global}}(C_{\pi(k)}, C_{\pi(k+1)})$$
di mana $D_{\text{global}}$ adalah jarak terpendek melintasi *Keypose Graph*.

#### Tingkat 2: Local Coverage TSP
Di dalam sel aktif yang sedang dikunjungi robot saat ini (*Local Planning Horizon*, radius $4.0\text{ m}$), terdapat $m$ titik kandidat sudut pandang aktif $\{v_1, \dots, v_m\}$.
Rute lokal dioptimasi secara instan menggunakan solver **Google OR-Tools Routing Library** (dipanggil melalui binding C++ native `ortools::ortools`):
$$\min_{\sigma \in \mathcal{S}_m} \sum_{j=1}^{m-1} \text{Cost}(v_{\sigma(j)}, v_{\sigma(j+1)})$$
dengan fungsi bobot perpindahan yang memperhitungkan:
$$\text{Cost}(v_a, v_b) = d_{\text{geodesic}}(v_a, v_b) + \lambda_{\text{heading}} \cdot \Delta \theta(v_a, v_b)$$
di mana penalti perubahan sudut hadap $\lambda_{\text{heading}}$ memprioritaskan lintasan lurus mulus untuk kestabilan roda diferensial.

Target waypoint terdepan dari rute optimal dipublikasikan langsung ke `/way_point`, yang kemudian dieksekusi secara dinamis oleh `local_planner` CMU tanpa selip lateral ($v_y = 0$).

---

## 8. ARSITEKTUR INTEGRASI TERPADU & PERBANDINGAN KOMPARATIF

Sistem navigasi terpadu ini menyediakan 3 mode eksekusi mandiri:
1. **Nav2 Coverage (2D)**: `ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=nav2`
2. **CMU Fields2Cover (3D)**: `ros2 launch robot_bringup sim_coverage.launch.py nav_backend:=cmu`
3. **CMU TARE Exploration (3D Otonom 1-Klik)**: `ros2 launch robot_bringup sim_exploration.launch.py world:=garage.world`

### Perbandingan Komparatif Tiga Pilar Navigasi

| Fitur / Karakteristik | Nav2 Coverage (2D) | CMU Coverage (3D) | CMU TARE Exploration (3D) |
| :--- | :--- | :--- | :--- |
| **Kebutuhan Peta Awal** | Wajib (`map.yaml`) | Wajib poligon batas area | **Tidak Perlu (Zero Prior Map)** |
| **Model Representasi** | 2D Occupancy Grid | 3D Voxel Ring + Terrain Map | 3D Rolling Occupancy Grid |
| **Sensor Utama** | 2D LiDAR (`/scan`) | 3D LiDAR (`/points`) | 3D LiDAR (`/points` 16-ch) |
| **Perencana Global** | Fields2Cover Boustrophedon | Fields2Cover Boustrophedon | **Dual-Layer TSP (Google OR-Tools)** |
| **Pengendali Lokal** | Regulated Pure Pursuit | CMU Motion Lattice (343 roll) | CMU Motion Lattice ($v_y = 0$) |
| **Kemampuan Tanjakan/Medan** | Terbatas pada lantai datar | Medan kontur 3D non-ekstrem | **Medan kontur 3D kompleks** |
| **Deteksi Area Belum Terjamah** | Statis (berdasarkan poligon) | Statis (berdasarkan poligon) | **Dinamis Real-time (Frontiers)** |
| **Tujuan Operasional** | Pembersihan/inspeksi teratur | Penyemprotan/sapuan lereng 3D | **Pencarian, pemetaan & penjelajahan** |

---

## 9. KESIMPULAN

Melalui integrasi arsitektur komprehensif ini:
1. Robot roda diferensial memiliki kemampuan navigasi cakupan penuh (*complete coverage*) menggunakan matematika algoritma **Fields2Cover** yang diperkaya perlindungan rintangan internal.
2. Di lingkungan indoor datar, robot memanfaatkan efisiensi dan kestabilan lokalisasi global **Nav2 AMCL**.
3. Di lingkungan 3D berkontur tajam, robot memanfaatkan **CMU Autonomy Stack** yang mengevaluasi kemiringan lereng kuantil, mendeteksi jurang/lubang secara geometris, dan memilih lintasan kisi bebas rintangan tanpa risiko terguling atau jatuh.
4. Dalam skenario eksplorasi area tak dikenal (*unmapped unknown environments*), integrasi **CMU TARE Planner** memungkinkan robot secara mandiri dan cerdas menjelajahi 100% volume ruangan secara otonom tanpa intervensi operator manusia.


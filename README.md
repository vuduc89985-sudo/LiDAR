# 🛰️ LiDAR Master Viewer - Fusion 4 LiDAR (WEB) v3

Hệ thống xem và quản lý dữ liệu LiDAR thời gian thực qua trình duyệt web, hỗ trợ tối đa **16 LiDAR** cùng lúc với các tính năng nâng cao: fusion point cloud, zone đa hình, calibration, tracking, đo latency, và cấu hình IP động.

![Version](https://img.shields.io/badge/version-3.1-blue)
![LiDAR](https://img.shields.io/badge/LiDAR-TR60%2FTR70-green)
![Platform](https://img.shields.io/badge/platform-Windows%2010%2F11-orange)

---

## 📋 Mục lục

1. [Tổng quan](#1-tổng-quan)
2. [Yêu cầu hệ thống](#2-yêu-cầu-hệ-thống)
3. [Cài đặt](#3-cài-đặt)
4. [Chạy chương trình](#4-chạy-chương-trình)
5. [Cấu hình mạng cho LiDAR](#5-cấu-hình-mạng-cho-lidar)
6. [Hướng dẫn đổi IP LiDAR](#6-hướng-dẫn-đổi-ip-lidar)
7. [Test đổi IP an toàn](#7-test-đổi-ip-an-toàn)
8. [Cấu hình mạng cho nhiều LiDAR](#8-cấu-hình-mạng-cho-nhiều-lidar)
9. [Calibration (Hiệu chỉnh ngoại tham)](#9-calibration-hiệu-chỉnh-ngoại-tham)
10. [Xem từ máy khác trong mạng LAN](#10-xem-từ-máy-khác-trong-mạng-lan)
11. [Build từ source code](#11-build-từ-source-code)
12. [Phím tắt](#12-phím-tắt)
13. [Giao thức WebSocket](#13-giao-thức-websocket)
14. [Troubleshooting](#14-troubleshooting)
15. [Tài liệu tham khảo](#15-tài-liệu-tham-khảo)

---

## 1. Tổng quan

### Kiến trúc hệ thống

```
┌─────────────────┐         ┌──────────────────────┐         ┌─────────────┐
│   LiDAR 1-16    │◄──TCP──►│  Backend C++         │◄──WS───►│  Frontend   │
│   (TR60/TR70)   │  1112   │  (lidar_web_gateway) │  9001   │  (Web UI)   │
└─────────────────┘         └──────────────────────┘         └─────────────┘
                                    │                              │
                                    ▼                              ▼
                             lidar_config.cfg              http://host:8080
                             extrinsics.cfg
```

### Tính năng chính

- ✅ **Hỗ trợ 1-16 LiDAR** cùng lúc (cấu hình động)
- ✅ **Fusion point cloud** - ghép nhiều LiDAR thành 1 bản đồ thống nhất
- ✅ **Zone đa hình**: Rectangle, Polygon, Circle, Sector, Freehand
- ✅ **Sửa đỉnh zone** (vertex editing) - kéo thả để tinh chỉnh
- ✅ **Cấu hình IP động** - đổi IP/port/tên LiDAR ngay trên web
- ✅ **Calibration**: Pick 2 điểm, ICP Refine, RMSE@góc 0°
- ✅ **Đo latency**: RTT, E2E, GOI4320 (bù lệch đồng hồ)
- ✅ **Motion detection** + clustering + occlusion detection
- ✅ **Ghi CSV** dữ liệu thô
- ✅ **Ping ICMP** kiểm tra kết nối từng LiDAR
- ✅ **Dark theme** UI, responsive

---

## 2. Yêu cầu hệ thống

### Phần cứng
- **CPU**: Intel Core i3 trở lên (khuyến nghị i5+)
- **RAM**: 4 GB trở lên (khuyến nghị 8 GB cho 4+ LiDAR)
- **Card mạng**: Ethernet Gigabit (cho 4+ LiDAR)
- **Ổ cứng**: 500 MB trống (cho log + CSV recording)

### Phần mềm
- **OS**: Windows 10 / 11 (64-bit)
- **Visual C++ Redistributable 2015-2022 x64** (bắt buộc)
- **Python 3.x** (chỉ cần khi chạy web server qua Python)
- **Visual Studio 2022** (chỉ cần khi build lại từ source)
- **Trình duyệt**: Chrome / Edge / Firefox (khuyến nghị Chrome)

---

## 3. Cài đặt

### 3.1. Cài đặt Visual C++ Redistributable (BẮT BUỘC)

```
Double-click: tools\vc_redist.x64.exe
→ Install → Restart máy (nếu được yêu cầu)
```

### 3.2. Cài đặt Python 3 (TÙY CHỌN)

Chỉ cần nếu muốn mở web qua `localhost:8080` cho nhiều máy truy cập.

1. Tải từ [python.org](https://www.python.org/downloads/) hoặc Microsoft Store
2. **QUAN TRỌNG**: Tích vào ô **"Add Python to PATH"** khi cài
3. Kiểm tra: mở CMD → gõ `python --version` → phải hiện version

> 💡 **Mẹo**: Nếu chỉ xem trên chính máy chạy backend, không cần Python. Chỉ cần double-click `frontend\index.html`.

### 3.3. Cài đặt Visual Studio 2022 (CHỈ KHI CẦN BUILD)

Chỉ cần nếu muốn sửa code C++ và build lại backend.

1. Tải [Visual Studio 2022 Community](https://visualstudio.microsoft.com/)
2. Khi cài, chọn workload: **"Desktop development with C++"**
3. Đảm bảo có: MSVC v143, Windows 10/11 SDK, C++ CMake tools

---

## 4. Chạy chương trình

### 4.1. Khởi động (mỗi lần chạy)

```
Double-click: start_all.bat
```

→ Sẽ mở:
- 2 cửa sổ console đen (backend + web server)
- Chrome tự động mở tại `http://localhost:8080`

### 4.2. Kiểm tra hoạt động

- Web hiện **"Online"** (màu xanh) ở góc trái trên
- Thấy điểm LiDAR (point cloud) trên canvas
- Trạng thái 4 LiDAR: **● Connected** (màu xanh)

### 4.3. Tắt hệ thống

- Đóng 2 cửa sổ console đen
- Hoặc nhấn `q` + Enter trong cửa sổ backend

---

## 5. Cấu hình mạng cho LiDAR

### 5.1. Nguyên tắc cơ bản

| Thành phần | IP mặc định | Port | Ghi chú |
|---|---|---|---|
| LiDAR 1 | `192.168.201.15` | 1112 | Base data port |
| LiDAR 2 | `192.168.201.16` | 1112 | |
| LiDAR 3 | `192.168.201.17` | 1112 | |
| LiDAR 4 | `192.168.201.18` | 1112 | |
| PC (máy điều khiển) | `192.168.201.100` | - | Phải cùng subnet |

### 5.2. Cấu hình IP cho PC (Windows)

1. Mở **Control Panel** → **Network and Internet** → **Network Connections**
2. Chuột phải vào card mạng **Ethernet** → **Properties**
3. Chọn **Internet Protocol Version 4 (TCP/IPv4)** → **Properties**
4. Chọn **"Use the following IP address"**:
   ```
   IP address:        192.168.201.100
   Subnet mask:       255.255.255.0
   Default gateway:   (để trống)
   ```
5. Bấm **OK**

### 5.3. Cấu hình IP cho LiDAR (qua web hãng)

1. Mở trình duyệt → `http://192.168.201.15`
2. Đăng nhập:
   - **Username**: `browser`
   - **Password**: `123456`
3. Vào tab **Network** hoặc **IP Configuration**
4. Đổi IP cho từng LiDAR: `.15`, `.16`, `.17`, `.18`
5. Bấm **Save** → **Parameters Take Effect**
6. Đợi 10-20 giây để LiDAR khởi động lại

### 5.4. Kiểm tra kết nối

Mở **Command Prompt** trên PC:
```cmd
ping 192.168.201.15
ping 192.168.201.16
ping 192.168.201.17
ping 192.168.201.18
```
→ Tất cả phải thấy `Reply from 192.168.201.x: bytes=32 time<1ms TTL=64`

---

## 6. Hướng dẫn đổi IP LiDAR

### 6.1. Khi nào cần đổi IP?

- Triển khai nhiều LiDAR (>4 con)
- Thay đổi cấu trúc mạng
- IP bị trùng với thiết bị khác
- Test tính năng đổi IP động

### 6.2. Quy trình đổi IP (2 bước BẮT BUỘC)

#### ⚠️ BƯỚC 1: Đổi IP trên web hãng

1. Mở trình duyệt → `http://192.168.201.15` (IP hiện tại của LiDAR)
2. Đăng nhập: `browser` / `123456`
3. Vào tab **Network** → đổi IP mới (vd: `192.168.201.99`)
4. Bấm **Save** → **Parameters Take Effect**
5. Đợi 10-20 giây để LiDAR khởi động lại

#### ⚠️ BƯỚC 2: Cập nhật IP trong web viewer

1. Trên web viewer, bấm nút **⚙ IP Config** (màu đỏ)
2. Tìm dòng LiDAR vừa đổi IP
3. Sửa ô **IP Address** thành IP mới: `192.168.201.99`
4. Bấm nút **✔** (Apply) ở cuối dòng
5. Đợi 3-5 giây → Backend tự reconnect + lưu vào `lidar_config.cfg`

### 6.3. Kiểm tra kết quả

- ✅ Trạng thái chuyển sang **● Connected** (màu xanh)
- ✅ Point cloud của LiDAR xuất hiện trên canvas
- ✅ Log panel hiện: `[IP] Thay doi LiDAR 1: 192.168.201.15:1112 -> 192.168.201.99:1112`
- ✅ Backend log: `[OK] LiDAR 192.168.201.99:1112 (TR70_Real1)`

### 6.4. ❌ Lỗi thường gặp khi đổi IP

| Tình huống | Nguyên nhân | Giải pháp |
|---|---|---|
| LiDAR "Disconnected" sau khi đổi IP | Quên cập nhật IP trong web viewer | Mở ⚙ IP Config → Apply |
| Ping fail | Khác subnet (PC: 192.168.201.x, LiDAR: 10.0.0.x) | Đổi IP PC theo subnet mới |
| IP trùng | 2 thiết bị cùng IP | Chọn IP khác (.99, .100, .200...) |
| Port sai | Đổi port trên web hãng nhưng không cập nhật | Sửa port trong web viewer (mặc định 1112) |

---

## 7. Test đổi IP an toàn

### 7.1. Các IP hợp lệ để test

```
✅ CÙNG SUBNET (hoạt động ngay):
192.168.201.15  →  192.168.201.99
192.168.201.15  →  192.168.201.100
192.168.201.15  →  192.168.201.200

❌ KHÁC SUBNET (cần đổi IP PC):
192.168.201.15  →  10.0.0.15
192.168.201.15  →  192.168.1.15
```

### 7.2. Quy trình test (5 phút)

1. **Chuẩn bị**: Đảm bảo 4 LiDAR đang chạy bình thường (.15, .16, .17, .18)
2. **Đổi IP trên web hãng**: `192.168.201.15` → `192.168.201.99`
3. **Ping test**: Mở CMD → `ping 192.168.201.99` → Phải thấy "Reply from..."
4. **Cập nhật web viewer**: ⚙ IP Config → sửa IP → Apply
5. **Kiểm tra**: Trạng thái "Connected" + point cloud hiện ra
6. **Test ổn định**: Để chạy 5-10 phút, kiểm tra FPS, latency, zone detection

### 7.3. Test đổi về IP cũ

- Mở `http://192.168.201.99` → đổi về `192.168.201.15`
- Web viewer → ⚙ IP Config → sửa về `.15` → Apply

---

## 8. Cấu hình mạng cho nhiều LiDAR

### 8.1. Kiến trúc khuyến nghị: Dùng Switch

```
PC (192.168.201.100)
    │
    ├──► Switch 5-8 ports (unmanaged)
    │       │
    │       ├──► LiDAR 1 (192.168.201.15)
    │       ├──► LiDAR 2 (192.168.201.16)
    │       ├──► LiDAR 3 (192.168.201.17)
    │       └──► LiDAR 4 (192.168.201.18)
```

**Ưu điểm:**
- ✅ Không cần cấu hình gì
- ✅ Tất cả thiết bị cùng subnet
- ✅ Rẻ (~200k VND cho switch 5 ports)
- ✅ Ít tốn điện, ổn định

**Switch khuyến nghị:**
- TP-Link TL-SG105 (5 ports Gigabit) ~ 200k VND
- D-Link DGS-1005A (5 ports Gigabit) ~ 250k VND

### 8.2. Nếu dùng Router (cần cấu hình)

#### Cách 1: Router như Switch (KHUYẾN NGHỊ)
- Tắt DHCP server trên router
- Tất cả thiết bị cùng subnet `192.168.201.x`
- Đặt IP tĩnh cho PC và từng LiDAR

#### Cách 2: Router có NAT (KHÔNG KHUYẾN NGHỊ)
- Router chia 2 mạng: WAN (PC) và LAN (LiDAR)
- ❌ PC không kết nối trực tiếp được LiDAR
- ✅ Giải pháp: Cắm PC vào cổng LAN của router

### 8.3. ❌ TUYỆT ĐỐI TRÁNH

- Router có NAT mà không cấu hình port forwarding
- PC và LiDAR khác subnet mà không có router nối
- DHCP server cấp IP trùng với IP tĩnh của LiDAR

---

## 9. Calibration (Hiệu chỉnh ngoại tham)

### 9.1. File cấu hình

- `extrinsics.cfg` (nằm cùng thư mục exe) chứa bộ ngoại tham hiện tại
- Tự động nạp khi khởi động backend
- Format: mỗi dòng `dx dy theta_deg`

### 9.2. Các phương pháp calibration

#### Phương pháp 1: Chỉnh thủ công (phím tắt)

| Phím | Chức năng |
|---|---|
| Q / A | dx ± 0.1 m |
| E / D | dy ± 0.1 m |
| Z / C | theta ± 1° |
| F | Print extrinsics hiện tại |
| V | Save extrinsics (lưu vào extrinsics.cfg) |

#### Phương pháp 2: Pick 2 điểm (K)

1. Bấm **Pick** (hoặc phím K)
2. Trên LiDAR 1 (reference): click 2 điểm đặc trưng (vd: góc tường)
3. Trên LiDAR 2 (cần calib): click 2 điểm tương ứng
4. Bấm **APPLY** → backend tính dx, dy, theta tự động

#### Phương pháp 3: ICP Refine (I)

1. Đảm bảo 2 LiDAR có vùng quét chồng lấn đủ lớn
2. Bấm **ICP Refine** (hoặc phím I)
3. Backend chạy thuật toán ICP (5 iterations) để tối ưu
4. Kết quả: RMS trước → RMS sau + bộ extrinsics mới

#### Phương pháp 4: RMSE@góc 0°

1. Đặt vật chuẩn ở khoảng cách biết trước (vd: 5.0 m) ngay góc 0°
2. Nhập giá trị thực vào ô `true(m)`
3. Bấm **RMSE@goc0**
4. Backend đo 100 mẫu → tính mean, RMSE, std

### 9.3. Lưu ý khi calibration

- Sau khi calib, bấm **V** (SaveCfg) để lưu vĩnh viễn
- Nếu lắp lại vị trí/khung giàn mới: phải calib lại từ đầu
- Fusion view chỉ chính xác khi extrinsics đúng

---

## 10. Xem từ máy khác trong mạng LAN

### 10.1. Mở firewall trên máy chạy backend

Mở **PowerShell (Admin)**, chạy 2 lệnh:

```powershell
New-NetFirewallRule -DisplayName "LiDAR Web (8080)" -Direction Inbound -Protocol TCP -LocalPort 8080 -Action Allow
New-NetFirewallRule -DisplayName "LiDAR WS (9001)" -Direction Inbound -Protocol TCP -LocalPort 9001 -Action Allow
```

### 10.2. Truy cập từ máy khác

Trên máy khác (cùng mạng LAN), mở trình duyệt:
```
http://<IP-may-backend>:8080
```

Ví dụ: nếu máy backend có IP `192.168.201.100`:
```
http://192.168.201.100:8080
```

### 10.3. Kiểm tra kết nối

Từ máy khác, mở CMD:
```cmd
ping 192.168.201.100
Test-NetConnection -ComputerName 192.168.201.100 -Port 8080
Test-NetConnection -ComputerName 192.168.201.100 -Port 9001
```

---

## 11. Build từ source code

### 11.1. Chuẩn bị

- Visual Studio 2022 với workload "Desktop development with C++"
- Các thư viện đã đi kèm trong thư mục `libs/`:
  - `websocketpp` (header-only)
  - `asio` (standalone)
  - `nlohmann/json` (header-only)
  - `ladarsdk_gd.lib` + `ladarsdk_gd.dll` (LiDAR SDK)

### 11.2. Các bước build

1. Mở Visual Studio 2022
2. Mở solution `backend\LidarWebGateway\LidarWebGateway.sln`
3. Chọn cấu hình **Release | x64**
4. Build → Build Solution (Ctrl+Shift+B)
5. File exe sẽ nằm ở `backend\LidarWebGateway\x64\Release\`

### 11.3. Deploy

Copy các file sau vào cùng thư mục:
```
lidar_web_gateway_v3.exe
ladarsdk_gd.dll
frontend\index.html
start_all.bat
tools\vc_redist.x64.exe
```

### 11.4. Thay đổi IP mặc định trong code

Nếu muốn đổi dãy IP mặc định (khi không có `lidar_config.cfg`), sửa trong `main()`:

```cpp
// Trong lidar_web_gateway_v3.cpp, hàm main():
const char* DEF_IPS[4] = {
    "192.168.201.15",
    "192.168.201.16",
    "192.168.201.17",
    "192.168.201.18"
};
```

→ Build lại và deploy.

---

## 12. Phím tắt

### 12.1. Điều khiển chính

| Phím | Chức năng |
|---|---|
| `Space` / `P` | Pause / Resume |
| `←` / `→` | Step frame (khi paused) |
| `1` - `8` | Chọn LiDAR 1-8 |
| `W` | Chụp màn hình (screenshot PNG) |
| `B` | Bật/tắt Fusion view |
| `R` | Reset view (zoom=8, pan=0) |
| `Esc` | Thoát chế độ zone/ruler/pick |

### 12.2. Hiển thị

| Phím | Chức năng |
|---|---|
| `G` | Polar Grid |
| `M` | Rays (tia laser) |
| `N` | Intensity mode |
| `U` | Accumulate mode |
| `L` | Ruler (đo khoảng cách) |
| `Y` | Zone mode |
| `K` | Pick mode |
| `J` | Cluster mode |
| `T` | Đo latency |

### 12.3. Calibration

| Phím | Chức năng |
|---|---|
| `Q` / `A` | dx ± 0.1 m |
| `E` / `D` | dy ± 0.1 m |
| `Z` / `C` | theta ± 1° |
| `F` | Print extrinsics |
| `V` | Save extrinsics |
| `I` | ICP Refine |
| `Enter` | Đóng polygon (khi vẽ zone) |

### 12.4. Zone

| Phím | Chức năng |
|---|---|
| `Y` | Bật/tắt zone mode |
| `0` | Xóa tất cả zone |
| `Enter` | Đóng polygon |
| Chuột phải | Xóa zone đang chọn / đóng polygon |
| `Delete` | Xóa zone đang chọn |

---

## 13. Giao thức WebSocket

### 13.1. Kết nối

```
ws://<host>:9001
```

### 13.2. Client → Server (JSON)

| Lệnh | Mô tả |
|---|---|
| `{"cmd":"get_state"}` | Lấy trạng thái các LiDAR |
| `{"cmd":"lidar_get_config"}` | Lấy cấu hình IP |
| `{"cmd":"select","id":0}` | Chọn LiDAR |
| `{"cmd":"extr_apply","id":0,"dx":0,"dy":0,"thetaDeg":0}` | Áp dụng extrinsics |
| `{"cmd":"extr_adjust","ddx":0.1,"ddy":0,"dth":0}` | Tinh chỉnh extrinsics |
| `{"cmd":"extr_save"}` | Lưu extrinsics |
| `{"cmd":"extr_print"}` | In extrinsics |
| `{"cmd":"pause","paused":true}` | Pause/Resume |
| `{"cmd":"record"}` | Bắt đầu/dừng ghi CSV |
| `{"cmd":"cluster"}` | Bật/tắt cluster |
| `{"cmd":"icp"}` | Chạy ICP Refine |
| `{"cmd":"latency","sleep":false}` | Đo latency |
| `{"cmd":"rmse_start","true":5.0}` | Đo RMSE |
| `{"cmd":"zones_set","zones":[...]}` | Cập nhật zones |
| `{"cmd":"zone_clear"}` | Xóa tất cả zones |
| `{"cmd":"zone_update","zoneId":1,...}` | Cập nhật 1 zone |
| `{"cmd":"lidar_set_ip","id":0,"ip":"...","port":1112}` | Đổi IP LiDAR |
| `{"cmd":"lidar_add","ip":"...","port":1112}` | Thêm LiDAR mới |
| `{"cmd":"lidar_remove","id":0}` | Xóa LiDAR |
| `{"cmd":"lidar_rename","id":0,"name":"..."}` | Đổi tên LiDAR |
| `{"cmd":"lidar_reconnect","id":0}` | Reconnect LiDAR |
| `{"cmd":"ping","id":0}` | Ping LiDAR |

### 13.3. Server → Client

#### JSON messages

| Type | Mô tả |
|---|---|
| `events` | Trạng thái: paused, recording, targets, boxes |
| `extr` | Extrinsics của 1 LiDAR |
| `state` | Danh sách LiDAR + extrinsics |
| `lidar_config` | Cấu hình IP của tất cả LiDAR |
| `latency` | Số liệu latency (RTT, E2E, GOI4320) |
| `rmse` | Kết quả đo RMSE |
| `icp` | Kết quả ICP Refine |
| `ping` | Kết quả ping ICMP |
| `log` | Log message |

#### Binary frame (msgType=2)

```
[0]      uint8   msgType = 2
[1]      uint8   sourceId
[2-3]    uint16  reserved
[4-7]    uint32  frameId
[8-11]   uint32  tsSec
[12-15]  uint32  tsUsec
[16-19]  float32 originX
[20-23]  float32 originY
[24-27]  float32 originTheta
[28-31]  float32 angleMin
[32-35]  float32 angleMax
[36-39]  float32 angleInc
[40-43]  float32 rangeMin
[44-47]  float32 rangeMax
[48-51]  float32 scanTime
[52-55]  float32 timeInc
[56-59]  uint32  waveCount
[60-63]  uint32  pointCount
[64..]   float32[] (localX, localY, intensity) × pointCount
```

---

## 14. Troubleshooting

### 14.1. Mất kết nối LiDAR

| Triệu chứng | Nguyên nhân | Giải pháp |
|---|---|---|
| LiDAR "○ Disconnected" | Backend không tìm thấy LiDAR | 1. Ping LiDAR<br>2. Kiểm tra IP trong ⚙ IP Config<br>3. Kiểm tra cable |
| Ping fail | Khác subnet / cable lỏng / LiDAR tắt | 1. Kiểm tra cable<br>2. Đổi IP PC theo subnet LiDAR<br>3. Kiểm tra nguồn LiDAR |
| Ping OK nhưng TCP fail | LiDAR không mở port 1112 / firewall | 1. Kiểm tra port trên web hãng<br>2. Tắt firewall tạm thời<br>3. Restart LiDAR |
| Kết nối được nhưng không có data | LiDAR standby / lỗi firmware | 1. Gửi lệnh Wakeup (Standby=0)<br>2. Restart LiDAR<br>3. Nâng cấp firmware |
| Kết nối chập chờn | IP trùng / mạng không ổn định | 1. Đổi IP khác<br>2. Kiểm tra switch/router<br>3. Dùng cable CAT5e/CAT6 |

### 14.2. Web không mở được

| Triệu chứng | Giải pháp |
|---|---|
| Chrome không tự mở | Mở thủ công: `http://localhost:8080` |
| "Cannot connect" | Kiểm tra Python đã cài chưa, hoặc double-click `frontend\index.html` |
| Web mở nhưng không có điểm | Kiểm tra backend có chạy không (cửa sổ console đen) |
| "WebSocket disconnected" | Kiểm tra firewall, port 9001 có bị chặn không |

### 14.3. Point cloud không hiển thị đúng

| Triệu chứng | Giải pháp |
|---|---|
| Fusion view lệch | Calib lại extrinsics (Pick/ICP) |
| Zone không báo đúng | Kiểm tra zone scope (Fusion/Local) |
| FPS thấp (< 10) | Giảm số LiDAR, tắt Accumulate/Intensity |
| Latency cao (> 100ms) | Kiểm tra mạng, giảm scan frequency |

### 14.4. Reset LiDAR về mặc định

Nếu không truy cập được web hãng:
1. Rút nguồn LiDAR
2. Nhấn giữ nút Reset (nếu có) trong 10 giây
3. Cắm nguồn lại → LiDAR về IP mặc định `192.168.201.15`

### 14.5. Kiểm tra nhanh (Command Prompt)

```cmd
:: 1. Ping LiDAR
ping 192.168.201.15

:: 2. Kiểm tra port (PowerShell)
Test-NetConnection -ComputerName 192.168.201.15 -Port 1112

:: 3. Xem ARP table (kiểm tra MAC address)
arp -a

:: 4. Kiểm tra firewall rules
netsh advfirewall firewall show rule name=all | findstr "LiDAR"
```

---

## 15. Tài liệu tham khảo

### 15.1. Tài liệu trong dự án

| File | Mô tả |
|---|---|
| `START_HERE - HUONG_DAN.txt` | Hướng dẫn nhanh (đọc trước tiên) |
| `README.md` | Tài liệu này |
| `docs\readme-en.html` | SDK LiDAR manual (tiếng Anh) |
| `docs\ladar_api_en.h` | SDK header file |
| `docs\LadarSet_User_Manual_EN.pdf` | LadarSet software manual |
| `docs\TR60 TR70 user manual.pdf` | Phần cứng TR60/TR70 |
| `BaoCao_TongHop` | Báo cáo quá trình thực hiện |

### 15.2. Thông số kỹ thuật LiDAR TR60/TR70

| Thông số | Giá trị |
|---|---|
| Quét góc | 270° |
| Tầm hoạt động | 0.1 - 100 m |
| Tần số quét | 25 / 50 / 100 Hz |
| Độ phân giải góc | 0.0625 / 0.125 / 0.25° |
| Nguồn | DC 10-32V (khuyến nghị 24V) |
| Công suất | ≤ 20W (sưởi ≤ 60W) |
| Chuẩn bảo vệ | IP67 / IP68 |
| Nhiệt độ hoạt động | -20°C ~ +55°C (wide: -55°C ~ +70°C) |
| Port mặc định | 1112 (base data) |
| IP mặc định | 192.168.201.15 |
| Tài khoản web | browser / 123456 |

### 15.3. Liên hệ hỗ trợ

- **Backend issues**: Kiểm tra log `lidar_web_log.txt`
- **Frontend issues**: Mở DevTools (F12) → Console tab
- **LiDAR issues**: Liên hệ nhà cung cấp phần cứng

---

## 📝 Changelog

### v3.1 (2026-08-18)
- ✅ Hỗ trợ đổi tên LiDAR (lidar_rename)
- ✅ Lưu tên vào lidar_config.cfg
- ✅ Cải thiện UI IP Config panel

### v3.0
- ✅ Hỗ trợ tối đa 16 LiDAR
- ✅ Zone đa hình: Rect/Polygon/Circle/Sector/Freehand
- ✅ Sửa đỉnh zone (vertex editing)
- ✅ Cấu hình IP động
- ✅ Đo latency RTT/E2E/GOI4320
- ✅ Ping ICMP

### v2.0
- ✅ Fusion 4 LiDAR
- ✅ Calibration Pick/ICP
- ✅ Motion detection
- ✅ Zone rectangle

### v1.0
- ✅ Xem 1 LiDAR qua web
- ✅ Hiển thị point cloud cơ bản

---

## 📄 License

Dự án nội bộ. Sử dụng SDK LiDAR theo license của nhà cung cấp.

---

**Chúc bạn sử dụng hiệu quả! 🚀**

Nếu gặp vấn đề, hãy kiểm tra section [Troubleshooting](#14-troubleshooting) trước khi hỏi hỗ trợ.
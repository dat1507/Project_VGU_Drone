# Checklist nghiên cứu hoãn — ưu tiên hai video demo

Mốc cập nhật: 19/10/2026, phân tích dữ liệu warmup session
`7ae2da20fe854e098d511d580d1ee335`. Đã có quyền host trong các lượt được duyệt;
không dùng lỗi sandbox trước đây để suy ra trạng thái runtime hiện tại. Kết quả
dưới đây là bằng chứng đã lưu, cần xác minh lại trước thao tác live.

**Không hoãn các điều kiện tối thiểu để được bay:** phiên/world/x500 đúng, GUI quan
sát được, Disarmed/Landed trước flight, DDS có dữ liệu fresh đúng phiên, EKF/heading
hợp lệ, anchor mới, giới hạn PX4/tracking-error, ACK/mode và đường xử lý abort. Các mục
dưới đây phân biệt điều kiện demo đang chặn với nghiên cứu mở rộng để sau.

**DEMO-ONLY theo chính sách operator mới:** static collision feasibility, AABB/
swept-sphere refinement, tracking-margin envelope và project ceiling 3,2 m
(absolute/relative) được hoãn theo phê duyệt mới, không chặn riêng
`--demo-only`. Log bắt buộc `DEMO_ONLY_COLLISION_RISK_ACCEPTED`; mặc định vẫn giữ
các gates cũ. Không hoãn SITL-only identity/local DDS binding, freshness/anchor,
runtime position/heading/reset, command ACK/Offboard, timeout và kết quả failure
khi nghi collision hoặc không tới Goal. Chưa có phê duyệt live flight/physics.

| Mục | Đã làm | Đang lỗi / chưa chứng minh cho demo | Hoãn sang các tuần sau |
|---|---|---|---|
| DDS data flow và QoS | Warmup nhận đủ sáu topic; 717/717 payload local position khớp ULog; observer đã sửa epoch/reset | Cần chứng minh fresh coherent epoch/anchor live trước setpoint; không dùng lại warmup cũ | Benchmark DDS, tuning QoS và độ trễ chi tiết |
| PX4 EKF/heading readiness | Native EKF/local/global valid; yaw align từ 4,004 s, không filter fault trong snapshot cuối | Final in-flight mag alignment chưa xong nên heading_good_for_control=False; không sửa cờ hoặc bỏ gate trước bay | Phân tích estimator chuyên sâu ngoài điều kiện tối thiểu takeoff/Offboard |
| Offboard failsafe và timeout | Executor tích hợp mới có prestream, Arm/Offboard ACK, takeoff, đủ 17 waypoint và Land; route budget chung 300 s, tests offline | Chưa kiểm chứng live; mất link/estimator/identity không bảo đảm Land, cần operator Pause; follower cũ vẫn giữ nguyên | Nghiên cứu rộng các tổ hợp failsafe; không hoãn kiểm tra abort tối thiểu |
| Gazebo session lifecycle và recovery | Launcher ownership, shutdown-review, verify-stopped đã có | Host runtime/GUI chưa xác nhận; log cũ có multicast network-unreachable | Refactor lifecycle/recovery tổng quát |
| Gazebo Transport/GIL và scene metadata | Đã sửa request adapter qua native CLI và static-field ownership; có tests | Chưa kiểm chứng playback thành công sau sửa; sandbox không truy cập Transport | Benchmark GIL/latency, metadata tổng quát |
| Ownership cleanup và model identity | Token/name/ID/structure/SDF proof và interrupted-create recovery đã triển khai | Model ID 105 còn tồn tại hay không phải review lại; chưa cleanup | Cơ chế quản lý nhiều demo entities; không hoãn ownership trước mutation |
| Collision monitoring runtime | Demo-only có body proxy trên observed GT, collision suspected không được success; physics collision giữ nguyên | Chưa kiểm chứng live; collision feasibility chỉ còn chặn chế độ mặc định; body proxy không phải physics contact sensor | AABB/swept-sphere research, corridor refinement, safety-margin feasibility và calibration collision model |
| Tracking error PSO so với đường bay | Follower có giới hạn 0,25 m, arrival 0,10 m | Chưa có telemetry chuyến bay thật; không có tracking statistics | Thống kê và đồ thị sai số nhiều chuyến |
| Smoothing, jitter, PSO mở rộng | Đã ghi nhận smoothing không hợp lệ; chỉ chọn raw CSV | Không dùng smoothed path cho demo | Toàn bộ cải tiến smoothing/jitter/PSO; không chạy lại optimizer |
| Benchmark seeds và thống kê | Offline 10 seeds/thuật toán đã hoàn thành | Không có blocker demo cần benchmark thêm | Thêm seeds, statistical significance, sensitivity |
| Reproducibility và logs | Có seed/config/hash, output riêng và journal; không ghi đè nghiên cứu | Chưa có flight telemetry/ACK/Gazebo truth/ULog cho hai chuyến | Bộ audit và pipeline reproducibility mở rộng; vẫn lưu log/video từng demo |

Các raw CSV và Start/Goal giữ nguyên. Chế độ mặc định giữ collision/envelope gates;
ngoại lệ DEMO-ONLY ở trên bỏ collision feasibility/extra tracking margin và
project ceiling 3,2 m. Tracking-error, numeric/XY/lower bounds, limits/failsafe PX4
vẫn giữ nguyên. Đổi phương án sang visual-only cần operator đồng ý riêng;
video phải ghi rõ `TRAJECTORY VISUALIZATION — NOT PX4 CONTROLLED FLIGHT`.

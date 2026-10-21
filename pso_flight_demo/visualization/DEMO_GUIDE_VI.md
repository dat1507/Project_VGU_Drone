# Demo A — raw PSO trajectory visualization

Đây là **hoạt ảnh hình học trong hệ Gazebo ENU**, không phải PX4 SITL controlled
flight. Drone minh họa xuất hiện ngay tại Start ở độ cao ENU 2,5 m; không có
Takeoff / Land. Không dùng DDS, anchor, NED, ROS controller hoặc velocity setpoint.

## Dữ liệu và kiến trúc

Nguồn cố định: `pso_flight_demo/outputs_c2/warehouse/offline_pso_20261019T021207Z/validated_run/`.
Chỉ đọc `selected_baseline/raw_enu.csv` và `selected_enhanced/raw_enu.csv`.
Schema thực tế: `x_enu_m,y_enu_m,z_enu_m`, đơn vị mét, 17 waypoint / 16 segment mỗi đường.

| Đường raw đã chọn | Seed | Chiều dài | Thời lượng mặc định |
|---|---:|---:|---:|
| Standard PSO (`baseline`) | 8 | 21,587980979 m | 29,588 giây wall time |
| Enhanced PSO | 8 | 13,573442480 m | 21,573 giây wall time |

Start chung: `(-11.741300836980269, 4.31172650287941, 2.5)`.
Goal chung: `(-5, -3, 2.5)`.

Parser kiểm tra header, finite values, số waypoint, Start/Goal, chiều dài, SHA256
trong index xuất gốc và metadata trial seed 8. Đối chiếu certificate raw
collision-free, radius 0,60 m + margin 0,25 m; xác minh hash SDF/profile hiện tại
và kiểm tra lại **toàn bộ segment liên tục** bằng geometry checker có sẵn.
Không optimizer, không smoothed path, không đổi dữ liệu gốc.

Hai model độc lập:

- `pso_demo_visual_drone`: quadrotor minh họa bằng box/cylinder/sphere; cập nhật pose.
- `pso_demo_visual_path`: 16 đoạn ống mảnh và 17 waypoint đứng yên; Start xanh lá,
  Goal đỏ; Standard vàng cam, Enhanced xanh cyan.

Cả hai `static=true`, chỉ có visual, không collision/sensor/joint/plugin/motor.
Không clone hoặc sửa `x500`. Mỗi model có ownership token trong tên link.
Mọi pose/cleanup kiểm tra cả tên, entity ID và token, không nhận model có sẵn.
Model path không di chuyển theo drone. Nhãn chữ trong Gazebo chưa được kiểm chứng;
dùng màu và overlay tiêu đề khi dựng video, hoặc HTML preview có nhãn sẵn.

Timeline nội suy tuyến tính theo từng raw segment, giữ mọi đỉnh để không cắt góc.
`--speed 1.0 --fps 20 --hold 4 --repeat 1` là mặc định; tốc độ là m/wall-second.
`--repeat 2` phát hai vòng, **teleport Goal → Start** được đánh dấu riêng trong
timeline; đó không phải một segment PSO mới hay động tác bay.

## Ba lệnh host chính

Chạy từ thư mục repository bằng Python hệ thống Ubuntu (`gz.transport13`,
`gz.msgs10`, NumPy và PyYAML có sẵn). Không source ROS workspace. Giữ launcher và
Gazebo GUI hiện tại; không mở server khác. Chỉ lệnh 1 được chạy trong giai đoạn
triển khai. Lệnh 2–3 dành cho operator **sau phê duyệt visualization riêng**.

1. Kiểm tra cả hai nguồn và xuất timeline / SDF / HTML, hoàn toàn offline:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --dry-run
   ```

2. Preview Standard trong world hiện có, sau phê duyệt riêng:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --algorithm standard --execute-visualization
   ```

3. Sau khi cleanup Standard, preview Enhanced, sau phê duyệt riêng:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --algorithm enhanced --execute-visualization
   ```

Mỗi lần tạo output mới tại `pso_flight_demo/outputs_visualization/raw_pso_<UTC>_<id>/`;
không ghi đè. Hai file `*_timeline.csv`, `*_metadata.json`, bốn SDF và hai
`*_preview.html` xuất ở cả dry-run lẫn execute. Mở HTML trực tiếp trong browser
để duyệt trước; không cần server. HTML là đồ thị XY + độ cao, không vẽ obstacle
warehouse, không thay thế bằng chứng hình ảnh Gazebo.

## Gate runtime và giới hạn Pause

**Cập nhật Transport:** trước khi dùng lại hai lệnh live ở trên, chạy
`--check-transport` theo [TRANSPORT_FIX_VI.md](TRANSPORT_FIX_VI.md). Bản sửa dùng
native Transport 13 client riêng cho request để tránh blocking Python GIL với
stats callback. PASS read-only chưa phải GO live; các gate static/pose và latency
vẫn phải được xác minh, không bỏ qua để quay video.

**Cập nhật ownership:** nếu đang kẹt model ID 105 tại `creating`, dùng đúng ba
bước trong [OWNERSHIP_FIX_VI.md](OWNERSHIP_FIX_VI.md). Không suy is_static=False
từ field vắng mặt: static declaration lấy từ SDF đã xác minh, ownership cần
ID/token và toàn bộ cấu trúc visuals. Journal cũ chưa có ID cần `--expect-model-id`.

Operator phải giữ `warehouse_world` Paused. Chương trình subscribe
`/world/warehouse_world/stats`, yêu cầu stats mới ≤2 giây và clock/iterations
không đổi suốt demo; không hardcode mốc 20.000 / 25.000 iterations.
Xác minh scene đúng world, đúng một `x500`, chưa có model demo trùng tên.
Service discovery kiểm tra đúng một provider và schema thực tế:

| Gazebo Transport service | Request → response |
|---|---|
| `/world/warehouse_world/scene/info` | `gz.msgs.Empty` → `gz.msgs.Scene` |
| `/world/warehouse_world/create` | `gz.msgs.EntityFactory` → `gz.msgs.Boolean` |
| `/world/warehouse_world/set_pose` | `gz.msgs.Pose` → `gz.msgs.Boolean` |
| `/world/warehouse_world/remove` | `gz.msgs.Entity` → `gz.msgs.Boolean` |

Trước mutation, operator phải gõ chính xác `VISUALIZATION ONLY` trong terminal
tương tác. Không hỗ trợ pipe, cờ tự approve hoặc mặc định True. Sau xác nhận,
kiểm tra lại stats. Spawn dùng `allow_renaming=false` và quaternion hợp lệ.
Service provider identity phải giữ nguyên; không gọi world control.

ACK create/pose **không được coi là render đã cập nhật**. Chương trình đợi
scene chứa đúng model/structure, thử chuyển một đoạn nhỏ trên segment raw đầu rồi trở lại Start;
chỉ phát timeline khi Pose_V ground truth mới được xác nhận. Hãy quan sát GUI để xác
nhận render thực sự. Nếu update không ổn khi Paused, timeout/rejection hoặc
Transport quá chậm, chương trình dừng, không retry mutation, không unpause.
HTML preview là phương án độc lập để quay trajectory nếu Gazebo không hỗ trợ.

Sandbox phát triển hiện không truy cập được Gazebo Transport (`getifaddrs`
lỗi), nên **chưa xác minh live service discovery hoặc render khi Pause**. API/schema
được đối chiếu bindings và ví dụ cài sẵn Harmonic: `gz/transport13/__init__.py`,
`/usr/share/gz/gz-sim8/worlds/{empty,shapes}.sdf` và `gz/sim/Server.hh`.
Không có kết luận đã spawn/đã chạy demo Gazebo.

## Quay video và cleanup

Chuẩn bị góc camera trong Gazebo GUI hiện có để nhìn toàn tuyến. Dùng screen
recording desktop (Ubuntu `Ctrl+Alt+Shift+R`, nếu có) hoặc công cụ quay màn hình
đang dùng; bắt đầu quay trước khi nhập câu xác nhận. Dùng chữ overlay
“Standard/Enhanced PSO — raw ENU trajectory visualization — no physical flight”.
Giữ đoạn giữ đầu/cuối 4 giây; crop terminal nếu cần. `x500` thật vẫn tách biệt.

Kết thúc/Ctrl+C không tự xóa model. Lệnh cleanup là cùng module với
`--cleanup /DUONG_DAN_OUTPUT/ownership.json` thay cho `--dry-run` (giữ prefix Python
ở lệnh 1). Chương trình cũng in sẵn lệnh cleanup đầy đủ dùng journal ngay trước
khi spawn: copy nguyên lệnh đó, không cần tự ghép đường dẫn.
Chỉ chạy khi muốn xóa demo và gõ `DELETE DEMO VISUALS`.
Chương trình xác minh clock, provider, tên + ID + ownership token trước từng
remove, rồi đợi scene xác nhận biến mất; không dùng wildcard / xóa `x500`.
Cleanup xong mới chạy thuật toán tiếp theo.

Nếu spawn dở dang, dùng `--cleanup /DUONG_DAN_OUTPUT/ownership.jsonl` để phục hồi
từ journal; nonce vẫn bắt buộc. Trước đó dùng `--review-cleanup`; nếu journal
chỉ có creating và chưa có observed ID, bắt buộc chỉ rõ ID từ bằng chứng độc lập
bằng `--expect-model-id`. Không dùng manifest cũ sau restart server hoặc
khi physics đã tiến. Nếu gate từ chối, giữ NO_GO để kiểm tra lại, không xóa theo
tên một cách thiếu căn cứ. Không cần Arm/Offboard, đổi parameter, DDS hay anchor.

Demo này không chứng minh dynamics, tracking, failsafe hoặc readiness của chuyến
bay Enhanced PSO. `live_control_go=false` trong mọi metadata demo.

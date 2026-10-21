# Demo Standard / Enhanced — tiếp tục từ blocker trực tiếp

**BLOCKED / PX4 FLIGHT NO_GO.** Chưa bay hoặc đổi sang visual-only.

`demo_first prepare` chỉ đọc hai raw CSV đã chứng nhận, kiểm tra hash/collision,
xuất bản sao nguyên byte và tọa độ **NED tương đối với A**. Đây không phải PX4
setpoints; không xuất absolute local NED khi chưa có anchor mới. Công thức dùng
module frames hiện có: `p_local_ned = anchor.px4_ned + R_ENU_NED(p_world_enu -
anchor.gazebo_enu)`. Không giả định hai origins trùng nhau.

Hai đường cùng A `(-11.741300836980269, 4.31172650287941, 2.5)` và B `(-5,-3,2.5)`.
Mỗi đường 17 điểm; Standard 21,587980979 m, Enhanced 13,573442480 m.
Envelope 0,60 + 0,25 m giữ nguyên; optimizer không được chạy.

| Điều kiện full flight hiện có | Standard | Enhanced |
|---|---:|---:|
| Raw nominal geometry | PASS | PASS |
| Thời gian tối thiểu với target 0,2 m/s, hold 5 s mỗi waypoint sau A, prestream 1,2 s | 189,140 sim s | 149,067 sim s |
| Timeout controller hiện tại | 120 s: không đủ | 120 s: không đủ |
| Raw cao nhất | 3,2 m ENU | 3,2 m ENU |
| Dung sai tracking ±0,25 m nằm trong cruise bounds hiện tại | Không | Không |

Các segment bị đánh dấu tracking tube trong `demo_plan.json` là do **bounds**;
không phát hiện obstacle-box intersection khi tăng inflation thêm 0,25 m cho hai
raw routes này. Điều đó không chứng minh chuyển động động lực học an toàn. Với
raw point chạm trần, không thể bảo đảm vẫn trong trần bằng dung sai đối xứng dương.
Relative cap 3,2 m còn phụ thuộc landed origin đo mới, không dùng z lịch sử.

Kế hoạch so sánh giữ cùng speed/arrival/tracking/hold/publish/ACK settings. Timeout
chung 220 sim s chỉ là **đề xuất chưa áp dụng**, gồm lower bound Standard + 30 s
dự phòng, làm tròn 10 s; không phải chứng nhận đủ thời gian động lực học. Tăng
timeout riêng không xử lý được độ cao/tracking. Không sửa follower để né các gate.

## Hai lệnh chính trên Ubuntu host

Chạy từ repository; không cần tự ghép nhiều probe. Lệnh 1 không truy cập Gazebo.
Lệnh 2 chỉ đọc/subscription, không spawn controller hoặc tiến physics:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.demo_first prepare
```

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.demo_first check-host --session pso_flight_demo/sessions/0d3aeccae53a4a01a6096ccbf44661ac
```

Output riêng `outputs_demo_first/prepare_*/demo_plan.json` và
`outputs_demo_first/host_readonly_*/host_check.json`; native evidence lưu cạnh đó.
Check dừng ngay khi không thấy host, identity sai hoặc Scene/stats không đọc được;
không retry. Tái dùng adapter/readiness hiện có, không dựng hệ thống safety mới.
Graph discovery không được coi là nhận messages. Khi Paused, thiếu message mới
không chứng minh DDS hỏng; check không tự mở clock window. GUI render phải được
operator nhìn thấy warehouse/x500; PID GUI còn sống không đủ bằng chứng render.

Nếu cần dừng/restart, lệnh thứ ba **chỉ đọc** dùng đúng session vừa xác minh:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.session_launcher shutdown-review pso_flight_demo/sessions/0d3aeccae53a4a01a6096ccbf44661ac
```

Sau đó operator phê duyệt riêng clock/shutdown. Theo giao thức hiện có trong
SHUTDOWN_RECOVERY_VI.md: fresh Disarmed/Landed + đúng ownership/world, native shutdown
trong bounded running-clock window nếu cần; xác nhận `ALL_CHILDREN_EXITED_START_CANDIDATE`
bằng verify-stopped trước start session mới. Không dùng session/mốc iterations lịch
sử trong ví dụ tài liệu. Phiên mới cho **PX4 flight cần DDS**, không dùng tùy chọn
`--without-dds` vốn chỉ phục vụ Demo A. Chưa cấp lệnh mutation vì runtime chưa rõ.

## Đường ngắn tiếp theo

1. Host check/GUI trước. Nếu còn `getifaddrs` hoặc không có stats/Scene, dừng ở
   runtime recovery; không tốn thời gian DDS/Offboard trong world chưa hoạt động.
2. Khi world khỏe, cần DDS messages/heading/anchor đúng phiên và xử lý giới hạn
   full-route height/tracking; sau đó hoàn thiện native takeoff → hover và timeout
   full route trong follower hiện có. Không có deadline ước đoán hoặc lệnh bay GO.
3. Sau phê duyệt flight riêng: Standard trước, Enhanced sau, cùng cấu hình; log
   arrival/Goal, ACK, ground truth và ULog. Landed+Disarmed phải được xác nhận. Drone
   kết thúc ở B: không teleport/đổi anchor để giả về A. Operator cần phê duyệt đưa
   drone về A hoặc clean restart; sau restart bắt buộc anchor/runtime checks mới.

Với các blocker full flight, phương án ngắn hơn cho video là module visual-only
đã có, model độc lập với x500. **Chưa chọn/chạy fallback.** Nếu operator đồng ý:
review model cũ bằng scene và journal trước; ID 105 có thể thuộc server cũ, không
được tự nhận/xóa. Hướng dẫn ownership nằm trong visualization/OWNERSHIP_FIX_VI.md.
Chỉ sau cleanup hợp lệ mới preview từng thuật toán, quay màn hình GUI hiện tại;
không khởi tạo simulator thứ hai. Nhãn video phải phân biệt rõ playback với PX4
flight. Xem DEFERRED_RESEARCH_CHECKLIST_VI.md cho các nghiên cứu để sau.

## Nếu GUI hiển thị nhưng CLI stats không có output

Không lặp check-host hoặc mở clock để thử sửa discovery. Dùng một snapshot chỉ đọc:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.demo_first diagnose-transport --session pso_flight_demo/sessions/0d3aeccae53a4a01a6096ccbf44661ac
```

Lệnh kiểm tra identity trước/sau, chỉ đọc các biến Transport được chọn từ process
server/PX4 (không dump environment đầy đủ), interfaces/routes và chạy các native
CLI probe song song, mỗi probe tối đa 6 giây. Stats dùng duration 3 giây để CLI kết
thúc tự nhiên và flush output; không có yêu cầu chạy physics. Scene vẫn chỉ là
scene/info, không có control/create/set_pose/remove. Không đổi GZ_IP/partition,
route hoặc retry. Output mới trong `outputs_demo_first/transport_diagnostic_*`.
`READONLY_DIAGNOSTICS_ONLY` không phải PASS/readiness; live_control_go luôn false.

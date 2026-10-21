# Flight tích hợp — Standard / Enhanced PSO

**Mặc định vẫn STATIC_NO_GO.** Đã thêm `--demo-only` dành riêng PX4/Gazebo SITL;
chế độ này bỏ project-level ceiling 3,2 m (absolute và relative), static collision/margin
gates theo phê duyệt riêng của operator; không đổi giới hạn/failsafe PX4.
Lượt cập nhật GUI Play chỉ chuẩn bị code/tests, không thực thi Arm/flight/physics.
Hai CSV raw vẫn 17 waypoint, Standard 21,587980979 m, Enhanced 13,573442480 m.
Không optimizer, smoothing, visual playback hoặc thay pose x500.

`integrated_flight.py` chứa plan, trajectory timing và state machine thuần Python;
`integrated_ros.py` nối ROS 2 Jazzy, native readiness và Gazebo observer hiện có.
Mặc định chỉ dry-run. Hai thuật toán dùng cùng cấu hình controller cố định.

## Chạy demo với GUI Play — không cần Clock Coordinator

Executor **đã hỗ trợ physics do GUI chạy từ bên ngoài sau startup**. Nó không
kiểm tra `paused=true` trong chuyến bay và không gọi WorldControl. Snapshot đầu
vào vẫn yêu cầu Paused; khởi động executor khi GUI đã Play sẽ bị từ chối. Vì vậy
không cần thêm flag hoặc cơ chế điều phối clock để dùng quy trình sau:

1. Giữ GUI **Paused**, x500 Disarmed/Landed tại A. Khi chuẩn bị chạy chuyến đã
   được duyệt, chạy lệnh bên dưới. Mốc 15000 và session chỉ dùng nếu vẫn khớp
   runtime; executor tự kiểm tra trước subscriptions. Không nhấn Play sớm.
2. Đợi dòng **`SUBSCRIPTIONS_READY`**, rồi nhấn **Play ngay** trong Gazebo GUI.
   Sau đó giữ Play xuyên suốt readiness → prestream → Arm → takeoff → 17 waypoint
   → B → Land/Disarmed. Lệnh này có thể tự Arm, không phải observer-only.
3. Theo dõi terminal và GUI. `LIVE_READINESS_PASS` nghĩa là fresh DDS, tiến clock,
   timesync epoch, frame và anchor đã được xác nhận; chương trình mới tạo control
   publishers. Khi hoàn tất, chỉ coi thành công nếu `result.json` xác nhận Goal,
   Landed và Disarmed. Nếu chương trình báo operator Pause/abort, operator nhấn
   Pause trên GUI; không tự chạy lại lệnh hoặc tiếp tục bay.

Lệnh Standard **chỉ dành cho operator bắt đầu chuyến bay đã được duyệt**:

```bash
cd ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation
source /opt/ros/jazzy/setup.bash && source ~/ros2_px4_ws/install/setup.bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.integrated_flight --algorithm standard --demo-only --session pso_flight_demo/sessions/7ae2da20fe854e098d511d580d1ee335 --expect-iterations 15000 --execute-flight --confirm-flight PX4_SITL_DEMO_ONLY_FLIGHT_APPROVED
```

Nếu chỉ kiểm tra offline: thay `--execute-flight --confirm-flight ...` bằng
`--dry-run` (không cần source ROS hay session cho dry-run). Không nhấn Play trong
dry-run. Lượt chuẩn bị hiện tại chưa chạy lệnh live trên.

Observer có sẵn trong executor: subscribe sáu topic PX4 + ACK và Gazebo ground
truth. Graph/packet đầu tiên không đủ: cả sáu normalized source timestamps và
GT phải tiến trong cùng epoch trước activation. Không tự sửa offset qua reset;
epoch đổi trước prestream xóa baseline readiness, đổi sau activation thì abort.
Tọa độ ENU → local NED chỉ dùng confirmed anchor vừa đo, không dùng spawn/origin
giả định. Heading cuối vẫn phải PASS tại A trước khi FOLLOW.

Đợi Play quá lâu sẽ hết TTL native proof 30 s hoặc readiness timeout 60 s;
chương trình không Arm và không tự cấp clock/retry. Nếu không thấy
`SUBSCRIPTIONS_READY` thì không nhấn Play để cố vượt lỗi. Nếu đã nhấn Play nhưng
startup/readiness thất bại trước Arm, xác minh vẫn Disarmed/Landed rồi operator
Pause và đánh giá log trước một lần chạy mới; không tự tiếp tục từ artifact cũ.

Terminal in `FLIGHT_LOG_DIRECTORY` cho output mới. Giữ `native_preflight.json`,
`confirmed_anchor.json`, `telemetry.jsonl`, `result.json` và ULog PX4; lỗi trước
node activation có thể chỉ có `failure.json`. Quay màn hình GUI hiện tại, không
cần simulator khác. Enhanced dùng cùng quy trình với `--algorithm enhanced`,
sau khi đưa drone về A bằng thao tác được duyệt, capture anchor mới và có phê
duyệt chuyến Enhanced. Không tự reset pose, model hay tự chạy chuyến thứ hai.

| Điều kiện | PX4 build hiện tại yêu cầu | Gate bổ sung của project |
|---|---|---|
| Arm | `Commander::arm()` gọi health checks và `canArm(nav_state)`; cấm calibration, kiểm tra throttle khi áp dụng; ACK là kết quả thực tế | Disarmed/Landed, session/schema/GCS/logger, initial yaw/tilt, anchor và frame đã xác minh, không controller khác; không force-arm |
| Đổi mode Offboard | `UserModeIntention::change()` kiểm tra `canRun`; `OffboardChecks` cần heartbeat còn mới theo `COM_OF_LOSS_T`; position mode cần local position valid; attitude/rate checks vẫn áp dụng | Prestream 20 Hz ít nhất 1,2 s **cả wall và native time**, mode ACK + `nav_state=14` trước Arm |
| Position setpoint | MC position controller cần dữ liệu NED hữu hạn/position valid và setpoint còn mới; velocity/acceleration feedforward có thể để NaN | NED chỉ được tạo sau measured anchor; actual speed ≤0,7 m/s; tilt ≤15°; tracking ≤0,25 m |
| Takeoff tích hợp | Trong Offboard, `MulticopterPositionControl.cpp:461–491` yêu cầu armed, setpoint mới và vị trí z NED cao hơn thực tế (z nhỏ hơn), hoặc velocity/acceleration hướng lên; PX4 takeoff ramp vẫn chạy | Ramp target thẳng đứng từ ground pose đo được đến A, ≤0,2 m/s, gia tốc ≤0,1 m/s², không chuyển động ngang trước khi ổn định |
| Heading | `heading_good_for_control` là final alignment; không có yêu cầu trực tiếp cờ này trong Offboard position check. Initial yaw và innovation/health checks vẫn được PX4 đánh giá | Initial yaw + GT/EKF heading lệch ≤0,15 rad trước prestream; final heading **bắt buộc tại A trước PSO và suốt FOLLOW** |
| Land | Native `NAV_LAND=21`; ACK/mode chưa đồng nghĩa touchdown | Tiếp tục heartbeat/hold đến ACK + `nav_state=18`, sau đó nhường descent cho native Land; chỉ success khi B đã hold, Land accepted, Landed=True và Disarmed |

Source đối chiếu tại `~/PX4-Autopilot`, build/ULog commit
`b3e343c1532fea4608cd373da3dcdb0fcc3fa763`:
`src/modules/commander/Commander.cpp:668,1097,1213,1245`,
`UserModeIntention.cpp:57`, `HealthAndArmingChecks/checks/offboardCheck.cpp:40`,
`checks/estimatorCheck.cpp:133,643`, `src/modules/mc_pos_control/MulticopterPositionControl.cpp:461`,
`src/modules/ekf2/EKF/ekf.h:268`.
Đây là các nhánh liên quan trực tiếp tới demo, không thay thế toàn bộ health checks
PX4 theo mode/parameter hiện tại. Không đổi PX4 parameters hoặc health gates.

Luồng tự động duy nhất: readiness → prestream → Offboard ACK → Arm ACK → vertical
takeoff → hold A → final heading → toàn bộ raw waypoints → hold B → Land ACK →
Landed và Disarmed. FOLLOW dùng profile trapezoid 0,15 m/s (takeoff giữ 0,2 m/s),
cùng gia tốc 0,1 m/s² cho cả hai thuật toán. Setpoint vẫn position-only; profile
được nội suy liên tục trên từng segment raw. Chuyển waypoint chỉ sau khi chặng đã
kết thúc, vị trí nằm trong 0,10 m, vận tốc không quá 0,10 m/s và hold đủ 5 s.
Tracking watchdog 0,25 m không đổi. Không cần demo takeoff/hover riêng, không gửi `NAV_TAKEOFF`
global-altitude vì chọn takeoff trực tiếp bằng Offboard position ramp đã có trong PX4.
Waypoint A là điểm 0; 16 segment còn lại đều được đi đúng thứ tự, không shortcut.
Mỗi segment dùng trapezoidal timing trên đường thẳng raw, dừng và hold 5 s tại đích.
Giới hạn route chung 300 s simulation, toàn mission 400 s simulation / 800 s wall;
takeoff 45 s simulation, ACK 3 s wall, Land/disarm 45 s wall. Không retry lệnh tự động.
Standard cần ít nhất khoảng 247,23 s riêng route theo timing FOLLOW hiện tại;
Enhanced khoảng 193,85 s. Đây không phải
bằng chứng tracking hoặc thời lượng bay thực tế.

Observer mới giữ timestamp thô để audit, đồng thời chuẩn hóa native clock theo
offset/epoch. Không ghép offset qua reset; biên chưa rõ vẫn NO_GO. Live clock yêu
cầu ≥3 status tiến, protocol DDS, RTT <10 ms, deviation observed/estimated ≤100 ms;
vẫn phải có cặp DDS/GT cách ≤20 ms. Các ngưỡng clock-jump/RTT lấy từ Timesync PX4;
20 ms frame gate, 0,02/0,15 m vị trí và các gate cũ không được nới.
Health errors của cold-start không bị xóa. Dữ liệu warmup cũ vẫn chưa PASS toàn cửa sổ.
Reset clock/EKF sau prestream làm abort, không tự ghép lại anchor giữa chuyến bay.

Trước khi tạo bất kỳ control publisher nào, executor lấy native snapshot Paused
đúng iterations và owned session, đối chiếu schema, parameter limits, GCS/logger,
scene/x500 entity ID; không nhận model demo ngoài dự kiến. Sau khi subscriptions
đã tạo, nó chờ telemetry fresh và paired Gazebo pose/PX4 local position để xác nhận
anchor mới. Origin Gazebo và local EKF được dịch đúng, không giả định trùng nhau.
Proof native có TTL 30 s; nếu hết hạn thì dừng trước control và cần capture lại,
không tăng TTL. Gazebo observer chỉ subscribe, journal lưu paired GT/telemetry ở
20 Hz và DDS/ACK/command events. Giữ ULog native trong PX4 rootfs.

Physics vẫn do operator quản lý theo phê duyệt riêng. Executor không unpause,
step hoặc Pause. Khi world còn Paused, nó chờ tối đa 60 wall seconds rồi NO_GO;
operator nhấn GUI Play trong thời hạn proof, sau thông báo `SUBSCRIPTIONS_READY`.
Flight cần clock chạy đủ mission budget; không dùng cửa sổ kiểm chứng 5–15 s để
bay toàn route. Không restart, reset pose hoặc tự quay drone từ B về A cho chuyến sau.
Enhanced cần trở lại A bằng quy trình được duyệt và anchor mới.

Khi lỗi tracking/timeout mà position/link/estimator còn dùng được và cột hạ cánh
collision-free, executor yêu cầu Land một lần rồi theo dõi ACK/mode/Landed/Disarmed.
Khi mất DDS, GT, estimator, epoch, identity hoặc không có cột descent an toàn,
không thể bảo đảm Land; dừng control và báo **OPERATOR PAUSE REQUIRED**. PX4 failsafe
Offboard/GCS hiện tại vẫn có hiệu lực, không đổi sang policy khác. Mất process/
API/ghi log cũng không có bảo đảm Land; operator phải sẵn sàng Pause trên GUI.
Không coi watchdog inline đọc nguồn GT riêng là supervisor process độc lập.

Các blocker của chế độ mặc định (không `--demo-only`):

- Raw geometry nominal PASS nhưng tube tracking ±0,25 m không nằm trọn trong
  cruise bounds 1,0–3,2 m. Cả hai CSV chạm z=3,2 m; một số segment còn vi phạm
  lower bounds của tube. Không obstacle hit trong audit tube hiện tại. Live command
  từ chối **trước khi import ROS adapter**, không đổi CSV hoặc nới bounds để chạy.
- Relative height kể cả tracking phải ≤3,2 m theo ground anchor thực tế. Ground
  z ULog cũ khoảng −0,013 m cho thấy ngay target z=3,2 m có thể vượt cap; không dùng
  ULog cũ làm anchor mới. Cần operator quyết định riêng cách xử lý route/bounds;
  chỉ `--demo-only` mới bỏ project-level ceiling theo phê duyệt mới; PX4 limits giữ nguyên.
- Chưa có fresh coherent window/confirmed anchor, chưa chứng minh final heading
  tại A hoặc bất kỳ tracking/ACK/Land live nào. Lượt chuẩn bị này không chạy live.

Lệnh hiện được phép (offline, không cần ROS):

```bash
python3 -B -m pso_flight_demo.integrated_flight --algorithm standard --dry-run
```

Lệnh Standard **chỉ sau khi blocker đã giải quyết và có phê duyệt riêng**; mốc
iterations phải là mốc Paused đã xác minh khi đó, không mặc định vẫn 15.000:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_px4_ws/install/setup.bash && PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.integrated_flight --algorithm standard --session pso_flight_demo/sessions/7ae2da20fe854e098d511d580d1ee335 --expect-iterations 15000 --execute-flight --confirm-flight PX4_PSO_FLIGHT_APPROVED
```

Enhanced dùng đúng lệnh trên với `--algorithm enhanced`, cùng controller; bắt
buộc phê duyệt chuyến mới, kiểm tra session/mốc thực tế, landed tại A và anchor mới.
Không chạy lệnh live trong giai đoạn chuẩn bị này. Với dữ liệu hiện tại lệnh vẫn
trả `STATIC_NO_GO`, không tạo publishers hoặc gửi lệnh bay.

## DEMO-ONLY — policy riêng đã được yêu cầu

`--demo-only` giữ nguyên 17 raw waypoint, A/B, hash CSV và thuật toán PSO.
Plan và journal ghi `DEMO_ONLY_COLLISION_RISK_ACCEPTED`. Chỉ bỏ static collision
feasibility, AABB/swept-envelope gate, extra tracking-margin reserve và project ceiling
3,2 m (absolute ENU lẫn relative so với ground anchor). Journal/plan ghi thêm
`DEMO_ONLY_PROJECT_CEILING_BYPASSED`. Không
đổi SDF, obstacle, model x500, Gazebo collision physics hoặc PX4 parameters.

Mặc định không có flag vẫn thực thi toàn bộ collision gates cũ. Demo-only vẫn
kiểm tra tọa độ hữu hạn, XY/lower bounds, actual speed/tilt,
tracking error ≤0,25 m, timestamp progression, fresh anchor, EKF reset, session,
PX4 health/ACK và Offboard. Bỏ tracking-margin feasibility không có nghĩa bỏ
tracking-error watchdog. Raw peak z=3,2 m và ground origin âm không còn bị chặn
bởi project ceiling ở riêng demo-only. Không nâng hay thay bất kỳ waypoint nào;
không đặt một trần thay thế. Chế độ mặc định vẫn giữ absolute/relative cap.

Live demo-only bắt buộc chứng minh local executable `build/px4_sitl_default/bin/px4`,
launcher ownership, PX4 env `PX4_SIMULATOR=gz`, world warehouse/x500/airframe 4001,
sensor IDs SIMULATION và Agent loopback 127.0.0.1:8888. DDS reader của cả ba control
topics phải thuộc cùng participant đã xác minh với PX4 telemetry. Controller chỉ
discovery trên localhost; external DDS discovery overrides bị từ chối. Không có
chế độ cho serial/remote/hardware vehicle và không có flag bỏ các kiểm tra này.

Collision feasibility không chặn preflight. Trong runtime, conservative body
proxy radius 0,60 m kiểm tra mọi Gazebo pose nhận được so với static obstacles;
không thêm safety/tracking inflation vào detector này. Đây là **collision suspected**,
không giả mạo physics contact sensor. Nghi va chạm làm mission thất bại, yêu cầu
Land một lần nếu state/link/position còn hợp lệ; nếu không, operator phải Pause.
Không đánh dấu success sau va chạm, dù đã tới Goal; không khẳng định collision-free.
Stall/tracking/timeout/mất Offboard cũng làm thất bại. Landed và Disarmed vẫn bắt buộc.

Dry-run hiện được phép, không ROS/control:

```bash
python3 -B -m pso_flight_demo.integrated_flight --algorithm standard --demo-only --dry-run
```

Lệnh live Standard **chỉ sau phê duyệt riêng**, session/mốc/telemetry/anchor hợp lệ,
Jazzy và overlay đã source. Token demo-only khác token mặc định:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.integrated_flight --algorithm standard --demo-only --session pso_flight_demo/sessions/7ae2da20fe854e098d511d580d1ee335 --expect-iterations 15000 --execute-flight --confirm-flight PX4_SITL_DEMO_ONLY_FLIGHT_APPROVED
```

Mốc 15.000 chỉ dùng nếu đã xác minh lại đúng mốc. Script không cấp physics:
operator phê duyệt và vận hành clock riêng, bắt đầu sau `SUBSCRIPTIONS_READY`;
readiness timeout 60 s, native proof TTL 30 s. Route budget chung 300 sim s,
mission 400 sim s / 800 wall s, áp dụng cho cả hai thuật toán. Nếu clock Pause,
không còn source progression thì không Arm; trong chuyến bay mất clock dẫn abort.
Enhanced đổi `--algorithm enhanced` với cùng cấu hình, sau khi trở lại A theo
quy trình được duyệt và có fresh anchor/phê duyệt chuyến mới. Không tự reset pose.

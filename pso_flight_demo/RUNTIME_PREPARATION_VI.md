# C2 — kiểm tra runtime, anchor và chặng Enhanced PSO đầu tiên

Entrypoint mới: `python3 -B -m pso_flight_demo.runtime_preparation`. Có `snapshot`, `observe-window`, `check-window`, `check-anchor`, `candidate` và observer `observe` cũ; không có execute, Arm, Takeoff, Land, Offboard hoặc điều khiển physics. Output luôn tạo mới; không sửa session launcher hay các artifact offline.

## Trạng thái bàn giao

- Parser GCS đã sửa trong cả `runtime_readiness` và `session_runtime`. Evidence lịch sử: telemetry instance 0 có heartbeat True, instances 1–3 False; parser dict cũ trả False của instance cuối. Hàm mới bảo toàn instance, chọn bất kỳ link nào được PX4/MAVLink phân loại GCS, có RX>0, timestamp không tương lai và tuổi ≤1 s native, cùng vehicle_status mới xác nhận GCS không lost. Không cố định index 0. Duplicate fields/instances fail closed.
- `gcs_progress()` yêu cầu clock native, timestamp link và RX tăng trên cùng link được phân loại GCS ở hai snapshot. Đây là bằng chứng RX+heartbeat-classification, không phải bộ đếm heartbeat riêng. Không gọi snapshot True là heartbeat đang tiến triển.
- Đã đọc ROS 2 Jazzy và cài đặt px4_msgs sau source workspace; 10 schema serialized fields và MESSAGE_VERSION khớp header PX4 đã build. VehicleLocalPosition v1, VehicleStatus v4, VehicleCommandAck v1, TimesyncStatus không suffix. Không khởi tạo ROS participant trong kiểm tra inventory.
- DDS runtime mới chưa được nhận. Mức đã có từ log cũ: GRAPH_ONLY. Không có fresh anchor được công nhận, không có NED setpoints/live bundle.
- Trong sandbox, boot ID trùng lịch sử nhưng supervisor PID không truy cập được. Lệnh snapshot đã dừng trước native IPC với NO_GO `Supervisor absent/restarted`. Điều này không chứng minh tiến trình host đã chết. Phải kiểm tra ngoài host; không adopt/restart hoặc tạo launcher thứ hai.
- Source đang chạy trong launcher cũ không được hot-reload. Lệnh CLI mới chạy bằng Python process riêng để sử dụng parser đã sửa; không thay bộ nhớ hay state của launcher đang chạy.

## Quy trình hiện tại: 20.000 → 25.000, ba terminal

Terminal A: giữ launcher hiện tại chạy; không tạo launcher khác. Các PID lịch sử chỉ là tham chiếu. Chỉ chuẩn bị công cụ trong tác vụ Codex; chưa cấp physics.

Terminal B chạy lệnh 1 trên Ubuntu host. Wrapper tự chọn repository, source ROS Jazzy và workspace, đặt domain 0 rồi chạy `observe-window --wall-seconds 60 --expect-iterations 20000` (global option được truyền đúng vị trí). Không cần ghép các lệnh setup:

```bash
bash ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation/pso_flight_demo/observe_window_host.sh
```

Preconditions: cùng session `f9dc3f80b86541ceac91f65d435a9876`/boot/PID/start ticks/profile/SDF; đúng warehouse_world và một x500; Paused, iterations=20000, native sim=20 s; Disarmed+Landed; COM_ARM_ON_BOOT=0; EKF/preflight/GCS tốt; schema đúng; không ROS control publisher. Nếu khác: NO_GO, không tự rebase plan, không restart/adopt tiến trình, không thay parameters. Heading false vẫn ngăn flight GO; một cửa sổ thu dữ liệu disarmed không cấp quyền bay.

B kiểm tra native trước, tạo ROS node, kiểm tra input control publishers, tạo Gazebo subscriber và đầy đủ sáu DDS subscriptions, spin executor để discovery. Chỉ sau graph/schema/GID/readers phù hợp và native baseline được kiểm tra lại mới in **SUBSCRIPTIONS_READY + đường dẫn output**. Đây không phải bằng chứng đã nhận message. **Giữ B đang chạy; không đợi B kết thúc rồi mới cấp physics.**

Sau phê duyệt riêng của operator cho cửa sổ 5 s, Terminal C chạy lệnh 2 với đường dẫn B vừa in. Phrase phải được operator truyền tường minh, không có default approval:

```bash
bash ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation/pso_flight_demo/observe_window_host.sh check-window /ABS/OUTPUT_FROM_B --operator-approval APPROVE_CLOCK_20000_25000
```

C kiểm tra heartbeat file ≤1 s, đúng PID/start ticks/boot ID của observer đang sống, còn ≥12 s trong deadline gốc; đọc lại native 20000/20 s/Paused/Disarmed/Landed/auto-arm-disabled/identity. Sau đó gửi nonce. B tiếp tục spin DDS, đồng thời kiểm tra native ở worker riêng và trả response cho nonce đó với graph hiện tại. Challenge timeout 6 s, không retry. File challenge/response publish atomically và không overwrite. C chỉ in lệnh khi response mới ≤1 s và còn ≥8 s; **C không thực thi lệnh**.

Chỉ khi C báo `READY_FOR_SEPARATE_CLOCK_APPROVAL`, B còn đang chạy và operator đã phê duyệt, operator trực tiếp thực hiện lệnh 3 ngay trong C:

```bash
gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true, multi_step: 5000'
```

Nếu B đã kết thúc, báo NO_GO hoặc proof đã hết hạn: không dùng lại lệnh cũ, không tự retry. Giới hạn 5–60 s của CLI vẫn giữ nguyên; không gia hạn deadline sau approval. `observer_status.json` có heartbeat/deadline/identity, `coordination_events.jsonl` lưu state transitions. Không có file/cờ nào khiến observer tự điều khiển physics.

State flow: CHECK_SESSION → CHECK_PX4_DISARMED_LANDED → CHECK_GAZEBO_PAUSED → CHECK_NO_CONTROL_PUBLISHERS → START_DDS_AND_GAZEBO_OBSERVERS → SUBSCRIPTIONS_READY → WAIT_FOR_OPERATOR_APPROVAL → CLOCK_WINDOW_EXECUTED_BY_OPERATOR → VERIFY_EXACT_CLOCK_PROGRESS → STOP_AND_SAVE_OBSERVATIONS → ASSESS_DDS_AND_SESSION → PASS/NO_GO. Approval chỉ tạo audit event; state CLOCK_WINDOW_EXECUTED_BY_OPERATOR chỉ dựa trên native stats tiến thực tế.

## Rollback và tiêu chuẩn bằng chứng

Operator theo dõi B/C. ACK WorldControl không chứng minh hoàn tất. Yêu cầu đúng +5000 iterations, +5.000.000 µs, end=25000/25 s và Paused. Nếu unexpected arming/motion, sai identity/gates, quá 20 s wall sau khi cấp clock hoặc B timeout/fail giữa cửa sổ: operator trực tiếp dùng **rollback Pause** được C in trong `rollback_command` (`gz service ... --req 'pause: true'`), rồi xác minh Paused. Không gửi thêm multi_step; không kill/restart. Observer không thể rollback thay operator.

Native checks chạy trên worker bounded, không chặn executor. Loop observer tối đa 60 s wall; cleanup và native after-probes có timeout riêng. B ghi timing lúc tạo subscriptions/kết thúc, spin_count/max_spin_gap, QoS request và publisher QoS, toàn bộ message timestamps/receive indices, native poll timestamps và state events. `source_sequence=null` vì các PX4 messages này không có sequence nguồn; receive index không phải sequence PX4.

Gazebo transport13 subscriber độc lập đọc `/world/warehouse_world/pose/info`, chỉ nhận đúng x500/entity ID từ native pose; register thất bại không được ready. Lưu ENU/quaternion, native sim timestamp và wall receive time. Cả DDS observer và Gazebo observer không có publisher điều khiển hay world-control client. `VehicleCommandAck` optional; không gửi command để tạo ACK thử.

| Mức | Ý nghĩa |
|---|---|
| GRAPH_ONLY | Discovery có publishers, chưa message |
| MESSAGE_RECEIVED | Có message; chưa đủ progression của mọi topic |
| TIMESTAMP_PROGRESSING | Sáu topic đều có source timestamps tiến; chưa tự chứng minh native clock/session |
| PASS_FINITE_CLOCK_EVIDENCE_ONLY | `assess_window()` cùng handshake đều PASS; luôn live_control_go=false |
| NO_GO | Sai clock/session/gates, thiếu dữ liệu hoặc timeout; không tự retry |

Sáu topic: VehicleLocalPosition v1, VehicleStatus v4, VehicleAttitude, VehicleLandDetected, EstimatorStatusFlags, TimesyncStatus; topic suffix lấy từ installed MESSAGE_VERSION. QoS vẫn best-effort/volatile/depth 100. Không nới các ngưỡng: exact native +5 s/+5000 steps và Paused trước/sau; một publisher mỗi output, cùng RTPS prefix/sysid-component 1/1; chuẩn hóa XRCE timesync bằng native = DDS timestamp + estimated_offset; ghép timesync trong 0,5 s wall; coverage ≥3 s, gap ≤1,5 s, source stamps trong ±50 ms của native window; native/DDS position và Gazebo/native pose ≤2 cm; reset/reference/heading không đổi bất thường; GCS timestamp/RX tăng.

Khi Paused và native delta=0, không có DDS mới không chứng minh DDS hỏng. Log lần ad242... có 3479 GT samples nhưng **tất cả native stamp=15.000.000 µs**, wall receive span khoảng60,274 s; native trước/sau cũng là15 s/15000. Vì vậy lần này không quan sát được physics window; không xác định được thứ tự trước/sau của lần cấp physics từ các file ấy. Chưa có bằng chứng QoS/executor bị lỗi. Instrumentation và handshake mới loại bỏ thiếu hụt về thời điểm/liveness, đồng thời giữ fail-closed khi data/schema/QoS hoặc assessment không đạt.

## Fresh anchor và check-anchor

Snapshot tạo candidate đo từ native GT + EKF, không lấy spawn làm anchor. Kiểm tra sample gap ≤20 ms; pose native Gazebo so GT ≤2 cm; SDF ENU heading zero; GT heading khớp chuyển yaw ENU→NED ≤0,02 rad; EKF/GT heading lệch ≤0,15 rad và `heading_good_for_control=True`. Ngưỡng heading là gate chuẩn bị bảo thủ mới, không sửa heading/rotate path để ép qua.

Lưu anchor cùng session ID, boot ID, PX4/Gazebo PIDs/start ticks, profile digest, SDF hash, native timestamps, sáu reset/reference fields, frame evidence và native gates. Candidate vẫn có thể được lưu khi GCS/DDS chưa GO, luôn `flight_ready=False`. `check-anchor` gọi native probe mới, kiểm tra ownership trước/sau, cả hai packet ≤30 s wall, hash/identity/reset/heading và sai lệch translation ≤0,15 m. Không chỉ đọc lại hai file cũ rồi gọi là fresh.

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.runtime_preparation check-anchor /absolute/new_snapshot/anchor_candidate.json
```

Sau clock/reset/reboot phải chụp candidate mới. Không dùng packet trước cửa sổ vì tuổi/reset/frame có thể đổi. NED diagnostics chỉ được tạo khi check-anchor thành công; chúng không phải setpoints hoặc live bundle. Formula dùng translation đo được và ENU↔NED trục (N=Y,E=X,D=−Z), không giả hai origin trùng nhau.

## Chặng 0,775 m và toàn bộ chuyển tiếp

Đọc đúng parent Enhanced seed 8, hash parent `dccc348e…c11e3b`, hash prefix `bef82d46…493c5`, hai điểm chính xác trùng raw[0:2]. Không optimize/rerun, đổi seed/Goal, sửa CSV hay smoothing.

- Start: (-11,74130083698;4,31172650288;2,5) ENU.
- Waypoint 1: (-11,05771522598;4,18750868729;2,84432252136) ENU.
- Length: 0,775420758 m; delta NED=(-0,124217816;+0,683585611;−0,344322521) m. Delta không thay thế anchor tuyệt đối.
- Segment và tracking tube 0,25 m qua geometry với radius 0,60 + margin 0,25. Chặng còn dưới cap relative 3,2 theo snapshot cũ (đỉnh relative 2,8573 m); phải kiểm lại bằng fresh anchor.
- `candidate_assessment()` kiểm tra các đoạn Ground→Start→Hold→Waypoint→Hold→Ground, kể cả các đoạn đứng và bounds XY. Tất cả inflated obstacle boxes giữ 0,85 m. Floor envelope gần tiếp xúc đất thất bại một cách tường minh, không được miễn/giảm để gọi cả chuyến là collision-free. Động lực takeoff/ground contact/landing vẫn NO_GO.
- Đề xuất Start Hold ổn định 3 s native trong cùng chuyến, không có chuyến Offboard 0,5 m riêng. Target ≤0,2 m/s, actual ≤0,7 m/s, tracking ≤0,25 m, arrival ≤0,10 m, hold waypoint 5 s. Cận thời gian chặng+prestream+hold 10,077 s; dự trù cả takeoff 30 + Start Hold 3 + ACK 3 + Land 30 + reserve 20 ≈96,077 s. Đây là dự trù, không chứng nhận dynamics. Chưa có giới hạn gia tốc target trong follower: NO_GO cho dynamics.
- Follower hiện chạy wall 20 Hz, prestream 1,2 s lấy từ DDS adjusted clock, ACK mode trong 3 s wall, route timeout 120 s, Land/disarm 30 s. Cần kiểm chứng RTF/clock vì timer này chưa dùng native simulation clock. Nó chỉ nhận aircraft đã airborne tại Start, không thực hiện Takeoff/Hold native.
- Dùng execution prefix hai điểm, giữ parent provenance: đến waypoint đầu, hold 5 s rồi request Land, không mở raw[2]. Chưa tạo live manifest. Không Pause ngay sau Land request; chờ ACK/mode và landed+disarmed, trừ tình huống Pause khẩn cấp.

## Watchdog/failsafe và phạm vi chưa chứng minh

`runtime_safety.GroundTruthWatchdog` là decision policy độc lập, không điều khiển. Nó so Gazebo ENU với anchor-converted EKF và target, kiểm swept segment GT, tốc độ và relative cap. Ngưỡng: GT stale >0,25 s wall, DDS stale >0,5 s, setpoint gap >0,15 s, sample native lệch >20 ms, EKF–GT >0,15 m, tracking >0,25 m, speed >0,7 m/s hoặc reset thì phát quyết định lỗi. Các timestamp đưa vào WatchSample phải được chuẩn hóa native; không truyền DDS epoch trực tiếp.

Chỉ `REQUEST_LAND` khi estimate/link/GT còn đủ tin cậy, armed+airborne và descent column đã clear. Nếu reset, mất DDS/GT/estimate/GCS, geometry không an toàn hoặc không đủ điều kiện Land: `REQUEST_EMERGENCY_PAUSE`. Hai trạng thái đều `executed=False`. `LandConfirmation` phân biệt request, ACK accepted + mode Land, rồi landed AND disarmed; ACK reject/3 s không xác nhận/30 s không hạ thì yêu cầu Pause, không retry.

Reader Gazebo và policy có unit tests, **chưa tích hợp vào live follower/supervisor**, chưa end-to-end flight test. Không hứa tự Land trong mọi lỗi. `ros_follower.py` được giữ nguyên; current abort của nó vẫn có thể Pause khi telemetry/estimate lỗi.

Parameters chỉ có snapshot cũ: COM_OF_LOSS_T=1 s, COM_OBL_RC_ACT=0 Position, NAV_DLL_ACT=2 Return. Chưa chứng minh policy indoor Land, chưa đổi parameters. Lệnh snapshot mới đọc lại failsafe/MPC limits; cần review và phê duyệt thay cấu hình riêng nếu cần. Mất controller có thể làm PX4 dùng policy Position/Return hiện tại, không thể dựa vào Python watchdog để bảo đảm Land khi DDS mất.

Kết luận bàn giao: **GO_FOR_NEXT_READONLY_CHECK** trên Ubuntu host; flight vẫn **NO_GO** cho đến khi có fresh session/DDS/anchor/heading, kiểm chứng và xử lý dynamics/takeoff/failsafe/watchdog, rồi phê duyệt bay riêng. Không tuyên bố đã bay hoặc đã xác nhận ACK/Land/ULog mới.

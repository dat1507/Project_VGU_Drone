# Demo A — sửa giao tiếp scene/info, chưa cấp quyền live visualization

**Cập nhật sau bản sửa này:** blocker Scene.is_static và pose graph được xử lý
bằng [OWNERSHIP_FIX_VI.md](OWNERSHIP_FIX_VI.md): SDF declaration + full structure
cho ownership, Pose_V fresh cho motion. Phần giới hạn bên dưới ghi nhận trạng thái
trước bản sửa ownership; không bỏ qua bước review/cleanup hiện tại.

## Bằng chứng API/source và giới hạn kết luận

Gói cài thực tế: `python3-gz-transport13` / `libgz-transport13` **13.6.0**,
`libgz-sim8-plugins` **8.15.0**. Wrapper đang đọc:
`/usr/lib/python3/dist-packages/gz/transport13/__init__.py`.

Chữ ký đúng: `Node.request(service, request, RequestClass, ReplyClass, timeout_ms)`.
Wrapper serialize request, gọi `request_raw`, parse bytes bằng ReplyClass rồi trả
`(bool, reply)`. Type cũ `gz.msgs.Empty` → `gz.msgs.Scene` và payload `b''` là đúng;
unit test gọi wrapper cài sẵn qua fake C++ boundary đã xác minh điều đó.

Các vấn đề source đã xác nhận:

1. [Binding đúng release 13.6.0, dòng 204–225 và 263–276](https://github.com/gazebosim/gz-transport/blob/gz-transport13_13.6.0/python/src/transport/_gz_transport_pybind11.cc#L204)
   không release GIL khi gọi blocking request; raw subscriber callback acquire GIL.
   [RunReceptionTask/TriggerCallbacks](https://github.com/gazebosim/gz-transport/blob/gz-transport13_13.6.0/src/NodeShared.cc#L311)
   xử lý subscriber và service response trên cùng reception thread. Nếu stats đến
   trong lúc main giữ GIL đợi request, callback có thể chặn đường nhận reply đến
   timeout. Đây là cơ chế lỗi của kiến trúc cũ; tăng 1500→5000 ms không loại bỏ nó.
2. Binding dùng cùng biến cho return và output result của RequestRaw, ghi đè
   service callback result bằng cờ executed. Bool của Python không phải Boolean.data
   hay bằng chứng callback thành công. API C++ có **executed** và **service result**
   riêng; wrapper chỉ trả một bool. Thông báo cũ gộp timeout/rejection, mất chẩn đoán.
3. [SceneInfoService 8.15.0](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/scene_broadcaster/SceneBroadcaster.cc#L677)
   trả scene render settings và models, không gán tên world vào Scene.name. Tên
   này có thể trống. Check bắt buộc `scene.name == WORLD` sẽ chặn response hợp lệ
   **sau khi request thành công**; nó không gây ra message timeout cũ.

CLI hoạt động với response 27.047 bytes là bằng chứng operator cung cấp. Sandbox
Codex vẫn báo `getifaddrs` khi discovery; chưa tái hiện được transaction lỗi trên
host. Không khẳng định GIL là nguyên nhân duy nhất của lần timeout đó, cũng chưa
loại trừ latency hoặc khác biệt môi trường terminal. Probe mới thu đủ thông tin
để phân biệt chúng. Không có bằng chứng DDS/PX4 lỗi từ sự cố này.

## Thay đổi adapter

- Python Node giữ discovery và subscriber stats. Service request dùng binary có
  sẵn `/usr/libexec/gz/transport13/gz-transport-service`, chính backend Transport 13
  của `gz service` trên Ubuntu này, chạy subprocess với argv và shell=false.
  Không tạo simulator, không sửa package/binding, không thử Python rồi fallback
  gửi lại. Mỗi transaction chỉ gửi đúng một lần.
- Native CLI kế thừa cùng environment snapshot. GZ_PARTITION/GZ_IP/discovery
  ports/multicast/config/verbose đổi giữa invocation thì NO_GO. Không tự đặt GZ_IP.
  Diagnostic lưu module/extension path, Python executable, partition, PYTHONPATH,
  LD_LIBRARY_PATH, provider/socket ID, schema, số bytes và latency thực tế.
- `scene/info` dùng 5000 ms như CLI operator đã chạy; outer wall deadline 6 giây.
  Create/set_pose/remove giữ 1500 ms, outer 2,5 giây. Đây là giới hạn chờ, không
  bảo đảm đạt fps. Không nới stats freshness 2 giây hoặc animation lag gate 1 giây.
- Kiểm tra một provider, schema và socket ID **trước/sau** request; latch mọi clock
  progression/unpause ngay cả khi đã Pause lại. Giữ nguyên allowlist bốn services.
- Parse strict protobuf text; không chỉ dựa vào exit code. [CLI 13.6 source](https://github.com/gazebosim/gz-transport/blob/gz-transport13_13.6.0/src/cmd/gz.cc#L243)
  phân biệt timeout và callback false bằng stdout/stderr; exit code có thể vẫn 0.
- Chấp nhận Scene.name trống dựa trên service namespace đầy đủ đã pin provider và
  stats `/world/warehouse_world/stats` bất biến; tên khác không rỗng bị từ chối.
  Yêu cầu đúng một x500, không x500_0, tên/ID models duy nhất và ID dương.
  Ownership nonce/link/entity ID, static gate, xác nhận operator và guards còn nguyên.

| Mã lỗi | Ý nghĩa |
|---|---|
| DISCOVERY_MISSING / AMBIGUOUS / SCHEMA_MISMATCH | Thiếu, nhiều hoặc sai schema provider |
| PROVIDER_CHANGED | Provider/schema đổi trong hoặc giữa request |
| REQUEST_TIMEOUT | Native service timeout hoặc outer wall timeout |
| REQUEST_FAILURE | Callback false, client launch/exit/stderr error |
| BOOLEAN_REJECTED | Service trả Boolean.data=false |
| INVALID_RESPONSE | Protobuf sai schema, thiếu trường bắt buộc hoặc Scene trống |
| WRONG_SCENE / INVALID_SCENE_MODELS | Tên scene sai hoặc x500/name/ID không hợp lệ |
| TRANSPORT_ENV_CHANGED / READONLY_VIOLATION | Môi trường đổi hoặc cố mutation trong probe |

## Hai lệnh hiện được phép chuẩn bị/chạy trên host

Từ repository, cùng terminal environment với `gz service` đang hoạt động:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --check-transport
```

Đây là kiểm tra **chỉ đọc**: subscribe stats, discovery bốn services và gọi duy nhất
scene/info. Không xác nhận operator, không create/set_pose/remove/control. Output mới
`outputs_visualization/transport_check_<UTC>_<id>/transport.json` và
`scene_response.pbtxt`. Thành công trả `PASS_READONLY_TRANSPORT`, nhưng
`visualization_go=false`, `live_control_go=false`. Nếu lỗi thì NO_GO, không retry.

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --dry-run
```

Dry-run vẫn không import/khởi tạo Transport, chỉ xuất timeline/SDF/HTML vào thư mục mới.
Standard 17 waypoint / 21.587980979 m; Enhanced 17 / 13.573442480 m.

## Điều kiện trước khi cấp lại lệnh live

Chưa có PASS từ Python adapter trên host sau sửa, nên **chưa cấp lại lệnh live**.
Không tự spawn để dò lỗi tiếp theo. Sau PASS read-only vẫn cần review các gate
model/pose và approval visualization riêng, không chỉ CLI exit=0.

Source SceneBroadcaster 8.15 tạo model graph với name/id/pose, không gán is_static;
pose graph cũng không thay thế stream pose hiện tại. Adapter vẫn yêu cầu static
evidence trong `owned()` và scene pose update trong `verify_pose()`: nếu trường
không được cung cấp hoặc pose không cập nhật, nó tiếp tục NO_GO. Không bỏ các check
này trong bản sửa Transport. Đây là blocker cần xử lý bằng bằng chứng phù hợp
trước khi hứa demo live sẽ chạy trọn tuyến.

Native CLI mỗi request có overhead discovery/process. Cần xem elapsed_wall_s trước
khi chọn fps hoạt ảnh; 20 fps chưa được benchmark trên host. Gate lag giữ nguyên,
không tự giảm fps/tốc độ hoặc bỏ waypoint để ép chạy. HTML preview vẫn độc lập.

Đã chạy 50 tests offline (30 hiện có + 20 mới), 0 FAIL; tests dùng fake Node/CLI,
giữ tests CSV/hash/raw interpolation/ownership/x500. Không execute-visualization,
không spawn/đổi pose, restart hoặc physics trong nhiệm vụ này.

# Khôi phục visual model ID 105 — chỉ chuẩn bị offline

## Root cause và bằng chứng

Manifest gốc không sửa:
`outputs_visualization/raw_pso_20261019T072352Z_dc7b7f23/ownership.jsonl`.
Token: `dc7b7f2315c24dc5a2f2ad450dae14f2`; world `warehouse_world`;
initial clock `[0,0,0]`. Journal có header `models: []` rồi `creating` cho
`pso_demo_visual_path`; chưa có `created`. ID **105** là bằng chứng operator đã
cung cấp, chưa phải xác nhận live trong lần sửa này.

SDF `enhanced_pso_demo_visual_path.sdf` được đọc và so byte-for-byte với model
sinh lại từ raw CSV đã chứng nhận, cùng thuật toán/token:

- `<static>true</static>`, một link `owner_dc7b7f2315c24dc5a2f2ad450dae14f2`.
- 33 visuals: 17 waypoint và 16 segment; không collision, joint, sensor, plugin.
- SHA256: `ca774e033938dda64922c1cf0b9b7d508dd8027d54ede589cc4a4ca33d1a73ae`.

Điều kiện cũ `not model.is_static` đã nhầm default protobuf với trạng thái runtime.
Test bindings thật cho thấy Model không có trường is_static trong ListFields()
vẫn trả `False`. [SceneBroadcaster Sim 8.15, SceneGraphAddEntities/AddModels](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/scene_broadcaster/SceneBroadcaster.cc#L758)
tạo model message với name/id/pose, không gán static. Vì vậy lỗi này là false
negative ownership, không chứng minh model thực sự dynamic.

## Nguồn chứng minh mới và các giới hạn

Ownership cần đồng thời:

1. Run/config/metadata hợp lệ, raw CSV/SDF warehouse/profile đúng hash; model SDF
   khớp chính xác generator hiện tại với raw route, algorithm và nonce gốc.
2. Provider/schema của bốn services và simulation clock vẫn khớp journal; stats
   fresh, Paused, không có clock progression. Restart dù clock trở về 0 vẫn bị chặn
   khi provider/socket ID đổi. Không thêm service vào allowlist.
3. Tên model trong allowlist, root entity ID đúng giá trị đã ghi hoặc ID operator
   chỉ rõ; đúng một link/token; không nested model/joint/sensor/plugin thêm vào.
4. Bộ tên/count visuals, geometry, kích thước, local poses, link pose và hierarchy
   IDs đúng SDF. Nếu parent_id có giá trị thì phải đúng link; SceneBroadcaster
   cũng có thể bỏ parent_id, khi đó ancestry được xác minh bằng protobuf nesting.

Static declaration được chứng minh bằng **SDF**, không suy từ giá trị mặc định.
Report phân biệt `static_source=VALIDATED_EXPORTED_SDF`,
`static_scene=NOT_PUBLISHED`, `runtime_static_proven=false` khi trường vắng mặt.
Không tuyên bố đã đọc ECM Static component hoặc chứng minh physics từ Scene.
Nếu field được xuất rõ và mâu thuẫn thì từ chối. Chứng cứ trên dùng để nhận diện
đúng visual do demo tạo, không cấp flight readiness hoặc quyền thay đổi x500.

Journal/parser mới từ chối malformed/truncated JSON, duplicate keys/events,
token/schema/world sai, ID boolean/không dương/trùng nhau và event sai thứ tự.
Không nhận entity từ tên một mình. Với journal `creating` cũ chưa có ID, bắt buộc
`--expect-model-id 105`; ID live khác thì NO_GO, không tự chọn ID khác.

Lần tạo mới ghi `creating` → `observed {name,id}` → `created`, fsync mỗi event.
Observed được lưu **trước** ownership verification. Nếu create ACK nhưng chưa
quan sát được entity, journal vẫn thiếu ID và bắt buộc review/ID độc lập.
Không sửa hoặc backfill journal gốc. Không xóa bất kỳ model không xác minh được.

## Ba lệnh host tiếp theo

Chạy từ repository. **Lệnh 1 chỉ đọc. Lệnh 2–3 cần phê duyệt riêng**, không được
chạy chỉ vì tests PASS. Codex không thực hiện các lệnh này trong lần triển khai.

1. Xác minh fresh ownership của đúng model ID 105, chỉ subscribe/read Scene:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --review-cleanup pso_flight_demo/outputs_visualization/raw_pso_20261019T072352Z_dc7b7f23/ownership.jsonl --expect-model-id 105
   ```

   Chỉ tiếp tục nếu `READY_FOR_SEPARATE_CLEANUP_APPROVAL`, đúng name/ID/token,
   33 visuals kiểm tra hợp lệ, provider/clock không đổi. Output mới:
   `outputs_visualization/ownership_review_<UTC>_<id>/cleanup_review.json`.
   Nếu ID 105 đã mất, được thay thế, khác server hoặc file thay đổi: dừng NO_GO.

2. Sau phê duyệt cleanup, chỉ xóa đúng visual đã chứng minh:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --cleanup pso_flight_demo/outputs_visualization/raw_pso_20261019T072352Z_dc7b7f23/ownership.jsonl --expect-model-id 105
   ```

   Đọc report name/ID trước khi nhập `DELETE DEMO VISUALS`. Chương trình kiểm tra
   lại source hashes, provider/clock, entity ID, token và hierarchy sau lúc chờ
   nhập, rồi ngay trước remove. Chỉ request `Entity { name: pso_demo_visual_path,
   id: 105, type: MODEL }`. Không đụng x500 hoặc drone visual không nằm trong
   journal. Đợi Scene xác nhận model biến mất; ACK một mình chưa đủ.
   Timeout ghi trạng thái request chưa chắc chắn, không retry và không tự xóa tiếp.
   Chỉ chuyển bước khi có `cleanup_complete.json` với `OWN_VISUALS_REMOVED`.

3. Sau cleanup thành công và phê duyệt visualization riêng:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.visualization.raw_pso_gazebo --algorithm enhanced --execute-visualization
   ```

   Cần cả hai tên demo không tồn tại. Nhập `VISUALIZATION ONLY` sau prechecks.
   Output/session/token mới; không tái sử dụng journal cũ cho model mới.
   Tốc độ hoạt ảnh/fps vẫn cấu hình tường minh; không tự giảm tốc, bỏ waypoint
   hoặc nới lag gate để ép chạy. Latency thực tế của CLI vẫn cần theo dõi trên host.

## Xác minh chuyển động và quay video

Scene graph giữ initial model pose nên không dùng nó để xác nhận set_pose đã áp
dụng. Follower visualization giờ subscribe chỉ đọc `/world/warehouse_world/pose/info`
(Pose_V), dùng model name + ID đã chứng minh, stamp cùng clock, dữ liệu fresh và
nhận sau request hiện tại. Ignore x500; không publish vào ROS/PX4. Pose ACK mà
không có ground truth khớp vẫn NO_GO trong timeout cũ, không tiến physics.
Các bước này chỉ được chạy khi operator thực hiện lệnh 3; chưa có bằng chứng live
ở lần sửa này. Cần kiểm tra GUI thực sự render rồi mới quay màn hình.

Sau Enhanced, cleanup bằng journal mới (đã có observed/created IDs), rồi mới chạy
Standard theo DEMO_GUIDE_VI.md. Đây là visual trajectory, không phải chuyến bay
PX4. Không yêu cầu DDS/anchor/Offboard. Giữ `live_control_go=false`.

## Kiểm chứng đã thực hiện

75 tests PASS, 0 FAIL (50 tests visual/Transport hiện có + 25 ownership/recovery
tests mới). Bao gồm static absent/contradiction, cấu trúc sai, x500/ID/token sai,
malformed manifest, interrupted create, observed ID, source thay đổi, restart,
cleanup chỉ ID 105, remove timeout/ACK chưa xác nhận và pose stream.
Tests dùng fake scenes/clients, không gửi request live. Các file source/CSV/SDF
warehouse/PX4 và dữ liệu nghiên cứu được giữ nguyên ngoài phạm vi module/tests/docs.

GO: chuẩn bị offline và fresh read-only review. Cleanup/visualization vẫn NO_GO
cho đến khi gates tương ứng đạt trên host và có phê duyệt operator riêng.

# Xử lý SHUTDOWN_BLOCKED — chỉ chuẩn bị, chưa shutdown/restart

## Nguyên nhân đã xác nhận từ source và session lưu trên đĩa

Session: `f9dc3f80b86541ceac91f65d435a9876`. Đây là bằng chứng lịch sử, không phải
xác nhận các PID còn sống ở thời điểm operator đọc tài liệu.

`session.json` ghi `shutdown_gate.safe=true`, `paused=true`, world đúng,
iterations=21936, sim_time=21.936 s. Hai listener đã trả:

| Topic | Giá trị | Source timestamp | Tuổi theo simulation clock |
|---|---|---:|---:|
| actuator_armed | armed=False | 21.781 s | 0.155 s |
| vehicle_land_detected | landed=True | 21.028 s | 0.908 s |

Cả hai nằm trong ngưỡng **≤1 giây simulation time** có sẵn. Logger probe exit=0;
ULog được ghi nhận: `./fs/log/2026-10-19/01_59_22.ulg` trong PX4 rootfs.
Không suy ra status cũ còn mới theo wall time; cần listener/native probes mới.

Source trước sửa của `Supervisor.shutdown()` có điều kiện:

```python
if not gate["safe"] or gate["paused"]:
    # SHUTDOWN_BLOCKED trước khi gọi host.shutdown_px4()
```

Vì vậy nguyên nhân của request này là **paused=true**, không phải parser hiểu sai
Armed/Landed. File `shutdown_processed_*.json` chỉ chứng minh owner đã nhận request.
`REQUEST_QUEUED_NOT_PROCESS_EXIT` không xác nhận PX4 đã nhận native shutdown hay đã
thoát. Trong snapshot không có `shutdown_result`; nhánh block nằm trước native call.

Đây là chính sách có chủ đích, đã ghi trong `REBOOT_GUIDE_VI.md`. Cơ sở PX4:

- `src/systemcmds/shutdown/shutdown.cpp`: gọi `px4_shutdown_request()`.
- `platforms/common/shutdown.cpp`: queue worker với delay; so sánh
  `hrt_absolute_time() > shutdown_time_us`, gọi shutdown hooks và có thể requeue.
- `platforms/common/work_queue/work_thread.c`: thời gian work queue dựa trên HRT.
- `src/modules/logger/logger.cpp`: đăng ký `Logger::request_stop_static` làm hook.

Clock SITL đứng có thể giữ worker/hook chờ. Không đổi gate thành True để né lỗi GUI.
Native source cũng có timeout riêng; process exit không tự chứng minh ULog đã flush
hoàn hảo. Giữ nguyên log, không tuyên bố đã kiểm chứng đóng log khi chưa có bằng chứng.

## Thay đổi nhỏ trong lifecycle hiện có

- `shutdown_gate()` giữ ngưỡng freshness 1 giây, tách `safe` (snapshot state) khỏi
  `can_shutdown`, so sánh hai stats trước/sau native probes để chứng minh cả
  iterations và sim_time tiến. `paused=false` đơn lẻ chưa đủ. Timestamp 0/invalid,
  stale, future, wrong world, armed/not-landed và probe timeout đều bị chặn.
- `Supervisor.shutdown()` giữ Pause block, báo đúng nguyên nhân; đối chiếu profile/
  SDF và identities trước mutation; chờ cả PX4 descendants trước Agent/Gazebo.
  Native request đúng một lần; timeout giữ children/ownership để review, không force.
- `shutdown-review SESSION` chỉ đọc: đối chiếu boot ID/PID/start ticks/exe/UID/SID/
  PGID, world/model, profile/SDF, native state, logger, COM_ARM_ON_BOOT=0 và conflicts.
  Chỉ loại đúng PID supervisor đã xác minh khỏi conflict scan, không bỏ qua cả login
  session hoặc controller khác. Không dùng DDS, EKF flight gates hay anchor.
- CLI `shutdown --execute` mới chạy review trước khi xếp request. Vì vậy khi Paused
  nó trả BLOCKED **trước khi tạo shutdown.request**; khi clock thực sự tiến, owner
  vẫn phải kiểm tra lại độc lập.
- `verify-stopped` chỉ đọc /proc, cả supervisor và recorded/unrecorded descendants
  trong child SIDs, cùng host process/UDP checks. `status` vẫn là JSON cache, không
  thay thế được kiểm tra process exit. Verify không start session.

**Supervisor đang chạy đã import code cũ. Sửa file không hot-reload nó.** Không
restart/kill supervisor để áp dụng patch. CLI mới dùng giao thức request cũ, bổ sung
kiểm tra trước queue; owner hiện tại vẫn giữ gate cũ và quyền shutdown children.
Các cải tiến bên trong Supervisor chỉ áp dụng từ phiên mới. `verify-stopped` độc
lập vẫn kiểm tra được descendants sau khi owner cũ hoàn tất.

## Ba lệnh host tiếp theo (hai Terminal đã chuẩn bị sẵn)

Chạy từ repository. Bước 1 là read-only. Bước 2–3 chỉ thực hiện sau phê duyệt clock
và shutdown riêng; Codex chưa thực hiện bất kỳ bước runtime nào trong lần sửa này.
Giữ launcher Terminal đang chạy; không Ctrl+C/kill owner hoặc children.

1. Terminal B — lấy kiểm tra mới ngay trước cửa sổ:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.session_launcher shutdown-review pso_flight_demo/sessions/f9dc3f80b86541ceac91f65d435a9876
   ```

   Chỉ tiếp tục nếu `READY_FOR_SEPARATE_CLOCK_APPROVAL`, `safe=true`, inventory
   khớp, world/model đúng, auto_arm_disabled=true, clock Paused và bất biến trong
   probes, không pending request/native attempt/controller. Nếu NO_GO, dừng.
   Nếu session/PID/world đổi, không sử dụng proposal cũ. Review mới sinh baseline
   thật; **không áp dụng lại kế hoạch DDS 20000→25000**.

2. Terminal B — sau approval, dùng đúng `clock_proposal.command` vừa in:

   ```bash
   gz service -s /world/warehouse_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true, multi_step: 5000'
   ```

   Đây là **tối đa 5000 bước** với pause:true, không continuous unpause. SDF hiện tại
   khai báo timestep 0.001 s, nên tương ứng tối đa khoảng 5 simulation seconds nếu
   cấu hình giữ nguyên. Với baseline lịch sử 21936, trần là 26936; không mặc định
   baseline đó còn đúng. Không đổi timestep/params. ACK chỉ là request, không phải
   chứng minh clock đã tiến hay đã tự Pause. Không tăng budget/retry tự động.

3. Terminal C đã chuẩn bị sẵn — chạy **ngay trong khi cửa sổ đang tiến**, không đợi
   5000 bước kết thúc rồi mới gửi shutdown:

   ```bash
   PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.session_launcher shutdown pso_flight_demo/sessions/f9dc3f80b86541ceac91f65d435a9876 --execute
   ```

   CLI mới kiểm tra hai native stats thực sự tiến và Landed/Disarmed còn fresh,
   identities/world đúng trước queue. Owner kiểm tra lại, gửi native shutdown một
   lần, đợi PX4 exit rồi dừng owned Agent và Gazebo graceful. Nếu window quá ngắn,
   gate sẽ block; **không tự cấp cửa sổ thứ hai**. Nếu đã có native shutdown attempt
   hoặc timeout, không gửi lại native request. Giữ NO_GO để review.

Finite clock ở đây phục vụ cleanup; PX4 exit có thể khiến lockstep dừng sớm, rồi
Gazebo được shutdown. Không yêu cầu nó đạt đủ 5000 bước khi PX4 đã thoát, cũng không
coi nó là bằng chứng DDS. Nếu shutdown bị chặn và world còn chạy, operator dùng
`clock_proposal.rollback_pause_command` (pause:true, không multi_step), kiểm tra stats
Paused; không suy ra đã Pause chỉ từ ACK. Tool không tự thực thi rollback.

## Sau shutdown: xác nhận hết children rồi mới start phiên mới

Trong Terminal C, sau khi request được queue, dùng kiểm tra chỉ đọc có deadline:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.session_launcher verify-stopped pso_flight_demo/sessions/f9dc3f80b86541ceac91f65d435a9876 --timeout 120
```

Chỉ tiếp tục khi `ALL_CHILDREN_EXITED_START_CANDIDATE`, restart_allowed=true,
inventory không còn supervisor/children, không identity mismatch/task chưa ghi nhận,
host_precheck không blocker và các cổng được giải phóng. PID reuse hoặc sandbox không
thấy host không được coi là “đã thoát”. Nếu timeout: NO_GO, không SIGKILL/killpg/pkill,
không unlink launcher locks, không thêm launcher. Giữ ULog/session/report cũ.

Sau khi kết quả trên đạt và operator duyệt start phiên mới:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.session_launcher --profile warehouse start --without-dds --execute
```

Đây là session mới có Gazebo/PX4, bỏ Agent vì Demo A không cần DDS; không sửa rcS
hoặc parameter PX4 (DDS client của rcS có thể vẫn tồn tại). Startup vẫn chạy
precheck/locks/ownership gates; nếu conflict xuất hiện lại thì dừng. Không reuse
session identity/anchor cũ. Không cần hoàn thiện DDS/Offboard, không tự warmup hay
bay. QGC hiện có không bị quản lý/kill; không mở thêm QGC. GUI trắng/xám vẫn cần
xác minh sau restart; chưa khẳng định restart sẽ sửa renderer.

Chỉ sau khi phiên mới Paused/GUI hiển thị đúng mới xin phê duyệt Demo A riêng, theo
`visualization/DEMO_GUIDE_VI.md`. Dữ liệu raw/timeline/visualization không thay đổi.

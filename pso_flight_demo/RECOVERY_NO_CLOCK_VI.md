# Recovery phiên không có clock — đề xuất, chưa được duyệt thực thi

Session: `0d3aeccae53a4a01a6096ccbf44661ac`. Các PID dưới đây là bằng chứng host
operator vừa cung cấp, phải xác minh lại boot ID, PID/start ticks, executable,
UID/SID/PGID và quan hệ ownership trước bất kỳ signal nào.

## Kết luận từ bằng chứng

- Một Gazebo server/PX4, đúng identities; operator nhìn thấy warehouse và x500.
- Stats không được discover/nhận, Scene CLI timeout, cả client mặc định và client
  ép Wi-Fi. Không tiếp tục thử IP hoặc mở clock để chữa discovery.
- Native `actuator_armed`: armed=False nhưng timestamp=0; vehicle_land_detected
  chưa từng publish. COM_ARM_ON_BOOT=0; logger đáp ứng, ULog đang mở.
- Chưa đủ bằng chứng fresh Landed/Disarmed; trạng thái Paused/Running UNKNOWN.
- `rollback()` không dừng children khi PX4 đã được tạo; `shutdown()` đòi world,
  fresh state và clock tiến. SIGINT supervisor cũng gọi gate này, không mở một
  đường recovery khác. Sửa source không hot-reload supervisor đang chạy.

Không có bước shutdown bình thường được cấp GO trong trạng thái này. Chưa thể
chứng minh nguyên nhân cụ thể của lỗi network bên server. Không coi đây là flight
thành công hoặc tái sử dụng readiness/anchor của phiên trước.

## Phương án đề xuất cần operator duyệt ngoại lệ

**Hủy phiên SITL lỗi bằng graceful-signal attempt**, tách khỏi normal shutdown.
Phương án này chấp nhận thiếu chứng cứ Landed/fresh clock cho mục đích hủy mô
phỏng; không thay gate flight hoặc giả gate normal shutdown thành True.
Chưa triển khai/chạy executor cho ngoại lệ này trong lần làm việc hiện tại.

1. Xác minh lại ownership/inventory, profile/SDF; lưu báo cáo và bản sao log vào
   output mới. PID reuse/mismatch, foreign process hoặc native báo Armed=True
   đều chặn. Không dùng tên process để nhận quyền sở hữu, không broad killpg/pkill.
2. Sau phê duyệt, đề xuất một SIGINT qua pidfd tới đúng PX4 đã xác minh (PID tham
   chiếu 25994), rồi chờ tối đa 30 wall seconds cho PX4/descendants thoát. Không
   native Arm/Land/Offboard, không unpause, không retry hoặc SIGKILL/SIGTERM escalation.
   Nếu PX4 không thoát hoặc identity đổi, dừng recovery; giữ Agent/Gazebo.
3. Chỉ khi PX4 đã thoát: SIGINT một lần tới các member đã xác minh của Agent
   (owner 25984), rồi Gazebo (launcher 25780, server 25782, GUI 25783), với tối đa
   30 wall seconds mỗi nhóm. Không tác động QGC hoặc supervisor 25773. Supervisor
   hiện có dự kiến tự ghi EXITED và thoát khi tất cả children/members đã hết.
4. Nếu tất cả children và supervisor thực sự thoát, dùng verify-stopped để xác
   minh không còn foreign conflicts/ports trước khi đề xuất start phiên mới.
   Nếu owner không tự thoát: dừng, không kill owner/unlink locks. Start/restart
   vẫn cần phê duyệt riêng; không tự tạo launcher thứ hai.

**Giới hạn:** SIGINT ở PX4 POSIX làm main thoát khỏi wait_to_exit rồi gọi shutdown;
không thể đảm bảo logger shutdown hook đã flush ULog khi HRT/clock đứng. File ULog
đang mở (`06_57_17.ulg` theo snapshot) có thể không hoàn chỉnh. Giữ nguyên file cũ,
không ghi đè hoặc tuyên bố đã flush. PX4 source/logger/warehouse/CSV không bị sửa.

Đây là đề xuất ngoại lệ cần đồng ý rõ ràng, không phải lệnh operator được phép
chạy ngay. Không có lệnh signal/start/physics nào được thực thi ở lần sửa này.

## Code offline đã hoàn tất

`session_runtime.world_snapshot()` trả paused=null nếu stats timeout/không hợp lệ;
`shutdown_gate()` báo CLOCK_UNKNOWN_STATS_UNAVAILABLE thay vì suy ra đang chạy.
Giữ fail-closed, không có clock_proposal khi thiếu chứng cứ.

83 tests PASS, 0 FAIL: test_shutdown_review, test_session_launcher và test_demo_first.
Ba tests mới kiểm tra timeout, empty-success và bool mặc định khi stats thực sự
đọc được. Code đã sửa không hot-reload supervisor đang chạy.

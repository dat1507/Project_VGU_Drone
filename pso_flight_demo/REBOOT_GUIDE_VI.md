# Khởi động lại Project_VGU_Drone sau reboot

## Phạm vi hiện tại

Đã có mã executor session_launcher, kiểm thử bằng mocks/offline. **Chưa chạy
startup/shutdown thật trên host.** Mặc định vẫn dry-run; chỉ thực thi khi có
--execute. Trong đợt triển khai này chỉ được duyệt viết code/kiểm thử offline,
chưa được duyệt dùng --execute trên phiên hiện tại.

c2 launch và c2 shutdown-plan vẫn chỉ in kế hoạch để giữ tương thích. Executor
mới dùng entry point riêng bên dưới. Không có lệnh arm, takeoff, chuyển mode,
setpoint, chạy PSO, unpause hoặc multi_step trong executor.

Tất cả đường dẫn lấy từ profile; không cần cài lại hoặc build lại sau mỗi reboot
nếu binaries/workspace trên ổ đĩa còn hợp lệ. Không chạy sudo, make SITL hoặc
một launcher khác song song với quy trình attach-to-existing-model này.

## Cách dùng ngay lúc này: chỉ dry-run

Mở Ubuntu Terminal tại thư mục dự án:

    cd /home/phuocdat/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation
    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse launch

Dry-run của executor mới, không tạo session/lock/log, không mở tiến trình:

    python3 -B -m pso_flight_demo.session_launcher --profile warehouse start

Có thể thêm --without-dds hoặc --check-host. --profile nhận tên builtin hoặc
đường dẫn YAML; không mặc định một world khác nếu profile chưa cấu hình được.

Kết quả: PRINT_COMMANDS_ONLY; launched_processes=0; các cờ auto_arm,
auto_takeoff, auto_unpause và auto_follower đều false.

Kiểm tra xung đột trên **Ubuntu host**, vẫn không khởi động gì:

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse launch --check-host

Nếu đang có phiên mô phỏng, kết quả NEW_SESSION_BLOCKED là đúng: không khởi tạo
thêm phiên. QGC đã mở được ghi nhận để giữ lại GUI đó, không mở bản thứ hai.
Nếu bị sandbox chặn hoặc không xác lập được khả năng quan sát host, không được
suy ra rằng host không có tiến trình. ps/ss thất bại luôn chặn new session.
NEW_SESSION_CANDIDATE chỉ có nghĩa chưa thấy xung đột, không phải GO bay.

Chỉ in quy trình tắt:

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.c2 --profile warehouse shutdown-plan

## Cấu hình đã đối chiếu

| Thành phần | Cấu hình warehouse |
|---|---|
| Project | ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation |
| Profile | pso_flight_demo/profiles/warehouse.yaml |
| World SDF | simulation_checks/task2_x500_uri/sparse_world.x500_test.sdf |
| World / model | warehouse_world / x500; SDF đã include đúng một x500 |
| PX4 | ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 |
| CWD/rootfs PX4 | ~/PX4-Autopilot/build/px4_sitl_default/rootfs |
| Startup / airframe | etc/init.d-posix/rcS / 4001_gz_x500, instance 0 |
| ROS | /opt/ros/jazzy/setup.bash |
| px4_msgs + Agent workspace | ~/ros2_px4_ws/install/setup.bash |
| Agent ưu tiên | ~/ros2_px4_ws/install/microxrcedds_agent/bin/MicroXRCEAgent |
| Agent dự phòng | MicroXRCEAgent trên PATH; đường dẫn cụ thể được in trong plan |
| QGC | ~/Downloads/QGroundControl-x86_64.AppImage; mở bằng GUI/Terminal người dùng |
| DDS | UDP 8888, domain 0, namespace rỗng theo profile hiện tại |

Workspace ~/Documents/VGU_Project_Drone/ros2_ws chứa state_reader là thành phần
quan sát tùy chọn; không bắt buộc cho module PSO. Mặc định chỉ source Jazzy và
ros2_px4_ws; không source thêm overlay không rõ schema.

Các biến startup được in từ profile:

| Biến | Vai trò |
|---|---|
| GZ_SIM_RESOURCE_PATH | Ưu tiên models của PX4 và thư mục SDF; giữ thêm các resource path hiện có |
| PX4_SYS_AUTOSTART=4001 | Chọn airframe x500 |
| PX4_SIMULATOR=gz | Chọn backend Gazebo |
| PX4_GZ_STANDALONE=1 | PX4 không tự tạo Gazebo world |
| PX4_GZ_WORLD=warehouse_world | Chọn đúng world |
| PX4_GZ_MODEL_NAME=x500 | Gắn vào x500 đã có; không spawn x500_0 |
| PX4_UXRCE_DDS_NS | Namespace từ profile, hiện rỗng |
| PX4_UXRCE_DDS_PORT | Agent port từ profile, hiện 8888 |
| ROS_DOMAIN_ID | Cùng domain với client DDS, hiện 0 |

Không đặt PX4_HOME_LAT/LON/ALT cho phiên này: SDF sở hữu spherical origin.
Plan bỏ ba biến này khỏi môi trường của tiến trình PX4 tương lai để startup
không tự gọi set_spherical_coordinates. Không sửa biến của shell/phiên hiện tại.

GZ_PARTITION, nếu dùng, phải giống nhau giữa Gazebo và PX4. Không tự đặt GZ_IP,
multicast routing, firewall hoặc PX4_NET_INTERFACE để chữa discovery. Các giá
trị kế thừa bất thường được cảnh báo để kiểm tra, không tự sửa.

DISPLAY/WAYLAND_DISPLAY thuộc desktop session Ubuntu. QGC trước đây đã mở được
bằng AppImage bình thường; không mặc định ép QT_QPA_PLATFORM hoặc sửa plugin
paths. Nếu GUI/FUSE lỗi sau reboot, kiểm tra trên host; sandbox không nhìn thấy
/dev/fuse không phải bằng chứng host thiếu FUSE.

## Cổng mạng: instance 0

| Luồng | PX4 local UDP | Đích UDP |
|---|---:|---:|
| GCS Normal | 18570 | QGC 14550, mặc định localhost |
| MAVLink Onboard/API | 14580 | 14540 |
| Payload | 14280 | 14030 |
| Gimbal | 13030 | 13280 |
| uXRCE-DDS | Client socket động | Agent 8888 |

ROS 2 Offboard dùng DDS ở đây, không yêu cầu tạo thêm MAVLink instance 14540.
Không mở TCP 5760 hoặc đổi port chỉ để làm GUI kết nối.
rc.mavlink tăng các local/API/payload/gimbal ports theo instance; launcher in
đúng công thức, không giả định các instance dùng cùng local port.

## Quy trình startup tương lai

1. Kiểm tra binaries/SDF/model resources, cấu hình GUI, processes và UDP ports.
   Có PX4/Gazebo/Agent cần dùng đang chạy hoặc controller thật đang hoạt động:
   dừng new-session workflow, không tự kill hay tái sử dụng.
2. Khởi động đúng một SDF bằng Gazebo không có -r. Xác nhận đúng world,
   paused=true và đúng một model. Không mở một Gazebo world khác từ make.
3. Nếu dùng ROS 2: khởi động một Agent ở port đã kiểm tra. Không cần Agent
   cho native/MAVLink-only demo; dùng launch --without-dds để xem plan tương ứng.
   Tùy chọn này chỉ bỏ bước mở Agent/ROS observer. rcS vẫn khởi động client DDS
   mặc định; không đổi tham số để tắt nó và không tự nhận Agent có sẵn làm partner.
4. Khởi động PX4 standalone để attach đúng model. Nhánh startup này không tạo
   thêm world/model và không đặt lại physics như nhánh spawn.
5. Người dùng mở QGC một lần nếu chưa có. QGC tự nghe UDP 14550 như phiên đã
   kiểm chứng; không tự tạo link khác hoặc thay MAVLink/DDS settings.
6. Chạy READINESS command được launcher in: nó source workspace phù hợp và chạy
   host-check; nếu có DDS thì thêm observer chỉ subscribe trong 3 giây.
7. Tại cold boot đang paused ở time 0, sensors/EKF/preflight có thể chưa chạy
   đủ. Phân loại WAIT_FOR_EXPLICIT_CLOCK_WARMUP, không tự unpause/multi_step,
   không coi việc chỉ chờ wall time là đã khởi tạo EKF.
8. Kiểm tra COM_ARM_ON_BOOT=0 bằng native param show trước khi phê duyệt clock.
   Nếu không đọc được hoặc khác 0, dừng; không tự đổi tham số. Cửa sổ khởi tạo
   có clock tiến và kiểm tra liên lạc phải được duyệt riêng, luôn Disarmed.
9. Sau khi initialized, quay về Paused/Disarmed theo quy trình được duyệt,
   lấy anchor mới và kiểm tra waypoint/collision/frame bằng C2 host-check.
   GO bay cần phê duyệt riêng; launcher không tự chạy follower hoặc PSO flight.

Readiness tự động đã có: native world/pose/iterations, sensors, Commander,
EKF, logger, MAVLink heartbeat snapshot, queue/drop delta, phần mềm/schema và DDS
graph/data khi yêu cầu. Nó không chứng minh heartbeat/telemetry liên tục khi
clock đứng. QGC Comms Lost và queue tích lũy sau Pause cần được phân biệt với
lỗi kết nối khi clock chạy; không tự đổi port để xử lý hiện tượng Pause.

## Tắt hệ thống đúng cách trong phiên tương lai được phê duyệt

Không shutdown khi drone đang bay/armed hoặc follower còn điều khiển. Xác nhận
Landed=True, Armed=False trước. shutdown-plan chỉ in các lệnh kiểm tra và lệnh
PX4 shutdown; nó không thực hiện bất kỳ bước nào.

Trình tự bình thường nếu clock vẫn đang tiến sau khi Land/Disarm:

1. Ghi nhận ULog path và logger status; không xóa hoặc đổi logger mode.
2. Gửi native px4-shutdown --instance 0 từ bin tương ứng, sau phê duyệt shutdown.
3. Chờ PX4 thực sự thoát và log đóng. Exit code của client chỉ xác nhận request,
   không thay thế xác minh PID đã thoát. Không Ctrl+C PX4 để thay cho shutdown
   có logger hooks khi native interface vẫn dùng được.
4. Đóng QGC qua GUI; dừng Agent/observer do phiên này sở hữu tại Terminal của
   chúng bằng thao tác graceful. Không dùng killall/pkill cho tiến trình khác.
5. Đóng Gazebo GUI/server cuối cùng qua GUI/Terminal đã khởi tạo phiên đó.
6. Kiểm tra các PID/ports đã được giải phóng, rồi mới shutdown/reboot Ubuntu
   hoặc chuyển world. Không tự chạy lệnh tắt máy Ubuntu trong launcher.

Nếu hiện đã Paused: **không tự Unpause để tắt PX4**. Source shutdown dùng
work queue/HRT và chờ logger shutdown hook; request có thể bị chờ khi simulation
clock đứng. Nếu chưa có quy trình cleanup được duyệt, giữ nguyên phiên, báo
trở ngại và xin xác nhận cho bước clock/cleanup riêng. Không lặp shutdown,
force-disarm, SIGKILL hoặc kill PID theo trí nhớ để vượt điều kiện này.

## Chuyển world và dữ liệu sau reboot

Kết thúc phiên cũ và xác minh cleanup trước. Tạo profile YAML MỚI cho SDF thật,
kiểm tra inspect và launch dry-run bằng --profile đường_dẫn_yaml. Profile sparse
LUT hiện chỉ là planned: chưa có SDF nên phải báo NO_GO.

Không thay SDF/profile của một phiên đang chạy rồi dùng lại anchor. Mỗi world
mới đều cần geometry/spawn/frame/Start/Goal validation và anchor mới. C1 live
follower vẫn warehouse-specific; không dùng nó cho world khác khi chưa mở rộng
và kiểm thử controller.

Disk giữ: source, SDF, profiles, binaries, CSV/metadata, ULog đã ghi và tham số
PX4 thực sự được save. Reboot mất physics time/pose/velocity, EKF state/origin/
covariance/alignment, readiness, PIDs, sessions, queues/counters.
Không khôi phục iteration 396120 bằng cách đọc CSV hoặc ULog.
Boot ID và process-start identity phải khớp báo cáo MỚI; luôn lấy anchor mới.

## Executor đã triển khai: cách dùng sau khi được duyệt chạy thật

Một Terminal giữ supervisor ở foreground suốt phiên:

    cd ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation
    python3 -B -m pso_flight_demo.session_launcher --profile warehouse start --execute

Thêm --without-dds cho demo native không mở Agent; thêm --timeout 45 để đổi
deadline từng stage (1..120 giây). Đây không phải thời gian chạy physics.
Mỗi probe có timeout ngắn riêng; tổng thời gian có thể dài hơn deadline tối đa
một lần probe đang chờ. Không retry spawn hoặc tự restart child.

Launcher tạo thư mục UUID MỚI trong pso_flight_demo/sessions/ và in đường dẫn
SESSION. Giữ Terminal này mở. Không dùng nohup/service hoặc đóng Terminal để
thay cho shutdown. PID owner/boot ID/start ticks/executable/session ID, profile,
SDF hash, plan, trạng thái và readiness được lưu trong session.json. Child logs
ở gazebo.log, agent.log (nếu bật) và px4.log. Các CSV/anchor/ULog cũ không bị ghi
đè; PX4 tự ghi ULog và rootfs theo startup chuẩn của nó khi thực thi được duyệt.

PX4 dùng -d để không cần pxh stdin; daemon vẫn sống và các native clients trong
bin vẫn dùng được. Gazebo được mở trước, xác minh paused/đúng world/một model,
Agent bind cổng trước PX4, rồi PX4 attach model đã có. QGC vẫn mở thủ công theo
đường dẫn profile; không mở thêm bản nếu đã chạy.

Từ Terminal khác, thay SESSION bằng đúng đường dẫn được in:

    python3 -B -m pso_flight_demo.session_launcher status SESSION
    python3 -B -m pso_flight_demo.session_launcher readiness SESSION

status chỉ đọc state đã lưu (có thể đọc cả sau khi supervisor thoát); readiness
đọc runtime mới. Khi bật DDS, readiness source đúng Jazzy/overlay trong child
shell và mở observer chỉ subscribe 3 giây; không mở controller. Profile domain
được truyền cho child, không thay môi trường shell người dùng. Readiness in
timestamp/flags/mode bits lấy từ VehicleStatus.msg của PX4 root được chọn,
health_report có timestamp thay đổi theo sự kiện; không gán timestamp mới giả.
Chưa kiểm chứng source/build khác nhau hoặc luồng DDS trên cold startup thật.

Nếu rcS/gz_bridge cần mẫu clock đầu tiên trước khi Commander/native clients
sẵn sàng, stage attach có thể timeout ở trạng thái Paused time 0. Khi đó giữ
PX4/Gazebo/Agent, báo CLEANUP_REQUIRED; cần xem px4.log và duyệt warmup/cleanup
riêng. Không có cam kết cold startup sẽ hoàn tất mọi stage mà không tiến clock.

BOOTSTRAPPED_WAIT_CLOCK chỉ có nghĩa đã attach Disarmed, xác minh
COM_ARM_ON_BOOT=0, world còn Paused và các cổng PX4 thuộc child. Các gates
sensors/EKF/preflight/GCS/DDS có thể False hoặc NOT_PROVEN khi time=0. Không
coi đây là GO bay. Launcher không tự chạy clock để làm chúng đạt.

Sau cửa sổ warmup Disarmed đã được duyệt riêng, Pause và lấy anchor mới:

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_px4_ws/install/setup.bash
    python3 -B -m pso_flight_demo.c2 --profile warehouse host-check --prepare-waypoint --observe-dds 3
    python3 -B -m pso_flight_demo.session_launcher check-anchor SESSION BUNDLE_DIRECTORY

check-anchor đối chiếu identity runtime vừa đọc, boot/PX4/Gazebo start ticks,
profile/SDF hash và EKF reset counters. Bundle phiên cũ hoặc world khác bị từ
chối. Nó không khởi động follower hoặc cấp GO live. Cold startup không phục hồi
anchor cũ, không tự xuất PSO hoặc tái dùng tọa độ spawn làm anchor.

## Tắt phiên do executor sở hữu

Xem kế hoạch mặc định, chưa gửi shutdown:

    python3 -B -m pso_flight_demo.session_launcher shutdown SESSION

Sau khi được duyệt tắt thật:

    python3 -B -m pso_flight_demo.session_launcher shutdown SESSION --execute
    python3 -B -m pso_flight_demo.session_launcher status SESSION

Lệnh thứ nhất chỉ xếp request vào supervisor: REQUEST_QUEUED_NOT_PROCESS_EXIT
không có nghĩa đã tắt xong. Supervisor yêu cầu đúng identity, không có controller
ngoài, Landed/Disarmed với timestamp phù hợp, đúng world/model và clock đang
chạy. Sau đó gửi native shutdown đúng một lần; chờ PX4 thực sự thoát qua logger
hooks; mới SIGINT graceful các task Agent và Gazebo thuộc phiên, Gazebo cuối.
Đóng QGC bằng GUI thủ công. Chờ STOPPED trước khi đổi world/reboot.

Nếu đã Pause, trả SHUTDOWN_BLOCKED và giữ nguyên processes/locks. Cần duyệt
riêng cửa sổ cleanup có clock tiến hoặc quy trình xử lý thủ công. Không tự
Unpause, SIGKILL hoặc phát lại native shutdown đã timeout. Ctrl+C/SIGTERM trên
supervisor cũng đi qua gate này, không bỏ qua nó; lúc Pause Terminal có thể vẫn
giữ supervisor để bảo toàn ownership. Không dùng force-kill để giải phóng lock.

Lock flock độc quyền gồm lifecycle toàn hệ thống và PX4 instance, giữ tới khi
supervisor kết thúc; không unlink lock inode. Kiểm tra process/UDP trước mỗi
stage và sau bind: port trống ở precheck chưa đủ, phải thực sự thuộc child.
Các launcher khác cùng module bị lock; tiến trình khởi động thủ công không tuân
lock vẫn được phát hiện qua process/cổng, nhưng không có bảo đảm atomic đối với
chương trình ngoài launcher. Không bind giữ chỗ hoặc đổi port để vượt lỗi.

Rollback trước khi mở PX4: chỉ SIGINT children đã chứng minh ownership, theo
thứ tự ngược. Khi đã mở PX4 hoặc identity không rõ: CLEANUP_REQUIRED, giữ
Gazebo/Agent và lock để xem xét; không gỡ simulator khỏi PX4 đang khởi tạo.
Signal từng task dùng pidfd + kiểm tra boot/start ticks/UID/executable/SID/PGID,
không pkill/killall hoặc signal một process group không kiểm chứng. Child không
thoát graceful: dừng cleanup, giữ log, không force-kill. Child tự detach sang SID
khác không được tự nhận làm child; pre/post checks sẽ chặn khi thấy conflict.

Nếu supervisor bị mất do crash/SIGKILL/reboot: không adopt PID từ file cũ.
Launcher sẽ từ chối shutdown từ manifest có owner cũ và chặn startup nếu còn
PX4/Gazebo/Agent. Cần kiểm tra thủ công được duyệt, không xóa lock/file PID như
cách sửa lỗi. Sau reboot không còn child nhưng session cũ vẫn là dữ liệu lịch sử.

## Kiểm thử host đầu tiên cần phê duyệt riêng

1. Lưu công việc, kết thúc phiên hiện tại theo quy trình được duyệt hoặc reboot
   thủ công. Không chạy executor song song với mô phỏng hiện có.
2. Trên Ubuntu host, chạy inspect và start dry-run --check-host; kiểm tra profile,
   đường dẫn, SDF, cổng và môi trường. Không thay network/parameter để vượt gate.
3. Duyệt startup thật rồi chạy start --execute ở foreground. Xác nhận đúng một
   world/x500/PX4, paused, log paths, COM_ARM_ON_BOOT=0. Terminal thứ hai thử
   start --execute phải bị chặn bởi lock (không tạo child thứ hai).
4. Chạy readiness; chấp nhận WAIT_CLOCK tại cold time 0. Muốn kiểm tra sensors,
   EKF/GCS/DDS có dữ liệu mới, xin duyệt cửa sổ clock Disarmed riêng; chưa bay.
5. Kiểm tra shutdown dry-run. Nếu Pause, execute shutdown phải trả BLOCKED và
   không đổi physics/processes. Chỉ sau phê duyệt cleanup clock riêng mới thử
   shutdown bình thường, kiểm tra STOPPED/ports/logs. Không thử lỗi bằng kill
   các processes của phiên hiện tại.
6. Với world khác: shutdown hoàn tất, chọn YAML/SDF thật và inspect lại; profile
   sparse_lut_planned vẫn bị chặn cho tới khi cấu hình hợp lệ. Phiên và anchor mới
   là bắt buộc. Startup/shutdown runtime và waypoint live vẫn chưa được kiểm thử.

## Kiểm thử offline có thể chạy lại

    PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest discover -s pso_flight_demo/tests -q
    PYTHONDONTWRITEBYTECODE=1 python3 -B -m pso_flight_demo.static_checks

Tests executor dùng child/probe/signal giả; chỉ test flock trên file tạm thuộc
module, không mở Gazebo/PX4/Agent hoặc ROS node thật. Static checks gồm AST,
compile trong bộ nhớ, import/control-command guards và whitespace; chưa phải
type checker hay kiểm thử startup/shutdown/flight trên Ubuntu host.

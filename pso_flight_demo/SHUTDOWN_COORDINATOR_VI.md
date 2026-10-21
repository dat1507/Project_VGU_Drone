# Coordinator shutdown SITL hữu hạn

Coordinator `shutdown_coordinator.py` chạy trên Ubuntu host và chỉ gọi các giao
diện đã có: `session_launcher shutdown-review`, một `gz service` lấy nguyên command
từ proposal hiện hành, `session_launcher shutdown --execute`, rồi
`verify-stopped`. Nó không sửa manifest, không ghi `shutdown.request`, không gọi
PX4 native shutdown, không kill process và không tạo session mới.

Coordinator yêu cầu hai xác nhận tương tác riêng, đúng chuỗi:

1. `APPROVE FINITE CLOCK 5000`
2. `APPROVE SESSION SHUTDOWN`

Sau cả hai xác nhận, nó chạy lại review và so baseline, proposal, safety state,
inventory và identities với review vừa duyệt. Sai khác nào cũng dừng trước clock.
Sau đó gửi đúng một `pause: true, multi_step: 5000`, quan sát iterations và sim time,
đo throughput hiện thời, rồi chỉ gọi `shutdown --execute` một lần khi dự báo còn đủ
headroom cho hai lượt `shutdown_gate` (prequeue trong CLI và recheck của supervisor).
Headroom là 3 lần thời lượng review đo được cộng 1 giây reserve; không giả định
5000 bước luôn đủ. Nếu không đủ, không gọi shutdown và không xin thêm clock window.

Nếu clock không tiến, stats sai/lùi, hết window, shutdown CLI bị từ chối hoặc timeout,
coordinator dừng. Khi clock còn chạy, nó gửi đúng lệnh rollback `pause: true` từ
proposal, không có `multi_step`; rollback ACK không thay thế việc xác minh stats.
Nếu shutdown được queue, coordinator gọi `verify-stopped` tối đa 120 giây. Chỉ
`ALL_CHILDREN_EXITED_START_CANDIDATE` cùng `restart_allowed=true` chứng minh hết
children; nếu không, giữ NO_GO và cần operator review thủ công.

Lệnh chạy thủ công trên Ubuntu host, từ repository:

```bash
PYTHONNOUSERSITE=1 python3 -B -m pso_flight_demo.shutdown_coordinator \
  --session pso_flight_demo/sessions/7ae2da20fe854e098d511d580d1ee335 \
  --expected-iterations 5943682
```

Chỉ chạy sau khi operator đã quyết định riêng cho cả hai thao tác runtime. Lệnh sẽ
hiển thị prompt và yêu cầu nhập từng chuỗi xác nhận; hủy prompt hoặc nhập sai sẽ
không gọi clock/shutdown. Baseline phía trên chỉ là mốc đã báo cáo trong phiên trước;
review mới phải xác nhận lại đúng mốc này, nếu không coordinator dừng. Đây không phải
phê duyệt khởi động session mới.

Không thực hiện live shutdown trong quá trình triển khai. Unit tests chỉ dùng fake
review, stats, service, PX4 safety evidence và process-verification response.

# LUT PSO 3D Recreation — Hướng dẫn thực hành

> **Bản tái hiện phương pháp, KHÔNG phải source code của tác giả LUT.** Viết độc lập từ *Path Planning-Based Drone Delivery System* (Tasnuba Nourin Oyshe, LUT, 2026), chủ yếu §3.1–3.7 và §4.1–4.2. Không tuyên bố trùng kết quả chính xác từng seed. Source 2D cũ nên giữ nguyên trong thư mục khác.

## 1. Thiết lập trên Ubuntu

Giải nén `LUT_PSO_3D_Recreation.zip` vào thư mục dự án, đặt song song với `lut_pso_week1`.

```bash
cd ~/Documents/VGU_Project_Drone/LUT_PSO_3D_Recreation
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

Nếu bạn đã có `.venv` riêng cho project 2D, **đừng xóa hay chuyển** nó; 3D nên dùng `.venv` riêng.

## 2. Chạy thử nhanh (khoảng vài giây)

```bash
python experiment_runner.py --trials 1 --particles 15 --iterations 20 --output results/smoke_local
```

Chế độ này **KHÔNG** có cùng cấu hình với luận văn. Nó chỉ xác nhận chương trình và output hoạt động.

## 3. Chạy thử nghiệm chuẩn theo luận văn

```bash
python experiment_runner.py --trials 10 --particles 100 --iterations 200 --output results/recreation_local
```

Lệnh sử dụng ba môi trường LUT, 2 thuật toán, mỗi tổ hợp 10 lần chạy. Không cần ROS2, PX4, Gazebo hoặc GPU. Trên máy test của nhóm triển khai, toàn bộ thí nghiệm 60 lần tối ưu + vẽ đồ thị hoàn thành trong khoảng 20 giây; máy của bạn có thể chậm hơn.

## 4. File đầu ra và cách đọc

- `config.json`: cấu hình và phiên bản phần mềm.
- `runs.csv`: 60 dòng dữ liệu, một dòng/môi trường/thuật toán/seed.
- `summary.csv`: 6 dòng trung bình ± sample std của 10 trials.
- `thesis_comparison.csv`: đặt trung bình kết quả thực nghiệm cạnh Bảng 1 của LUT.
- `path_sparse_baseline_seed0.png` và tương tự: minh họa đường đi trong 3D của **seed 0**, đường xanh lam đứt đoạn = polyline PSO; đường xanh lục = spline 200 điểm; khối đỏ = vật cản.
- `convergence_sparse_seed0.png` và tương tự: best-so-far fitness của **seed 0**, không phải trung bình 10 trials.
- `convergence_sparse_baseline_seed0.csv`: bảng lịch sử fitness của seed 0.

Các trường CSV quan trọng:

- `raw_collision`: va chạm đường waypoint nối đoạn (đúng phép kiểm tra được dùng bên trong fitness).
- `smoothed_collision`: va chạm theo đoạn thẳng giữa **200 mẫu spline**; phép kiểm tra bổ sung an toàn sau xử lý.
- `smoothed_length`: tổng khoảng cách Euclid dọc theo 200 điểm spline (LUT Eq17).
- `smoothness_rad`: tổng góc đổi hướng giữa các đoạn liên tiếp trên spline (LUT Eq18-19).
- `convergence_iteration`: lần đầu gbest đã đạt fitness bằng với gbest cuối cùng (LUT Eq21).
- `collision_rate_pct`: tổng raw_collision/10 × 100%, tương ứng metric của luận văn (LUT Eq20).
- `spline_collision_rate_pct`: phép thử riêng của nhóm: tổng smoothed_collision/10 × 100%.
- `feasible_smoothed_length_mean`: trung bình chiều dài **chỉ trên các đường waypoint khả thi**, để tránh đánh đồng đường ngắn nhưng xuyên vật cản với nghiệm tốt.

**Lưu ý:** Summary `length_mean`, `smoothness_mean_rad` tính trên **mọi lần chạy**, kể cả đường va chạm, để so khớp lối báo cáo ở Bảng 1 của luận văn. Dùng `feasible_smoothed_length_mean` nếu muốn đánh giá chất lượng đường bay sử dụng được.

## 5. Mỗi file .py tương ứng phần nào trong thesis?

| File | Luận văn | Vai trò |
|---|---|---|
| `environment.py` | §3.2 | Lập phương AABB, slab collision + kiểm tra giới hạn |
| `scenarios.py` | §4.1.1 | Sparse/Moderate/Dense, tọa độ gốc |
| `pso_core.py` | §3.3–3.5 | Mã nghiệm 45 chiều, fitness 10000, PSO chung |
| `baseline_pso.py` | §3.4 | Gọi PSO với w=0.7 |
| `pso.py` | §3.5 | Gọi PSO với w(t)=0.9−0.5t/200 |
| `path_smoother.py` | §3.6–3.7 | Cubic spline 200 mẫu, length, turn-angle sum |
| `experiment_runner.py` | §4.1 | Chạy lô, lưu lịch sử, vẽ đường bay 3D |
| `data_processor.py` | §3.1 | Trung bình, độ lệch chuẩn, tỷ lệ va chạm |
| `analysis.py` | §4.2 | 3D plots, convergence, bảng so sánh LUT |
| `tests/test_environment.py` | kiểm thử bổ sung | Test hình học, fitness, khởi tạo, convergence, spline |

## 6. Những lựa chọn kỹ thuật KHÔNG được luận văn nói đủ rõ

**(1) Velocity ban đầu:** đặt zero; bản gốc không công bố cụ thể.  
**(2) Bộ sinh số ngẫu nhiên:** NumPy `default_rng(seed)`; tác giả không công bố seed hay trình tự tạo số.  
**(3) Vòng lặp inertia:** t từ 0 đến 199, trực tiếp theo Eq14 có T_max=200; giá trị dùng ở vòng cập nhật cuối = 0.4025 (hệ số tại t=200 mới chính xác bằng 0.4).  
**(4) Chỉnh sửa spline để chạy được:** bản viết nói lặp start/end khi tính chord-length, nhưng cách đó tạo knot lặp và `make_interp_spline` báo lỗi; bản này giữ các waypoint duy nhất liên tiếp, không thêm bản sao điểm cuối vào knot. Đây là diễn giải chạy được của Eq16, chưa xác nhận trùng code gốc.  
**(5) 'Catmull-Rom' so với `make_interp_spline`:** tên và biểu thức của LUT không hoàn toàn tương đương. Mặc định dùng `--smoothing thesis_spline` (SciPy clamped cubic). Tùy chọn `--smoothing catmull_rom` chỉ là thí nghiệm so sánh riêng, không mặc nhận tác giả dùng cách này.  
**(6) Drone geometry:** thesis tính đường hình học như điểm, không có bán kính drone; bản recreation cũng không tự thêm biên an toàn.  
**(7) Đường cong sau smoothing:** luận văn xác định collision rate bằng fitness của **polyline tối ưu**. Bản này tính thêm collision của spline; việc nó có thể xuyên vật cản chính là vấn đề an toàn cần báo cáo.

## 7. Hướng phân tích giới hạn

Trước tiên hãy đọc `results/recreation_local/thesis_comparison.csv`. Kiểm tra:

1. Moderate baseline/enhanced có còn thất bại 100% như Bảng 1 của LUT hay không?
2. Dense enhanced tốt hơn baseline bao nhiêu về tỷ lệ khả thi?
3. Có bao nhiêu nghiệm raw an toàn nhưng sau smoothing lại không an toàn?
4. Thành phần penalty 10000 (nhị phân) có làm nhiều particle không thể cải thiện khi tất cả đường đều va chạm hay không?
5. Có khác biệt gì nếu thay đổi **một** yếu tố ngoài tham số chính ở thí nghiệm *riêng*, chẳng hạn mức nhiễu khởi tạo?

Tập tin `LIMITATIONS_AND_IDEAS.md` cung cấp kế hoạch controlled experiments trước khi thiết kế Adaptive PSO mới. **Không biến các thí nghiệm mở rộng thành kết quả LUT gốc.**

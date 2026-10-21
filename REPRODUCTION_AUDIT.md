# LUT Thesis Recreation — Bảng xác minh nguồn gốc

**Luận văn:** Tasnuba Nourin Oyshe, *Path Planning-Based Drone Delivery System*, LUT, 2026, trang 20–28. Các file `.py` ở đây được tự viết theo phương pháp đã công bố, **không phải mã nguồn chính thức**.

| Mục | Nguồn gốc LUT | Triển khai | Đánh giá |
|---|---|---|---|
| Không gian [0,100]^3 | Tr.20 | `environment.py` | Phù hợp |
| AABB cubes center ± side/2 | Tr.20–21 Eq3 | `environment.py:Cube` | Phù hợp |
| Slab method segment–AABB | Tr.21 Eq4–7 | `environment.py` | Phù hợp; xử lý cả đoạn song song |
| Path 15 intermediate 3D, 45-dimensional vector | Tr.21 Eq8 | `pso_core.py` | Phù hợp |
| Fitness raw path length+binary 10000 | Tr.22 Eq9–11 | `pso_core.py:evaluate` | Phù hợp |
| w=0.7, c1=c2=2 | Tr.22–23 Eq12–13 | `pso_core.py` | Phù hợp |
| Enhanced w(t)=0.9−(0.5)t/200 | Tr.23 Eq14 | `pso_core.py:inertia_at` | Phù hợp theo chỉ số t=0...199; indexing không nêu rõ |
| 100 particles, 200 updates | Tr.27–28 | `experiment_runner.py` defaults | Phù hợp |
| Uniform noise in [-2.5,2.5] around diagonal | Tr.22 | `pso_core.py` | Phù hợp về phân phối; RNG/seeds chưa biết |
| Position clamping [0,100] | Tr.23 | `pso_core.py` | Phù hợp |
| Smoothing cubic spline `make_interp_spline` | Tr.24 Eq15–16 | `path_smoother.py` | **Diễn giải có điều chỉnh, xem README** |
| 200 spline samples | Tr.24,26 | `smooth_path(...,samples=200)` | Phù hợp |
| Final smoothed length | Tr.26 Eq17 | `path_length` | Phù hợp |
| Total turn angle (radians) | Tr.26 Eq18–19 | `turn_angle_sum` | Phù hợp |
| Collision threshold from final raw gbest fitness | Tr.26 Eq20 | `raw_collision` direct geometric check | Phù hợp trong phạm vi penalty của nghiên cứu |
| First iteration final best reached | Tr.26 Eq21 | `convergence_iteration` | Phù hợp |
| 3 cases with exact obstacles/start/end | Tr.27 | `scenarios.py` | Phù hợp |
| 10 independent repetitions | Tr.28 | `--trials 10` | Phù hợp |
| SciPy/NumPy versions, PSO velocity init, RNG implementation | Không nêu trong nguồn | Phiên bản ghi `config.json`; v=0 và `default_rng` | **Không thể khôi phục chính xác** |

## Vấn đề diễn giải smoothing

- Luận văn gọi Catmull–Rom nhưng dùng `make_interp_spline`, là hàm cubic spline interpolating.
- Mô tả pad duplicate start/end + chordal u là không áp dụng trực tiếp được bằng SciPy vì các điểm lặp tạo knot trùng.
- Mặc định ở đây dùng `make_interp_spline` với clamped boundary sau khi xóa waypoint trùng, không pad nút trùng. Đây là **diễn giải thực thi được** gần nhất với Eq16, nhưng không được nhận là phiên bản tác giả nếu chưa có mã gốc.
- `--smoothing catmull_rom` được cung cấp chỉ để thí nghiệm phân tích độ nhạy.

## Các phát biểu có thể sử dụng trong báo cáo

- 'Independent method-level recreation of the LUT 3D PSO framework' — **đúng**.
- 'Parameters and scenarios were transcribed from the thesis' — **đúng, với các ngoại lệ ghi rõ**.
- 'Exact reproduction of the author's original software' — **không đúng**.
- 'Identical statistical results to original LUT' — **chưa thể khẳng định**.

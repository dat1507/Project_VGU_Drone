# Hạn chế đã quan sát và giả thuyết cải tiến (không được lẫn với LUT gốc)

## Kết quả chạy minh họa của bộ recreation

Thử nghiệm được chạy ngày 2026-10-11 trong môi trường Python ở đây với đúng 3 scenarios × 2 algorithms × 10 seeds × 100 particles × 200 iterations. Không phải kết quả trên máy Ubuntu của người dùng. Xem `results/recreation/summary.csv` để có số gốc.

| Case | Algorithm | Raw collision | Collision sau spline |
|---|---|---:|---:|
| Sparse | Baseline | 0% | 30% |
| Sparse | Enhanced | 0% | 40% |
| Moderate | Baseline | 100% | 100% |
| Moderate | Enhanced | 100% | 100% |
| Dense | Baseline | 70% | 70% |
| Dense | Enhanced | 10% | 60% |

**Kết luận 1 — Trung tâm Moderate:** Mọi lần chạy ví dụ ở Moderate đều va chạm như luận văn (Table 1, tr.28). Kết quả là phù hợp về hiện tượng, không chứng minh tất cả tham số hoặc mã là giống nhau. Bẫy khởi tạo quanh đường chéo được tác giả phân tích trên tr.31.

**Kết luận 2 — Va chạm do spline:** Ở Dense/Enhanced, raw collision 10% nhưng smoothed collision 60% trong bài thử này. Đây là kiểm tra **bổ sung** mà luận văn không dùng cho Collision Rate trong Table 1. Một đường waypoint an toàn không đảm bảo spline an toàn. Các đoạn nối 200 mẫu spline đã được kiểm tra theo AABB; khoảng trống giữa các mẫu cực nhỏ vẫn chưa được chứng minh an toàn liên tục một cách hình thức.

**Kết luận 3 — Penalty nhị phân:** Trong vùng mà tất cả candidate đều va chạm, penalty luôn là 10000, không phân biệt mức xuyên vật cản. Khi đó PSO tối ưu chiều dài ngay cả khi nghiệm vô hiệu. Đây là một tính chất của objective LUT Eq11 và là giả thuyết cơ chế dẫn đến stagnation; chưa phải bằng chứng quan hệ nhân quả độc lập.

## Đề xuất cải tiến cần KIỂM ĐỊNH, chưa triển khai Adaptive PSO

A. **Feasible corridor initialization**: thay thế một phần quần thể quanh đường chéo bằng các waypoint khởi tạo tránh AABB, có thể từ A*/RRT* 3D hoặc phương pháp heuristic. Đo collision rate Moderate khi GIỮ NGUYÊN inertia/fitness/iterations. Nếu cải thiện, xem xét chuyển thành thao tác initialization trong Adaptive PSO.

B. **Continuous graded constraint penalty**: thay penalty 10000 đồng nhất bằng hàm phân biệt độ xuyên vật cản, ví dụ distance-to-box hoặc chiều dài đoạn đi trong vật cản, có thể kết hợp feasibility-first selection. Giữ nguyên các yếu tố khác. Giả thuyết: tạo gradient đánh giá thứ hạng giữa các nghiệm va chạm.

C. **Collision-aware smoothing / repair**: kiểm tra đoạn spline và sửa/điều chỉnh đường cong cho đến khi vẫn tránh AABB. Smoothing phải được đánh giá lại cả length và turn-angle; không tự coi nó là kết quả gốc LUT.

D. **State-aware inertia**: điều khiển w dựa trên swarm diversity, improvement stagnation và tỷ lệ feasible (sau khi baseline đã tái hiện). Đây mới là giả thuyết *Adaptive PSO mới* khác với lịch w(t) giảm tuyến tính đã có trong LUT.

## Thiết kế thử nghiệm có kiểm soát

- **Gốc**: lệnh chuẩn `python experiment_runner.py --trials 10 --particles 100 --iterations 200 --output results/recreation_local`.
- **Thử sensitivity mức jitter** (KHÔNG phải LUT gốc):

```bash
python experiment_runner.py --scenario moderate --trials 10 --particles 100 --iterations 200 --jitter 10 --output results/sensitivity_jitter10
```

- **Thử lựa chọn spline:**

```bash
python experiment_runner.py --trials 10 --particles 100 --iterations 200 --smoothing catmull_rom --output results/sensitivity_catmull
```

Trong mọi so sánh, sử dụng 10 seed giống nhau, máy/phiên bản thư viện giống nhau, cùng computational budget và cùng tiêu chí. Chỉ thay đổi một yếu tố mỗi lần. Phân biệt các đường gấp khúc an toàn với đường spline sau làm mượt.

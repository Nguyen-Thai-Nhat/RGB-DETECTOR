# GIAO TIẾP I2C PROTOCOL VỚI CẢM BIẾN MÀU SẮC RGB  - RGB DETECTOR
* I2C INTERFACE: Thực hiện giao thức đọc dữ liệu từ cảm biến màu sắc qua giao thức I2C.
* HANDLING COLOR: Nhận giá trị từ RGB CONTROLLER và quy đổi giá trị đó thành màu tương ứng.
* RGB CONTROLLER: Máy trạng thái điều khiển ghi và đọc dữ liệu từ cảm biến màu sắc.
* CYCLONE10_PLL.v: File Verilog chính chứa mô tả cấu trúc module PLL, gồm định nghĩa module và cách kết nối tín hiệu.
* CYCLONE10_PLL.qip: File cấu hình IP để Quartus biết project sử dụng IP này. Nó bao gồm thông tin đường dẫn, constraint,...

## Lưu ý

Source code này đã bao gồm cả CYCLONE10_PLL

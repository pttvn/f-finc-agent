# f-FINC Agent

## Hướng dẫn sử dụng

Demo: https://pttvn.github.io/f-finc-agent

### Biên dịch và nạp code cho ESP32

1. Mở file `src/esp32/f-finc-agent/f-finc-agent.ino` bằng Arduino IDE
2. Chọn board "ESP32 Dev Module" trong menu Tools > Board
3. Chọn Partitions Scheme "Huge APP (3MB No OTA/1MB SPIFFS)" trong menu Tools > Partition Scheme
4. Kết nối ESP32 với máy tính qua cổng USB
5. Chọn cổng COM phù hợp trong menu Tools > Port
6. Nhấn nút Upload để biên dịch và nạp code

### Chỉnh sửa giao diện web

1. Vào thư mục `src/esp32/f-finc-agent/www`
2. Chỉnh sửa file `webpage.html` theo nhu cầu
3. Sau khi chỉnh sửa xong, chạy file `convert.bat` để tạo file `webpage.ino`
4. Biên dịch và nạp lại code cho ESP32 như hướng dẫn ở trên

### Build firmware và cập nhật lên GitHub

1. Mở dự án trong Arduino IDE
2. Vào menu Sketch > Export Compiled Binary để tạo file .bin
3. Copy file .bin vừa tạo vào thư mục `firmware`
4. Chỉnh sửa version trong file `firmware/manifest.json`
5. Commit và push các thay đổi lên GitHub

## Lưu ý

- Đảm bảo đã cài đặt thư viện cần thiết cho ESP32
- Kiểm tra kỹ cấu hình mạng trước khi nạp code
- Sau khi nạp code, có thể truy cập giao diện web qua địa chỉ IP của ESP32

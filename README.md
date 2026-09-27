# BMS Data Acquisition and Monitoring

## Mục tiêu 

Dự án thu thập và giám sát dữ liệu từ bộ pin 13S2P với điện áp danh định khoảng 48V

STM32F1 gửi request frame và nhận dữ liệu phản hồi từ BMS qua giao thức UART, sau đó giải mã và truyền các thông số thông qua giao thức CAN bus tới ESP32S3 để đưa dữ liệu lên FireBase để giám sát từ xa

## Kiến trúc hệ thống
Pack 13S5P + BMS
→ UART JBD
→ STM32F1
→ CAN
→ ESP32-S3
→ Firebase


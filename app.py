from flask import Flask, render_template, request, jsonify
import socket

app = Flask(__name__)

# ================= CẤU HÌNH MẠNG =================
# TODO: Thay địa chỉ IP này bằng IP hiển thị trên màn hình Serial của Arduino IDE
ESP32_IP = "DIA_CHI_ESP32" 
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_command', methods=['POST'])
def send_command():
    data = request.json
    action = data.get('action')
    
    # Gán trực tiếp tốc độ cho Động cơ Trái và Phải
    motor_left, motor_right = 0.0, 0.0
    
    if action == 'forward': 
        motor_left, motor_right = 0.6, 0.6    # 2 động cơ xoay ngược chiều nhau trong không gian để cùng Tiến
    elif action == 'backward': 
        motor_left, motor_right = -0.6, -0.6  # 2 động cơ xoay ngược chiều nhau trong không gian để cùng Lùi
    elif action == 'left': 
        motor_left, motor_right = 0.6, -0.6   # Đảo chiều để Rẽ Trái đúng hướng
    elif action == 'right': 
        motor_left, motor_right = -0.6, 0.6   # Đảo chiều để Rẽ Phải đúng hướng
    elif action == 'stop': 
        motor_left, motor_right = 0.0, 0.0    # 2 động cơ dừng hẳn
        
    print(f"-> Gửi xuống xe: Trái={motor_left}, Phải={motor_right}")
    
    # Định dạng gói tin gửi xuống ESP32
    message = f"{motor_left:.1f},{motor_right:.1f}\n"
    try:
        sock.sendto(message.encode('utf-8'), (ESP32_IP, UDP_PORT))
    except Exception as e:
        print("Lỗi gửi dữ liệu:", e)
        return jsonify({"status": "error", "message": str(e)}), 500

    return jsonify({"status": "success", "left": motor_left, "right": motor_right})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
#!/bin/bash
set -e

# 1. Ortam Değişkenlerini ve Kaynakları Yükle
source /opt/ros/humble/setup.bash
source /app/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export DISPLAY=:0

echo "🚀 Docker Başlatılıyor..."

# 2. Gazebo'yu Başlat (Senin verdiğin komut)
echo "🏠 Gazebo Başlatılıyor (empty_world)..."
ros2 launch turtlebot3_gazebo empty_world.launch.py &
PID_GAZEBO=$!
sleep 10 # Gazebo'nun kendine gelmesi için bekle

# 3. Navigasyonu Başlat
# Harita yolunu Docker içindeki konuma (/app/maps/...) göre ayarladık
echo "🗺️ Nav2 Başlatılıyor (kucuk_ev_haritasi)..."
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/app/maps/kucuk_ev_haritasi.yaml use_sim_time:=True &
PID_NAV=$!
sleep 8 # Navigasyonun yüklenmesi için bekle

# 4. Senin Python Kodunu Çalıştır
echo "🧹 Temizlik Robotu Kodu Çalıştırılıyor..."
python3 /app/temizlik_robotu.py

# 5. İşlem bitince hemen kapanmasın, Gazebo kapanana kadar bekle
wait $PID_GAZEBO

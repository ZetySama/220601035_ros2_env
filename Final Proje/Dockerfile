# 1. Temel İmaj: ROS 2 Humble (Gazebo dahil full sürüm)
FROM osrf/ros:humble-desktop-full

# 2. Çalışma dizini oluştur
WORKDIR /app

# 3. Gerekli paketleri yükle
# Turtlebot3 ve Navigasyon paketlerini kuruyoruz
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3* \
    ros-humble-navigation2 \
    ros-humble-nav2-simple-commander \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 4. Dosyaları Kopyala
# src klasöründeki paketlerini içeri al
COPY src /app/src
# Harita klasörünü içeri al
COPY maps /app/maps
# Python kodunu içeri al
COPY temizlik_robotu.py /app/temizlik_robotu.py
# Başlatma scriptini içeri al
COPY entrypoint.sh /app/

# 5. Projeyi Derle (Build)
# src içinde Turtlebot paketleri olduğu için build şart
RUN . /opt/ros/humble/setup.sh && colcon build

# 6. Ortam değişkenlerini ayarla
ENV TURTLEBOT3_MODEL=waffle_pi
# Her bash açılışında kaynakları otomatik tanıt
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /app/install/setup.bash" >> ~/.bashrc

CMD ["/app/entrypoint.sh"]

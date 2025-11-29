# [cite_start]1. Taban imaji: Ros Humble'in minimum kurulumu [cite: 71]
FROM ros:humble-ros-base

# 2. calisma dizinini /ws olarak ayarla
WORKDIR /ws

# 3. Gerekli build araclarini kur (GÜNCELLENDİ: ros-dev-tools ve digerleri)
# Bu paketler, ROS 2 paketlerini derlemek için gereken ament_cmake, rosidl ve diğer araçları içerir.
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    python3-rosdep \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-dev-tools

# [cite_start]4. Kaynak kodlarini /ws/src icine kopyala [cite: 72]
COPY src src/

# 5. Launch dosyasini kopyala
COPY launch launch/

# 6. Entrypoint script'ini kopyala
COPY entrypoint.sh .

# [cite_start]7. Kodlari Konteyner Icinde Derle (GÜNCELLENDİ: source komutu eklendi) [cite: 73]
# Bu satir, derleme oncesinde ROS 2 ortam degiskenlerini yukler.
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# 8. Entrypoint'i calistirilabilir yap
RUN chmod +x entrypoint.sh

# [cite_start]9. Konteyner basladiginda entrypoint.sh'yi calistir [cite: 74]
ENTRYPOINT ["/ws/entrypoint.sh"]

# BYM412 ROBOTİK DERSİ - ÖDEV 3

Proje Tanımı
Bu proje, İstanbul Sağlık ve Teknoloji Üniversitesi BYM412 Robotik Dersi (Güz Dönemi) kapsamında geliştirilmiştir. Projenin amacı, ROS 2 Humble tabanlı dağıtık bir sensör-işlemci-sunucu mimarisini **Docker** teknolojisi kullanarak konteynerize etmek ve izole bir ortamda çalıştırmaktır.

Proje Yapısı
Proje, ROS 2 standartlarına uygun olarak aşağıdaki dizin yapısına sahiptir:

- `Dockerfile`: Projenin Docker imajını oluşturmak için gerekli talimatları içerir.
- `entrypoint.sh`: Konteyner başlatıldığında ROS 2 ortamını ve projeyi otomatik olarak ayağa kaldıran betiktir.
- `launch/`: Tüm düğümleri tek komutla başlatan `my_project.launch.py` dosyasını içerir.
- `src/`: ROS 2 paketlerinin kaynak kodları buradadır.
  - `sensor_publisher_pkg`: Sensör verisi üretir.
  - `data_processor_pkg`: Veriyi işler.
  - `command_server_pkg`: Karar mekanizması (servis) sunar.
  - `my_robot_interfaces`: Özel servis mesajlarını (`.srv`) tanımlar.
- `SSF_HASH.txt`: Sistem doğrulama (Student System Fingerprint) imzasını içerir.




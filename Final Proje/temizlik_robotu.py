import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    print("Nav2 başlatılıyor, lütfen bekleyin...")
    navigator.waitUntilNav2Active()
    print("Nav2 aktif! Temizlik planı oluşturuluyor...")

    # --- ODALARIN KOORDİNATLARI ---
    rooms = [
        # 1. Oda (Giriş / Merkez Bölge)
        {'name': "Giris/Merkez", 'min_x': -0.389, 'max_x': 1.84, 'min_y': -0.477, 'max_y': 1.31},

        # 2. Oda (Sağ Orta Bölge)
        {'name': "Sag Orta Oda", 'min_x': 3.51, 'max_x': 5.84, 'min_y': 2.33, 'max_y': 4.25},
        
        # 3. Oda (Sol Orta Bölge)
        {'name': "Sol Orta Oda", 'min_x': -3.1, 'max_x': 0.044, 'min_y': 3.84, 'max_y': 6.56},

        # 4. Oda (Sağ Üst Bölge)
        {'name': "Sag Ust Oda", 'min_x': 4.41, 'max_x': 5.78, 'min_y': 6.13, 'max_y': 8.61},

        # 5. Oda (Sol Üst Bölge)
        {'name': "Sol Ust Oda", 'min_x': -1.58, 'max_x': 0.128, 'min_y': 8.47, 'max_y': 10.6},
    ]

    # Robotun süpürme genişliği (Şerit aralığı)
    step_size = 0.35 

    full_cleaning_path = []

    # Her oda için zikzak rotaları hesapla ve birleştir
    for room in rooms:
        print(f"Rota hesaplanıyor: {room['name']}...")
        room_path = generate_zigzag_path(
            navigator, 
            room['min_x'], room['max_x'], 
            room['min_y'], room['max_y'], 
            step_size
        )
        full_cleaning_path.extend(room_path)

    # --- TEMİZLİK BAŞLIYOR ---
    print(f"Toplam {len(full_cleaning_path)} temizlik noktası (Waypoint) oluşturuldu.")
    print("Robot temizliğe başlıyor...")
    
    # Nav2'ye hesaplanan tüm noktaları sırayla gitmesi için veriyoruz
    navigator.followWaypoints(full_cleaning_path)

    # İşlem bitene kadar bekle ve bilgi ver
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0: # Çok sık yazdırmamak için
            print(f"Hedefe gidiliyor... Kalan nokta sayısı: {len(full_cleaning_path) - feedback.current_waypoint}")
            
    # Sonuç Kontrolü
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('GÖREV BAŞARILI: Tüm ev tertemiz oldu!')
    elif result == TaskResult.CANCELED:
        print('GÖREV İPTAL EDİLDİ.')
    elif result == TaskResult.FAILED:
        print('GÖREV BAŞARISIZ! Robot bir yere takılmış olabilir.')

    navigator.lifecycleShutdown()

# --- ZİKZAK (BOUSTROPHEDON) ALGORİTMASI ---
def generate_zigzag_path(navigator, min_x, max_x, min_y, max_y, step):
    poses = []
    
    # Güvenlik payı: Duvarın tam dibine girmesin diye kutuyu 10cm daraltıyoruz
    safe_margin = 0.1
    start_x = min_x + safe_margin
    end_x = max_x - safe_margin
    start_y = min_y + safe_margin
    end_y = max_y - safe_margin

    current_x = start_x
    move_up = True 

    # X ekseni boyunca kayarak ilerle
    while current_x <= end_x:
        if move_up:
            # Aşağıdan Yukarıya git
            poses.append(create_pose(navigator, current_x, start_y))
            poses.append(create_pose(navigator, current_x, end_y))
        else:
            # Yukarıdan Aşağıya git
            poses.append(create_pose(navigator, current_x, end_y))
            poses.append(create_pose(navigator, current_x, start_y))

        # Yönü değiştir ve yana kay
        move_up = not move_up
        current_x += step
        
    return poses

# Yardımcı Fonksiyon: Koordinattan Pose Mesajı Üretir
def create_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    # Robotun yönünü (Orientation) sabit tutuyoruz, Nav2 kendi dönecektir
    pose.pose.orientation.w = 1.0 
    return pose

if __name__ == '__main__':
    main()

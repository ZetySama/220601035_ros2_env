import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        # /camera/camera_sensor/image_raw topic'ine abone ol
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera_sensor/image_raw', # RViz'de gördüğümüz gerçek topic adı
            self.image_callback,
            10)
        self.subscription  # unused variable warning'i önle

        # /cmd_vel topic'ine yayın yap
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # CvBridge'i başlat
        self.bridge = CvBridge()
        
        self.get_logger().info('Ball Chaser Node is ready.')

    def image_callback(self, msg):
        # --- HATA AYIKLAMA ---
        # self.get_logger().info('Image received!') 
        # --------------------
        
        try:
            # ROS Görüntü mesajını OpenCV görüntüsüne çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Görüntüyü HSV renk uzayına çevir
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Beyaz/Gri yerine KIRMIZI renk için HSV aralıklarını belirle (GÜNCELLENDİ)
        # Kırmızı, HSV çemberinde 0 ve 180'e yakındır, bu yüzden iki ayrı aralık gerekir.
        # İlk aralık (0 derece civarı)
        lower_red_1 = np.array([0, 70, 50])
        upper_red_1 = np.array([10, 255, 255])
        
        # İkinci aralık (180 derece civarı)
        lower_red_2 = np.array([170, 70, 50])
        upper_red_2 = np.array([180, 255, 255])
        
        # İki maskeyi oluştur
        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        
        # Final maskeyi oluşturmak için maskeleri birleştir
        mask = mask1 + mask2

        # Görüntünün genişliğini al
        h, w, d = cv_image.shape
        
        # Maskenin momentlerini (ağırlık merkezini) hesapla
        M = cv2.moments(mask) 
        
        twist_msg = Twist()
        
        # --- HATA AYIKLAMA ---
        # self.get_logger().info(f"Calculated M['m00']: {M['m00']}") 
        # --------------------

        if M['m00'] > 0:
            # Top bulundu demektir. Ağırlık merkezini (cx) hesapla.
            cx = int(M['m10'] / M['m00'])
            
            # Görüntünün merkez noktasını bul
            center_x = w // 2
            
            # Hatayı hesapla (merkezden ne kadar saptı)
            error = cx - center_x
            
            # --- Kontrol Mantığı ---
            
            # Top yaklaşık merkezde ise ileri git
            if abs(error) < 50:
                twist_msg.linear.x = 0.2  # İleri hız
                twist_msg.angular.z = 0.0 # Dönme
                # --- HATA AYIKLAMA ---
                self.get_logger().info(f"Ball FOUND, Moving FORWARD. cx={cx}") 
                # --------------------
            # Top merkezde değilse, merkeze almak için dön
            else:
                twist_msg.linear.x = 0.0 # Dur
                # Hata ne kadar büyükse o kadar hızlı dön
                # -0.005 katsayısı dönüş hızını ayarlar
                twist_msg.angular.z = -0.005 * float(error)
                # --- HATA AYIKLAMA ---
                direction = "RIGHT" if error > 0 else "LEFT"
                self.get_logger().info(f"Ball FOUND, Turning {direction}. error={error}")
                # --------------------
        else:
            # Top görünmüyorsa dur
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            # --- HATA AYIKLAMA ---
            self.get_logger().info("Ball NOT FOUND, Stopped.")
            # --------------------

        # --- HATA AYIKLAMA ---
        # self.get_logger().info(f"Publishing Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")
        # --------------------
        self.publisher_.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    ball_chaser = BallChaser()
    rclpy.spin(ball_chaser)
    
    # Düğüm düzgün kapatıldıktan sonra kaynakları serbest bırak
    ball_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

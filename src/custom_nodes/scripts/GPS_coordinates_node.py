import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from math import radians, cos, sin, sqrt, atan2

# ==========================
#  Global Config
# ==========================

PRINT_INFO = True

rows = {
    "section1 (solar opaque)": {
        "row1": {"N": {"lat": 41.2879572, "lon": 2.0439164}, "S": {"lat": 41.2877921, "lon": 2.0439758}},
        "row2": {"N": {"lat": 41.2879437, "lon": 2.0439143}, "S": {"lat": 41.2877799, "lon": 2.0439708}},
        "row3": {"N": {"lat": 41.2879345, "lon": 2.0438966}, "S": {"lat": 41.2877724, "lon": 2.0439547}},
        "row4": {"N": {"lat": 41.28793419, "lon": 2.0438513}, "S": {"lat": 41.2877693, "lon": 2.0439135}},
        "row5": {"N": {"lat": 41.2879326, "lon": 2.043848}, "S": {"lat": 41.2877606, "lon": 2.0439057}},
        "row6": {"N": {"lat": 41.2879206, "lon": 2.0438277}, "S": {"lat": 41.2877566, "lon": 2.0438868}},
        "row7": {"N": {"lat": 41.287925, "lon": 2.0437833}, "S": {"lat": 41.2877553, "lon": 2.0438448}},
        "row8": {"N": {"lat": 41.2879138, "lon": 2.0437784}, "S": {"lat": 41.2877487, "lon": 2.0438396}},
        "row9": {"N": {"lat": 41.287911, "lon": 2.0437601}, "S": {"lat": 41.287743, "lon": 2.0438177}},
        "row10": {"N": {"lat": 41.2879049, "lon": 2.0437224}, "S": {"lat": 41.2877473, "lon": 2.0437729}},
        "row11": {"N": {"lat": 41.2879027, "lon": 2.043703}, "S": {"lat": 41.2877455, "lon": 2.04376}},
        "row12": {"N": {"lat": 41.2878918, "lon": 2.0436927}, "S": {"lat": 41.2877346, "lon": 2.0437483}},
    },
    "section2 (solar semi)" : {
        "row1": {"N": {"lat": 41.2880032, "lon": 2.0441778}, "S": {"lat": 41.2878395, "lon": 2.0442372}},
        "row2": {"N": {"lat": 41.2879897, "lon": 2.0441751}, "S": {"lat": 41.2878287, "lon": 2.0442331}},
        "row3": {"N": {"lat": 41.2879979, "lon": 2.0441341}, "S": {"lat": 41.2878326, "lon": 2.0441905}},
        "row4": {"N": {"lat": 41.2879852, "lon": 2.044132}, "S": {"lat": 41.2878237, "lon": 2.0441887}},
        "row5": {"N": {"lat": 41.287977, "lon": 2.044091}, "S": {"lat": 41.2878255, "lon": 2.0441437}},
        "row6": {"N": {"lat": 41.2879554, "lon": 2.0440717}, "S": {"lat": 41.2878069, "lon": 2.0441368}},
        "row7": {"N": {"lat": 41.2879721, "lon": 2.0440274}, "S": {"lat": 41.2878143, "lon": 2.0440926}},
        "row8": {"N": {"lat": 41.2879621, "lon": 2.0440309}, "S": {"lat": 41.2878082, "lon": 2.0440871}},
    },
    "open air": {
        "row1": {"N": {"lat": 41.28805769, "lon": 2.0444263}, "S": {"lat": 41.2878894, "lon": 2.0444874}},
        "row2": {"N": {"lat": 41.2880423, "lon": 2.0444298}, "S": {"lat": 41.2878853, "lon": 2.0444864}},
        "row3": {"N": {"lat": 41.28804729, "lon": 2.0443928}, "S": {"lat": 41.287886, "lon": 2.0444507}},
        "row4": {"N": {"lat": 41.2880359, "lon": 2.0443902}, "S": {"lat": 41.2878828, "lon": 2.044441}},
        "row5": {"N": {"lat": 41.2880355, "lon": 2.0443539}, "S": {"lat": 41.2878766, "lon": 2.0444122}},
        "row6": {"N": {"lat": 41.28802479, "lon": 2.044348}, "S": {"lat": 41.2878632, "lon": 2.0444043}},
        "row7": {"N": {"lat": 41.28803339, "lon": 2.044314}, "S": {"lat": 41.2878669, "lon": 2.0443698}},
        "row8": {"N": {"lat": 41.2880161, "lon": 2.0443051}, "S": {"lat": 41.2878506, "lon": 2.0443658}},
    }
}

# ==========================
# Helper functions
# ==========================

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

def gps_to_local(lat_ref, lon_ref, lat, lon):
    x = haversine(lat_ref, lon_ref, lat_ref, lon) * (1 if lon > lon_ref else -1)
    y = haversine(lat_ref, lon_ref, lat, lon_ref) * (1 if lat > lat_ref else -1)
    return x, y

def project_point_on_row(lat, lon, N, S):
    Nx, Ny = 0.0, 0.0
    Sx, Sy = gps_to_local(N["lat"], N["lon"], S["lat"], S["lon"])
    Px, Py = gps_to_local(N["lat"], N["lon"], lat, lon)

    dx, dy = Sx - Nx, Sy - Ny
    if dx == 0 and dy == 0:
        return 0.0, sqrt(Px**2 + Py**2), 0.0

    t = ((Px - Nx) * dx + (Py - Ny) * dy) / (dx*dx + dy*dy)
    t_clamped = max(0, min(1, t))

    proj_x = Nx + t_clamped * dx
    proj_y = Ny + t_clamped * dy

    dist_from_N = sqrt(proj_x**2 + proj_y**2)
    dist_perp = sqrt((Px - proj_x)**2 + (Py - proj_y)**2)
    return dist_from_N, dist_perp, sqrt(dx*dx + dy*dy)

# ==========================
# ROS2 Node
# ==========================

class GPSRowDetector(Node):
    def __init__(self):
        super().__init__('gps_row_detector')
        self.sub_gps = self.create_subscription(NavSatFix, '/gps/fix', self.listener_callback, 10)
        self.pub_info = self.create_publisher(String, '/navigation/information', 10)
        self.last_dist_from_N = None
        self.last_row = None
        self.get_logger().info("GPSRowDetector initialized and listening...")

    def listener_callback(self, msg: NavSatFix):
        lat, lon = msg.latitude, msg.longitude

        # 1️⃣ Identificar la fila más cercana
        best_section = best_row = None
        best_dist_from_N = None
        min_perp_dist = float("inf")

        for section, section_rows in rows.items():
            for row_name, coords in section_rows.items():
                dist_from_N, dist_perp, _ = project_point_on_row(lat, lon, coords["N"], coords["S"])
                if dist_perp < min_perp_dist:
                    min_perp_dist = dist_perp
                    best_section, best_row = section, row_name
                    best_dist_from_N = dist_from_N

        # 2️⃣ Ajustar fila real según dirección
        direction = "unknown"
        real_row = best_row
        if self.last_row == (best_section, best_row) and self.last_dist_from_N is not None:
            if best_dist_from_N > self.last_dist_from_N:
                direction = "North → South"
                if int(best_row.replace("row","")) % 2 == 0:
                    real_row = f"row{int(best_row.replace('row',''))-1}"
            elif best_dist_from_N < self.last_dist_from_N:
                direction = "South → North"
                if int(best_row.replace("row","")) % 2 == 1:
                    real_row = f"row{int(best_row.replace('row',''))+1}"

        self.last_dist_from_N, self.last_row = best_dist_from_N, (best_section, best_row)

        # 3️⃣ Publicar información
        info_str = (
            f"section: {best_section}, "
            f"row: {real_row}, "
            f"position_from_N: {best_dist_from_N:.2f} m, "
            f"direction: {direction}"
        )
        if PRINT_INFO:
            self.get_logger().info(info_str)
        self.pub_info.publish(String(data=info_str))

# ==========================
# Main
# ==========================

def main(args=None):
    rclpy.init(args=args)
    node = GPSRowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

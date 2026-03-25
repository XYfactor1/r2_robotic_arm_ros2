
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import time
from pathlib import Path

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

model_name = str(Path.home() / "ros2_ws" / "V8_TEXT.pt")

class RealSenseYOLOCupDetector:
    def __init__(self, model_name='yolov8n', conf_threshold=0.5, cube_height=0.1):
        """
        初始化RealSense相机和YOLO模型。
        
        参数:
            model_name (str): YOLO模型名称 (默认: 'yolov8n')
            conf_threshold (float): 检测的置信度阈值
            cube_height (float): 目标正方体的高度，单位为米 (默认: 0.1 米 = 10 cm)
        """
        self.conf_threshold = conf_threshold
        self.cup_class_id = 41  # COCO数据集中'cup'类别的ID
        self.cube_height = cube_height  # 正方体高度参数

        # ── ROS2 初始化 ──────────────────────────────────────────────────
        rclpy.init()
        self._ros_node = Node('realsense_yolo_detector')
        # 发布最近目标的3D位置（新坐标系：X前、Y左、Z上）
        self._pub_position = self._ros_node.create_publisher(
            PointStamped, '/detected_target/position', 10
        )
        # 发布最近目标的类别名称与姿态信息（便于调试）
        self._pub_info = self._ros_node.create_publisher(
            String, '/detected_target/info', 10
        )
        self._ros_node.get_logger().info('RealSense YOLO Detector node started.')
        # ────────────────────────────────────────────────────────────────
        
        # 初始化RealSense管道
        self.pipeline = rs.pipeline()
        self.timeout_count = 0  # 超时计数器
        self.max_consecutive_timeouts = 5  # 最多允许连续超时次数
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # 开始视频流（带重试机制）
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.profile = self.pipeline.start(config)
                break
            except RuntimeError as e:
                if attempt < max_retries - 1:
                    print(f"相机初始化失败 (第 {attempt+1} 次尝试): {e}")
                    time.sleep(1)
                    self.pipeline = rs.pipeline()  # 重新创建管道
                else:
                    raise RuntimeError(f"无法初始化相机，已重试 {max_retries} 次") from e
        
        # 获取深度传感器的深度比例
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # 创建一个对齐对象，用于将深度帧与彩色帧对齐
        self.align = rs.align(rs.stream.color)
        
        # 加载YOLO模型
        self.model = YOLO(f'{model_name}')  # Loads official YOLOv8 model
        self.conf_threshold = conf_threshold
        
        # 创建颜色映射对象，用于可视化深度数据
        self.colorizer = rs.colorizer()
        # self.colorizer.set_option(rs.option.visual_preset, 0) # 0=Dynamic, 1=Fixed, 2=Near, 3=Far
        # self.colorizer.set_option(rs.option.min_distance, 0) # 单位：米
        # self.colorizer.set_option(rs.option.max_distance, 16) # 单位：米

        # 坐标系约定：
        # - RealSense 光学坐标系（原始）：X右、Y下、Z前
        # - 本项目所用新坐标系：X前(沿光轴)、Y左、Z上
        # 变换关系（new = T * optical）：
        #   Xn = Zo
        #   Yn = -Xo
        #   Zn = -Yo

    def _optical_to_new(self, x_o: float, y_o: float, z_o: float):
        """将RealSense光学坐标系(X右,Y下,Z前)转换为新坐标系(X前,Y左,Z上)。"""
        x_n = float(z_o)
        y_n = float(-x_o)
        z_n = float(-y_o)
        return x_n, y_n, z_n

    def get_cube_top_center(self, center_pos, normal_vector=None):
        """
        将目标的中心点坐标转换为正方体顶部面的中心坐标。
        
        假设识别到的目标是一个正方体，识别点为正方体的中心。
        本方法计算正方体顶部面中心的坐标。
        
        参数:
            center_pos: 元组 (x, y, z)，表示正方体中心在新坐标系中的位置
                       新坐标系：X前、Y左、Z上
            normal_vector: 元组 (nx, ny, nz)，表示侧面的法向量（可选）
                          如果提供，将用于计算顶部面中心在X和Y方向的位置
        
        返回:
            tuple: (x_top, y_top, z_top)，表示正方体顶部面中心的坐标
        """
        if center_pos is None:
            return None
        
        x, y, z = center_pos
        
        # Z 轴方向：顶部面中心在 Z 方向上升 (cube_height / 2)
        z_top = z + self.cube_height / 2.0
        
        # X 和 Y 轴方向：基于法向量计算偏移
        if normal_vector is not None and len(normal_vector) == 3:
            # 法向量 (nx, ny, nz) 表示侧面的法向方向
            # 顶部面中心应该沿着法向量方向移动一定距离
            nx, ny, nz = normal_vector
            
            # 计算法向量在 XY 平面的投影（用于 XY 方向的偏移）
            # 假设正方体的边长等于高度 cube_height
            # 顶部面中心相对于中心点在 XY 平面上的偏移量为：
            # 偏移 = (cube_height / 2) * (nx, ny) / sqrt(nx^2 + ny^2)
            
            xy_length = np.sqrt(nx**2 + ny**2)
            if xy_length > 1e-6:
                # 归一化 XY 分量
                nx_norm = nx / xy_length
                ny_norm = ny / xy_length
                
                # 根据法向量方向计算 XY 偏移
                # 侧面的法向量指向目标外侧，顶部中心应沿着这个方向偏移
                xy_offset = self.cube_height / 2.0
                x_top = x + xy_offset * nx_norm
                y_top = y + xy_offset * ny_norm
            else:
                # 如果 XY 分量过小，不做偏移
                x_top = x
                y_top = y
        else:
            # 如果没有法向量信息，保持原位置（向下兼容）
            x_top = x
            y_top = y
        
        return (x_top, y_top, z_top)

    def get_compute_device_label(self):
        """返回当前推理使用的算力平台字符串（GPU/CPU）。"""
        # Ultralytics底层通常是torch，这里做一个不强依赖的兜底判断
        try:
            import torch  # type: ignore

            if torch.cuda.is_available():
                idx = torch.cuda.current_device()
                name = torch.cuda.get_device_name(idx)
                return f"GPU:{idx} {name}"
            return "CPU"
        except Exception:
            # 如果torch不可用（理论上ultralytics会带），就返回未知
            return "Unknown"
    
    def get_frames(self):
        """
        从RealSense获取对齐的彩色和深度帧。
        具备自动错误恢复能力。
        """
        try:
            # 等待获取一对连贯的帧（超时时间：1000ms）
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # 重置超时计数器
            self.timeout_count = 0
            
            # 将深度帧与彩色帧对齐
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None, None
            
            # 将图像转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 为深度帧上色以便可视化
            depth_colormap = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
            
            return color_image, depth_image, depth_colormap
        except RuntimeError as e:
            self.timeout_count += 1
            if "Frame didn't arrive" in str(e):
                print(f"⚠️  帧数据超时 (连续 {self.timeout_count} 次)，尝试恢复...")
                
                # 如果超时次数过多，执行完整重启
                if self.timeout_count > self.max_consecutive_timeouts:
                    print(f"❌ 连续超时 {self.timeout_count} 次，执行完整重启...")
                    try:
                        self.pipeline.stop()
                    except:
                        pass
                    time.sleep(1)
                    self.pipeline = rs.pipeline()
                    try:
                        config = rs.config()
                        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                        self.profile = self.pipeline.start(config)
                        self.timeout_count = 0  # 重置计数器
                        print("✓ 相机已重新启动")
                    except Exception as reconnect_error:
                        print(f"❌ 重新启动失败: {reconnect_error}")
                else:
                    # 轻量级恢复：只是略等待后重试
                    time.sleep(0.01)
            return None, None, None
    
        # 运行YOLO推理(Ultralytics YOLO默认期望BGR格式)
    def detect_cups(self, color_image):
            """
            使用YOLO在彩色图像中检测杯子。
            
            参数:
                color_image: 输入的彩色图像(BGR格式)
                
            返回:
                list: 检测到的杯子的边界框列表 [x1, y1, x2, y2, 置信度, 类别ID]
        
            """
            results = self.model(color_image, conf=self.conf_threshold, verbose=False)

            detections = []
            for r in results:
                if r.boxes is None or len(r.boxes) == 0:
                    continue

                boxes = r.boxes.xyxy.cpu().numpy()          # (N,4) float
                confs = r.boxes.conf.cpu().numpy().astype(float)
                clss = r.boxes.cls.cpu().numpy().astype(int)

                # 类别名映射（可能为None）
                names = getattr(r, "names", None)

                for box, conf, cls_id in zip(boxes, confs, clss):
                    name = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)

                    # 如果你仍然只想看"杯子"，再打开下面这行并把类别改成你训练集里的类id
                    # if cls_id != self.cup_class_id:  # 注意：自训练模型一般不是41
                    #     continue

                    detections.append(
                        {"box": box, "conf": conf, "cls": cls_id, "name": name}
                    )
            return detections
    
    def get_3d_position(self, bbox, depth_image):
        """
        计算边界框中心点在相机坐标系中的3D位置。
        
        参数:
            bbox: 边界框 [x1, y1, x2, y2, 置信度, 类别ID]
            depth_image: 来自RealSense的深度图像
            
        返回:
            tuple: 相机坐标系中的(x, y, z)坐标，单位为米

        注意：本函数返回"新坐标系"下的3D坐标：
            原点（Origin）：相机光心（optical center）
            X 轴（+X）：相机正前方（沿光轴向前）
            Z 轴（+Z）：相机视角的上方
            Y 轴（+Y）：相机视角的左方
        """
        # 1. 获取图像尺寸
        h, w = depth_image.shape[:2]
        
        # 2. 确保边界框在图像范围内
        x1 = max(0, min(int(bbox[0]), w-1))
        y1 = max(0, min(int(bbox[1]), h-1))
        x2 = max(0, min(int(bbox[2]), w-1))
        y2 = max(0, min(int(bbox[3]), h-1))
        
        # 3. 计算中心点（使用浮点运算）
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        
        # 4. 获取中心点周围区域的中值深度（更鲁棒）
        half_size = 5  # 可以调整
        x_start = max(0, int(center_x) - half_size)
        y_start = max(0, int(center_y) - half_size)
        x_end = min(w, int(center_x) + half_size + 1)
        y_end = min(h, int(center_y) + half_size + 1)
        
        roi = depth_image[y_start:y_end, x_start:x_end]
        if roi.size == 0:
            return None
        
        # 使用非零深度值的中位数
        valid_depths = roi[roi > 0]
        if valid_depths.size == 0:
            return None
        
        depth = np.median(valid_depths) * self.depth_scale  # 转换为米
        
        # 5. 获取相机内参
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        
        # 6. 先得到RealSense光学坐标系（X右,Y下,Z前）
        x_o = (center_x - intrinsics.ppx) / intrinsics.fx * depth
        y_o = (center_y - intrinsics.ppy) / intrinsics.fy * depth
        z_o = depth

        # 7. 转换到新坐标系（X前,Y左,Z上）
        x_n, y_n, z_n = self._optical_to_new(x_o, y_o, z_o)
        return (x_n, y_n, z_n)

    def estimate_plane_orientation(self, bbox, depth_image, roi_scale=0.5, sample_step=3, min_points=80):
        """\
        基于深度图估计检测框中心区域的平面法向量，并输出(pitch, yaw)。

        - 平面点来自bbox中心的roi_scale比例子框（默认1/2尺寸）
        - 使用相机内参将像素反投影到3D点
        - 用SVD拟合平面法向量

        返回:
            (pitch_deg, yaw_deg, normal_xyz) 或 (None, None, None)
        """
        if bbox is None:
            return None, None, None

        h, w = depth_image.shape[:2]
        x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
        x1 = max(0.0, min(x1, w - 1.0))
        x2 = max(0.0, min(x2, w - 1.0))
        y1 = max(0.0, min(y1, h - 1.0))
        y2 = max(0.0, min(y2, h - 1.0))
        if x2 <= x1 or y2 <= y1:
            return None, None, None

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        bw = (x2 - x1) * roi_scale
        bh = (y2 - y1) * roi_scale

        rx1 = int(max(0, min(w - 1, round(cx - bw / 2.0))))
        rx2 = int(max(0, min(w, round(cx + bw / 2.0))))
        ry1 = int(max(0, min(h - 1, round(cy - bh / 2.0))))
        ry2 = int(max(0, min(h, round(cy + bh / 2.0))))
        if rx2 <= rx1 or ry2 <= ry1:
            return None, None, None

        # 相机内参
        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()

        pts = []
        for v in range(ry1, ry2, sample_step):
            for u in range(rx1, rx2, sample_step):
                d_raw = depth_image[v, u]
                if d_raw <= 0:
                    continue
                z_o = float(d_raw) * self.depth_scale
                x_o = (u - intr.ppx) / intr.fx * z_o
                y_o = (v - intr.ppy) / intr.fy * z_o
                x_n, y_n, z_n = self._optical_to_new(x_o, y_o, z_o)
                pts.append((x_n, y_n, z_n))

        if len(pts) < min_points:
            return None, None, None

        P = np.asarray(pts, dtype=np.float32)  # (N,3)
        centroid = P.mean(axis=0)
        Q = P - centroid

        # SVD: 最小奇异值对应法向量
        try:
            _, _, vh = np.linalg.svd(Q, full_matrices=False)
        except np.linalg.LinAlgError:
            return None, None, None

        n = vh[-1, :].astype(np.float32)
        norm = float(np.linalg.norm(n))
        if norm <= 1e-6:
            return None, None, None
        n = n / norm

        # 统一方向：让法向量尽量"朝向相机前方"
        # 在新坐标系里，+X是前方，因此若nx<0就翻转
        if n[0] < 0:
            n = -n

        # 角度定义（新坐标系：X前、Y左、Z上）
        # yaw: 绕Z轴（上）旋转，右转为负、左转为正
        # pitch: 绕Y轴（左）旋转，抬头为正、低头为负
        # 对于一个指向方向向量v（这里用平面法向量n），可用如下近似：
        #   yaw = atan2(ny, nx)
        #   pitch = atan2(nz, sqrt(nx^2 + ny^2))
        yaw = float(np.degrees(np.arctan2(n[1], n[0])))
        pitch = float(np.degrees(np.arctan2(n[2], np.sqrt(n[0] ** 2 + n[1] ** 2))))

        return pitch, yaw, (float(n[0]), float(n[1]), float(n[2]))
    




    def visualize(self, color_image, depth_colormap, detections, positions,
                  fps=None, device_label=None, nearest_idx=None):
        if detections:
            for i, (det, pos) in enumerate(zip(detections, positions)):
                box = det["box"]
                conf = float(det["conf"])
                name = det.get("name", str(det.get("cls", "")))

                pitch = det.get("pitch")
                yaw = det.get("yaw")

                x1, y1, x2, y2 = [int(v) for v in box]

                # 最近目标用红框，其余用绿框
                color = (0, 0, 255) if i == nearest_idx else (0, 255, 0)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), color, 2)

                nearest_tag = " [NEAREST]" if i == nearest_idx else ""

                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

                label = f"{name}{nearest_tag} {conf:.2f}"
                if pos is not None:
                    label += f" ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})m"

                if pitch is not None and yaw is not None:
                    label += f" pitch:{pitch:.1f} yaw:{yaw:.1f}"

                cv2.putText(
                    color_image, label, (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
                )

        # 叠加FPS显示
        if fps is not None:
            cv2.putText(
                color_image,
                f"FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        # 叠加算力平台显示
        if device_label:
            cv2.putText(
                color_image,
                f"Device: {device_label}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )

        images = np.hstack((color_image, depth_colormap))
        cv2.imshow("RealSense YOLO Detection", images)

    def publish_nearest_target(self, nearest_det, nearest_pos):
        """
        将最近目标的3D位置和信息通过ROS2话题发布。
        转换识别的目标中心为正方体顶部面的中心坐标后发布。
        """
        now = self._ros_node.get_clock().now().to_msg()

        # 获取法向量信息（如果有）
        normal_vector = nearest_det.get("normal")
        
        # 将目标中心转换为正方体顶部面中心（基于法向量）
        top_center_pos = self.get_cube_top_center(nearest_pos, normal_vector)
        
        # 发布 PointStamped（使用顶部面中心坐标）
        pt_msg = PointStamped()
        pt_msg.header.stamp = now
        pt_msg.header.frame_id = 'camera'
        pt_msg.point.x = float(top_center_pos[0])  # X 前
        pt_msg.point.y = float(top_center_pos[1])  # Y 左
        pt_msg.point.z = float(top_center_pos[2])  # Z 上
        self._pub_position.publish(pt_msg)

        # 发布文字信息（包含原始中心和转换后的顶部中心）
        pitch = nearest_det.get("pitch")
        yaw   = nearest_det.get("yaw")
        name  = nearest_det.get("name", "unknown")
        conf  = float(nearest_det.get("conf", 0.0))
        info  = (
            f"name={name} conf={conf:.2f} "
            f"center=({nearest_pos[0]:.3f},{nearest_pos[1]:.3f},{nearest_pos[2]:.3f}) "
            f"top_center=({top_center_pos[0]:.3f},{top_center_pos[1]:.3f},{top_center_pos[2]:.3f})"
        )
        if pitch is not None and yaw is not None:
            info += f" pitch={pitch:.2f} yaw={yaw:.2f}"
        str_msg = String()
        str_msg.data = info
        self._pub_info.publish(str_msg)

    def run(self):
        """
        检测和可视化的主循环。
        """
        try:
            prev_time = time.perf_counter()
            device_label = self.get_compute_device_label()
            print(f"Compute device: {device_label}")
            self._ros_node.get_logger().info(f"Compute device: {device_label}")
            while True:
                now = time.perf_counter()
                dt = now - prev_time
                prev_time = now
                fps = (1.0 / dt) if dt > 0 else 0.0

                # 让 ROS2 处理一次回调（非阻塞）
                rclpy.spin_once(self._ros_node, timeout_sec=0.0)

                # 获取帧
                color_image, depth_image, depth_colormap = self.get_frames()
                if color_image is None or depth_image is None:
                    continue

                # 检测目标
                detections = self.detect_cups(color_image)

                # 计算所有目标的3D位置与姿态
                positions = []
                for det in detections:
                    pos = self.get_3d_position(det["box"], depth_image)
                    positions.append(pos)

                    pitch, yaw, _n = self.estimate_plane_orientation(det["box"], depth_image, roi_scale=0.5)
                    det["pitch"] = pitch
                    det["yaw"] = yaw
                    det["normal"] = _n

                # ── 找到最近的目标（新坐标系 X 为前向距离，值越小越近）──────
                # 过滤掉距离超过 4 米的目标
                MAX_DISTANCE = 4.0  # 单位：米
                nearest_idx = None
                nearest_pos = None
                nearest_det = None
                valid_pairs = [
                    (i, pos) for i, pos in enumerate(positions) 
                    if pos is not None and pos[0] <= MAX_DISTANCE
                ]
                if valid_pairs:
                    nearest_idx, nearest_pos = min(valid_pairs, key=lambda t: t[1][0])
                    nearest_det = detections[nearest_idx]
                    
                    # 获取法向量信息
                    normal_vector = nearest_det.get("normal")
                    
                    # 计算顶部面中心坐标（基于法向量）
                    top_center_pos = self.get_cube_top_center(nearest_pos, normal_vector)
                    
                    print(
                        f"[NEAREST] {nearest_det.get('name')} "
                        f"center=({nearest_pos[0]:.3f}, {nearest_pos[1]:.3f}, {nearest_pos[2]:.3f}) m"
                    )
                    print(
                        f"          top_center=({top_center_pos[0]:.3f}, {top_center_pos[1]:.3f}, {top_center_pos[2]:.3f}) m"
                    )
                    if nearest_det.get("pitch") is not None:
                        print(
                            f"          pitch={nearest_det['pitch']:.2f} deg, "
                            f"yaw={nearest_det['yaw']:.2f} deg"
                        )
                    # 发布 ROS2 消息（仅发布最近目标）
                    self.publish_nearest_target(nearest_det, nearest_pos)

                self.visualize(
                    color_image.copy(),
                    depth_colormap,
                    detections,
                    positions,
                    fps=fps,
                    device_label=device_label,
                    nearest_idx=nearest_idx,
                )

                # 按 ESC 键退出循环
                if cv2.waitKey(1) & 0xFF == 27:
                    break

        finally:
            # 停止视频流
            self.pipeline.stop()
            cv2.destroyAllWindows()
            # 关闭 ROS2
            self._ros_node.destroy_node()
            rclpy.shutdown()


def main():
    """ROS2 entry point."""
    # 可以在这里调整正方体的高度（单位：米）
    # 默认值：0.35 米 (35 cm)
    cube_height = 0.35
    
    detector = RealSenseYOLOCupDetector(
        model_name=model_name, 
        conf_threshold=0.5,
        cube_height=cube_height
    )
    detector.run()

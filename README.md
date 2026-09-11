# nav2_uav_bridge

讓 Nav2（原生為地面機器人設計）可以拿來控制 PX4 SITL 無人機的橋接層，並提供一個
「用 AprilTag 當作障礙物來源」的避障 demo（單機、Gazebo 模擬）。

## AprilTag 立牌目前放在哪裡

立牌是在 Gazebo 啟動後，用 `ros_gz_sim create` 動態生成的（不是寫死在 world
檔案裡），目前的位置/朝向是：

```
世界座標 (x, y, z) = (2, 0, 0)　　朝向 -Y 1.5708（yaw 轉 90 度）
```

無人機是 spawn 在世界原點 `(0, 0, 0)` 附近，所以立牌等於在無人機正前方 2 公尺處。
立牌模型本身在 `<link>` 裡有內建 `z=1.7` 的高度偏移（把面板墊高離地），所以
實際看到的黑白標記面板世界座標是 `(2, 0, 1.7)`。

**要調整位置的話**，改 `launch/apriltag_pipeline_launch.py` 裡 `spawn_marker`
這個 Node 的 `arguments`：

```python
spawn_marker = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-world', 'default',
        '-file', '/root/PX4-Autopilot/Tools/simulation/gz/models/apriltag_marker_0/model.sdf',
        '-name', 'apriltag_marker_0',
        '-x', '2', '-y', '0', '-z', '0', '-Y', '1.5708',   # ← 改這幾個數字
    ],
)
```

改完存檔後要重新 `colcon build --packages-select nav2_uav_bridge`，下次重跑
`apriltag_pipeline_launch.py` 才會用新位置生成立牌。如果模擬還在跑、只是想
臨時挪動位置測試，也可以直接下指令移動已經生成的模型，不用重啟整套：

```bash
ros2 run ros_gz_sim remove -world default -entity apriltag_marker_0
ros2 run ros_gz_sim create -world default \
  -file /root/PX4-Autopilot/Tools/simulation/gz/models/apriltag_marker_0/model.sdf \
  -name apriltag_marker_0 -x <新的x> -y <新的y> -z 0 -Y <新的朝向>
```

## 這個套件在做什麼

Nav2 假設機器人是一台會發布 `/odom`、吃 `/cmd_vel`、有雷射掃描的地面機器人。
PX4 無人機完全不是這個介面。這個套件用四個小節點把兩邊接起來：

| 節點 | 做什麼 |
|---|---|
| `odom_bridge` | 訂閱 PX4 的 `VehicleLocalPosition`（NED），轉成 `nav_msgs/Odometry`（ENU）發布到 `/odom`，同時廣播 `odom → base_link` 這段 TF |
| `cmd_vel_bridge` | 訂閱 Nav2 算出來的 `/cmd_vel`（機身座標系 Twist），轉成 PX4 的 `OffboardControlMode` + `TrajectorySetpoint`（NED 速度），並負責自動 arm、切 offboard、起飛到固定高度 |
| `fake_scan_publisher` | 發布一個假的 `sensor_msgs/LaserScan`（每個角度都回報 `range_max`，永遠沒有障礙物）。純粹是因為 AMCL 跟 `collision_monitor` 需要有 `/scan` 才能跑，這台無人機沒有真雷達 |
| `apriltag_obstacle_bridge` | 把 `apriltag_ros` 偵測到的 tag，透過查 TF 轉成世界座標，包成 `PointCloud2` 發布到 `/apriltag_obstacles`，餵給 costmap 的 `obstacle_layer` |

## 架構 / 資料流

```
PX4 SITL (Gazebo)
  │  uXRCE-DDS (MicroXRCEAgent)
  ▼
/fmu/out/vehicle_local_position_v1 ──► odom_bridge ──► /odom, TF(odom→base_link)
/fmu/in/trajectory_setpoint         ◄── cmd_vel_bridge ◄── /cmd_vel ◄── Nav2 controller_server

Gazebo 相機 ──► ros_gz_bridge ──► /camera/image_raw, /camera/camera_info
                                        │
                                        ▼
                                  apriltag_node (apriltag_ros)
                                        │  TF: camera_link → tag36h11:0
                                        ▼
                              apriltag_obstacle_bridge
                                        │  查 map→...→tag36h11:0 完整 TF 鏈
                                        ▼
                              /apriltag_obstacles (PointCloud2, map frame)
                                        │
                    ┌───────────────────┴───────────────────┐
                    ▼                                        ▼
            local_costmap.obstacle_layer            global_costmap.obstacle_layer
                    │                                        │
                    ▼                                        ▼
              controller_server (MPPI)              planner_server (NavFn)
                    │
                    ▼
                 /cmd_vel ──► cmd_vel_bridge（回到上面）
```

### TF 樹

```
map ─(static, 重合)─► odom ─(odom_bridge 動態發布)─► base_link ─┬─(static)─► base_footprint
                                                                  └─(static)─► camera_link
```

`map → odom` 是寫死的 identity transform（這個 demo 沒有全域定位，`map` 座標系
基本上等於無人機自己 PX4 local-position 的原點，也就是它在 Gazebo world 裡的
spawn 點——這也代表**如果無人機不是 spawn 在 world (0,0,0)，`map` 座標系會跟
Gazebo 世界座標系差一個固定偏移量**，比對真實位置時要注意。

## 執行方式

總共需要 **5 個獨立的終端機視窗**，每個視窗只跑一個指令、常駐在前景不要關掉。
每個 ROS2 相關的終端機都要自己 source 一次環境（terminal 之間互不共用，複製貼上
的時候不要漏掉 `source` 那幾行）。照順序，等前一步完全啟動好再開下一步：

**終端機 1 — DDS agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**終端機 2 — PX4 SITL + Gazebo**
```bash
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500_mono_cam ./build/px4_sitl_default/bin/px4
```
等 Gazebo 視窗完全跑起來、模型都載入好，再往下一步。

**終端機 3 — AprilTag pipeline**（立牌生成 + 相機橋接 + 偵測 + 障礙物 bridge）
```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch nav2_uav_bridge apriltag_pipeline_launch.py
```
立牌會延遲 5 秒才生成（等 Gazebo world 穩定），不用另外處理。

**終端機 4 — Nav2 主程式**（odom/cmd_vel bridge、TF、costmap、規劃器全部在這裡）
```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch nav2_uav_bridge nav2_uav_bringup_launch.py
```

**終端機 5 — RViz**
```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

`apriltag_pipeline_launch.py`（終端機 3）跟 `nav2_uav_bringup_launch.py`（終端機 4）
特意分成兩個獨立的 launch 檔——前者裡的 `apriltag_node`、`apriltag_obstacle_bridge`
是已知會在長時間運行後卡死的節點（見下方注意事項），分開才能只重開終端機 3，
不用連 Nav2 整套一起重開。

## 設定檔

- `params/nav2_params.yaml` — 標準 Nav2 參數檔，`local_costmap`/`global_costmap` 都把
  `obstacle_layer` 接到 `/apriltag_obstacles`（**兩個 costmap 都要接，只接
  `local_costmap` 的話全域規劃器會完全不知道有障礙物，規劃出來的路徑會直接
  穿過去**）
- `params/apriltag.yaml` — `family`/`size`。`size` 一定要對應 tag **黑色標記本體**
  的實際邊長，不是立牌整塊面板的尺寸，錯了會讓測距系統性地偏移（見下方注意事項）
- `maps/blank_map.yaml` + `blank_map.pgm` — 給 `map_server` 用的空白靜態地圖
  （純粹是因為 Nav2 需要一張地圖才能跑起來，這裡沒有事先畫任何障礙物）

## 注意事項（踩過的坑）

這些是實際測試中發生過、而且不容易一眼看出來的問題，供之後接手的人參考。

1. **不要同時跑兩份 `nav2_uav_bringup_launch.py`。**
   兩套 stack 會搶著發 `/tf`、`/map`、costmap，導致 RViz 地圖顯示不出來、
   Nav2 lifecycle 卡在 `unconfigured`、甚至整個 `nav2_container` crash。
   啟動前先 `ps aux | grep nav2_uav_bringup` 確認沒有殘留的舊 process。

2. **`apriltag_node` 跟 `apriltag_obstacle_bridge` 不在主 launch 檔裡，是獨立
   手動啟動的長壽命 process，而且撐不了太久。**
   PX4/Gazebo 每重啟一次，模擬時間就會「往回跳」，這兩個節點內部的 tf2 buffer
   /時鐘狀態撐過幾次跳動之後很容易卡死——外觀上 process 還活著，但
   `/apriltag_obstacles` 會停止更新（或者更危險：卡在很久以前、無人機在
   完全不同位置時的舊座標，因為 costmap 的 `clearing: False` 不會自動清掉）。
   **PX4/Gazebo 重開之後，這兩個節點也建議跟著重開。**

3. **`apriltag.yaml` 的 `size`（tag 邊長）務必用實測校正，不要單靠貼圖尺寸推算。**
   單目 AprilTag 測距是用「已知實際尺寸 + 影像視覺大小」反推距離，`size` 設太大
   會讓算出來的距離系統性地偏遠。**貼圖分析算出來的尺寸不可靠**——我們一開始
   用貼圖黑色區域佔比反推出 `0.48`，結果實測誤差還是接近 2 倍，最後是靠比對
   Gazebo 真實座標才抓到正確值 `0.24`，兩次估計差了一倍。校正方法：同時記錄
   Gazebo 裡無人機/立牌的真實世界座標（`gz topic -e -t /world/default/pose/info`）
   跟 `/apriltag_obstacles` 算出來的座標，分別算出「無人機到真實立牌」跟
   「無人機到偵測結果」的距離，兩者比值就是 `size` 該乘上的校正倍率
   （比值 >1 代表 `size` 設太大，要往下調）。改完 `size` 記得連 **終端機 4
   的 Nav2 也要重開**（見下一點）。

4. **改完 `size` 或重啟 `apriltag_obstacle_bridge` 之後，記得連 Nav2（終端機 4）
   也一起重開，不然 costmap 裡的舊障礙物標記不會消失。**
   `obstacle_layer` 設定 `clearing: False`，代表任何一次標記過的格子都是
   **永久的**，不會因為之後偵測結果變準了就自動更新/清除。校正 `size`
   之後如果只重啟 `apriltag_node`，costmap 裡还是会同时存在「舊尺寸算出來的
   錯誤標記」跟「新尺寸算出來的正確標記」疊在一起，除非把整個 Nav2 stack
   （costmap 的記憶體狀態）重開，舊標記才會真正消失。

5. **節點剛啟動時如果看到 `TF_OLD_DATA ignoring data from the past` 警告，
   通常是無害的過渡現象，不用緊張。**
   這是 `use_sim_time:=true` 的節點在收到第一筆 `/clock` 訊息之前，內部時鐘
   還是 0 造成的暫時性警告，正常情況下幾秒內就會自己停止。真正要看的是
   節點穩定之後，資料有沒有正常持續更新（例如 `/apriltag_obstacles` 是否
   真的在發布新的點），而不是這幾行警告本身。

6. **`global_costmap` 一定要把 `obstacle_layer` 放進 `plugins` 清單，且
   `observation_sources` 要接到 `/apriltag_obstacles`。**
   只接 `local_costmap` 的話，全域規劃器（`planner_server`/NavFn）完全看不到
   障礙物，算出來的路徑一開始就會直接穿過去，只能靠局部規劃器臨場閃避
   （常常來不及）。

7. **`local_costmap` 的視窗大小要大於 `inflation_radius` 的兩倍以上。**
   `inflation_radius` 太接近視窗半徑，會讓整個局部 costmap 被危險係數填滿，
   MPPI 反而找不到低成本路徑。

8. **`cost_scaling_factor` 的方向很反直覺：數字越大，代價衰減越快、危險區域
   看起來越小越集中；數字越小，衰減越慢、危險區域擴散得越廣。** 想要路徑
   對任何顏色都更不敢碰，是調低這個值，不是調高。

9. **AprilTag 偵測有「死角」**：距離立牌約 0.6 公尺以內幾乎完全偵測不到
   （鏡頭視角/對焦限制）。飛得越快，死角前能反應、修正路徑的時間越短，
   避障可靠度越差。

10. **這台機器上 `ros2 topic echo`/`ros2 topic hz` 常常抓不到東西**，尤其是
   PX4 用 `BEST_EFFORT + TRANSIENT_LOCAL` QoS 的 topic——CLI 工具的預設訂閱
   QoS 常常對不上，看起來像是「沒資料」但其實資料是有的。要偵錯的話，寫一支
   帶正確 QoS 設定的小 rclpy 腳本會比信任 CLI 工具準確。

11. **`VehicleCommand.target_system` 要設 `0`（broadcast），不要用 `uav_id`。**
   PX4 的 `MAV_SYS_ID` 是 `instance + 1`，不是你自己指定的 id，直接用 id
   當 target 會被 Commander 的 target_system 檢查悄悄丟棄。

## 已知限制 / 之後可以做的事

- 障礙物來源只有單一個 AprilTag 點，沒有真正的形狀/深度資訊，`inflation_radius`
  只能對稱地往四面八方擴散，無法表示「這個障礙物只有某個方向危險」
- 沒有讓無人機在避障時主動調整機頭朝向去對準障礙物（純靠 Nav2 的路徑/速度
  控制），如果之後要接真實避障邏輯，這塊需要另外設計
- `map` 座標系目前假設無人機一定 spawn 在 Gazebo world 原點，沒有做真正的
  全域定位；多機或非原點 spawn 的情境需要額外處理

## 給接手者的指南

### 環境需求（假設已經裝好，這裡不重講安裝步驟）

- ROS2 Jazzy（`/opt/ros/jazzy`）
- 已經 build 好的 PX4-Autopilot（`~/PX4-Autopilot`，含 `gz_x500_mono_cam` 機型）
- `MicroXRCEAgent`、`ros_gz_bridge`、`apriltag_ros`、`nav2_bringup` 這幾個套件
- `px4_msgs` 要跟正在跑的 PX4 韌體版本對得上，對不上的話 topic 資料會悄悄變成
  空的/亂的，不會報錯

### 建議閱讀順序

1. 先看上面「架構 / 資料流」那張圖，搞懂資料怎麼從 PX4 → Nav2 → 再回到 PX4
2. 照「執行方式」那 5 個終端機把整套跑起來一次，親眼看到無人機能自己飛去繞過立牌
3. 出狀況時，先查「注意事項」那 11 條，大部分踩過的坑都寫在那裡
4. 真的要動參數，`params/nav2_params.yaml` 裡每個數值的意義可以參考「注意事項」
   裡 `inflation_radius`/`cost_scaling_factor`/critic 權重那幾條，改完記得
   重開終端機 4（Nav2 不會自動重讀 yaml）

### 上手前的驗證流程（確認環境沒問題再開始改東西）

1. 5 個終端機都啟動、都沒有紅字 error（`TF_OLD_DATA` 那種警告可以忽略，見注意事項第 5 條）
2. RViz 裡看得到白色的地圖、機身模型有跟著無人機的位置動
3. 讓無人機飛近立牌，確認 RViz 裡看得到障礙物色塊（`Local Costmap`/`Global Costmap`/`PointCloud2 /apriltag_obstacles` 這幾個 Display 記得打勾）
4. 用 RViz 的 **Nav2 Goal** 送一個立牌後面的目標點，確認全域路徑（Global Planner
   顯示）會繞開障礙物，不是直線穿過去
5. 上面 4 步都正常，才算是一個「乾淨」的起點，再開始改參數或加功能

### 想擴充功能，大概從哪裡下手

- **想接真的雷射/深度相機**：改 `local_costmap`/`global_costmap` 的
  `obstacle_layer.observation_sources`，把 `apriltag` 換成/加上新的來源，
  資料格式是 `LaserScan` 或 `PointCloud2` 都可以，不用動其他節點
- **想要無人機避障時機頭主動朝向障礙物**：目前完全沒做這塊，`cmd_vel_bridge`
  只吃 Nav2 算出來的線速度/角速度，沒有額外的朝向邏輯，是最大的架構缺口
- **想接多台無人機**：現在整條 pipeline（TF frame 名稱、topic 名稱）都是單機
  寫死的，要多機得比照 `three_uav_formation` 那個專案的做法，每台加自己的
  命名空間

### 真的卡住的時候

這個專案偵錯過程中發現，**光看 RViz 畫面或相信 CLI 工具的輸出常常會被誤導**
（例如 costmap 裡的障礙物位置其實是很久以前的舊資料、`ros2 topic hz` 因為 QoS
兜不起來而顯示「沒資料」但其實資料是有的）。真的要確認一個數字對不對時，
建議寫一支簡短的 rclpy 腳本，直接訂閱相關 topic 印出數值，並且用
`gz topic -e -t /world/default/pose/info` 查 Gazebo 裡的真實座標互相比對，
不要只憑畫面判斷。

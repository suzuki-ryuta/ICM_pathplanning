#!/usr/bin/env python
# coding: utf-8

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import math

# === 関数群 ===
def deg_to_rad(deg):
    return deg * np.pi / 180.0


def generate_vertex(ref, w, h, th):
    """左手の各リンクを生成"""
    x = [ref[0]]
    y = [ref[1]]
    x2 = ref[0] + w * np.cos(th)
    y2 = ref[1] + w * np.sin(th)
    x.append(x2); y.append(y2)
    x3 = x2 - h * np.sin(th)
    y3 = y2 + h * np.cos(th)
    x.append(x3); y.append(y3)
    x4 = x3 - w * np.cos(th)
    y4 = y3 - w * np.sin(th)
    x.append(x4); y.append(y4)
    x.append(ref[0]); y.append(ref[1])
    return x, y


def generate_vertex2(ref, w, h, th):
    """右手の各リンクを生成"""
    x = [ref[0]]
    y = [ref[1]]
    x2 = ref[0] - w * np.cos(th)
    y2 = ref[1] - w * np.sin(th)
    x.append(x2); y.append(y2)
    x3 = x2 + h * np.sin(th)
    y3 = y2 - h * np.cos(th)
    x.append(x3); y.append(y3)
    x4 = x3 + w * np.cos(th)
    y4 = y3 + w * np.sin(th)
    x.append(x4); y.append(y4)
    x.append(ref[0]); y.append(ref[1])
    return x, y


# === 入力 ===
cluster_date = input("クラスタフォルダの日付 (例: 2025-10-07): ").strip()
cluster_dir = f"../ICM_Log/path/{cluster_date}/"
if not os.path.exists(cluster_dir):
    raise FileNotFoundError(f"クラスタフォルダが見つかりません: {cluster_dir}")

angle_name = input("ハンド角度ファイル名 (.csv抜きで入力): ").strip()
angle_path = f"../ICM_Log/path/{angle_name}.csv"
if not os.path.exists(angle_path):
    raise FileNotFoundError(f"角度ファイルが見つかりません: {angle_path}")

# === データ読み込み ===
df = pd.read_csv(angle_path, header=None, skiprows=[0])  # ハンド角度
cluster_files = sorted(glob.glob(os.path.join(cluster_dir, "clusters_*.csv")))

if len(cluster_files) == 0:
    raise RuntimeError(f"{cluster_dir} にクラスタCSVが見つかりません。")

print(f"検出: {len(cluster_files)} フレーム")

# === ハンド寸法 ===
height = [39, 132, 130, 90]
width  = [60, 54, 54, 54]

# === 描画設定 ===
fig, ax = plt.subplots(figsize=(6, 8))
ax.set(xlim=(-250, 250), ylim=(-10, 500))
ax.set_aspect('equal')

# 動画保存用設定
save_path = os.path.join(cluster_dir, "hand_cluster.mp4")
writer = FFMpegWriter(fps=10, metadata=dict(artist='YNU Maeda Lab'))

# === フレーム更新関数 ===
def update(i):
    ax.clear()
    ax.set(xlim=(-250, 250), ylim=(-10, 500))
    ax.set_aspect('equal')
    ax.set_title(f"Frame {i}")

    # ---- クラスタ点群 ----
        # ---- クラスタ点群 ----
    cluster_df = pd.read_csv(cluster_files[i])

    # (x, y) ごとに θ の個数をカウント
    # theta列がある前提
    grouped = (
        cluster_df
        .groupby(["cluster_id", "x", "y"])
        .size()
        .reset_index(name="theta_count")
    )

    # サイズ設定
    base_size = 10       # θが1つのときのサイズ
    size_gain = 10       # θが1つ増えるごとの拡大量

    for cluster_id, group in grouped.groupby("cluster_id"):
        sizes = base_size + size_gain * (group["theta_count"] - 1)

        ax.scatter(
            group["x"],
            group["y"],
            s=sizes,
            alpha=0.6,
            label=f"C{cluster_id}"
        )

    if len(grouped["cluster_id"].unique()) <= 5:
        ax.legend(loc='upper right', fontsize=8)


    # ---- ハンド描画 ----
    t1, t2, t3, t4, t5, t6 = df.iloc[i, :6]

    # 左手
    AbsAngle = 0.0
    center = [-30, 0]
    ref = [center[0]-0.5*width[0], center[1]]
    x, y = generate_vertex(ref, width[0], height[0], 0.0)
    ax.plot(x, y, color='black')
    center[0] -= height[0]*np.sin(0)
    center[1] += height[0]*np.cos(0)

    AbsAngle += t1
    for theta, w, h in zip([t1, t2, t3], width[1:], height[1:]):
        AbsAngle += theta
        rad = deg_to_rad(AbsAngle)
        ref = [center[0]-0.5*w*np.cos(rad), center[1]-0.5*w*np.sin(rad)]
        x, y = generate_vertex(ref, w, h, rad)
        ax.plot(x, y, color='black')
        center[0] -= h*np.sin(rad)
        center[1] += h*np.cos(rad)

    # 右手
    AbsAngle = 0.0
    center = [30.01, 415]
    ref = [center[0]+0.5*width[0], center[1]]
    x, y = generate_vertex2(ref, width[0], height[0], 0.0)
    ax.plot(x, y, color='black')
    center[0] += height[0]*np.sin(0)
    center[1] -= height[0]*np.cos(0)

    for theta, w, h in zip([t4, t5, t6], width[1:], height[1:]):
        AbsAngle += theta
        rad = deg_to_rad(AbsAngle)
        ref = [center[0]+0.5*w*np.cos(rad), center[1]+0.5*w*np.sin(rad)]
        x, y = generate_vertex2(ref, w, h, rad)
        ax.plot(x, y, color='black')
        center[0] += h*np.sin(rad)
        center[1] -= h*np.cos(rad)

# === アニメーション作成・保存 ===
ani = FuncAnimation(fig, update, frames=min(len(df), len(cluster_files)), repeat=False)
print(f"動画を保存中: {save_path}")
ani.save(save_path, writer=writer)
print("保存完了！")
plt.close()



# !/usr/bin/env python
# coding: utf-8

# import os
# import glob
# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation, FFMpegWriter
# import math

# # === 関数群 ===
# def deg_to_rad(deg):
#     return deg * np.pi / 180.0


# def generate_vertex(ref, w, h, th):
#     """左手の各リンクを生成"""
#     x = [ref[0]]
#     y = [ref[1]]
#     x2 = ref[0] + w * np.cos(th)
#     y2 = ref[1] + w * np.sin(th)
#     x.append(x2); y.append(y2)
#     x3 = x2 - h * np.sin(th)
#     y3 = y2 + h * np.cos(th)
#     x.append(x3); y.append(y3)
#     x4 = x3 - w * np.cos(th)
#     y4 = y3 - w * np.sin(th)
#     x.append(x4); y.append(y4)
#     x.append(ref[0]); y.append(ref[1])
#     return x, y


# def generate_vertex2(ref, w, h, th):
#     """右手の各リンクを生成"""
#     x = [ref[0]]
#     y = [ref[1]]
#     x2 = ref[0] - w * np.cos(th)
#     y2 = ref[1] - w * np.sin(th)
#     x.append(x2); y.append(y2)
#     x3 = x2 + h * np.sin(th)
#     y3 = y2 - h * np.cos(th)
#     x.append(x3); y.append(y3)
#     x4 = x3 + w * np.cos(th)
#     y4 = y3 + w * np.sin(th)
#     x.append(x4); y.append(y4)
#     x.append(ref[0]); y.append(ref[1])
#     return x, y


# # === 入力 ===
# cluster_date = input("クラスタフォルダの日付 (例: 2025-10-07): ").strip()
# cluster_dir = f"../ICM_Log/path/{cluster_date}/"
# if not os.path.exists(cluster_dir):
#     raise FileNotFoundError(f"クラスタフォルダが見つかりません: {cluster_dir}")

# angle_name = input("ハンド角度ファイル名 (.csv抜きで入力): ").strip()
# angle_path = f"../ICM_Log/path/{angle_name}.csv"
# if not os.path.exists(angle_path):
#     raise FileNotFoundError(f"角度ファイルが見つかりません: {angle_path}")

# # === データ読み込み ===
# df = pd.read_csv(angle_path, header=None, skiprows=[0])  # ハンド角度
# cluster_files = sorted(glob.glob(os.path.join(cluster_dir, "clusters_*.csv")))

# if len(cluster_files) == 0:
#     raise RuntimeError(f"{cluster_dir} にクラスタCSVが見つかりません。")

# print(f"検出: {len(cluster_files)} フレーム")

# # === ハンド寸法 ===
# height = [39, 132, 130, 90]
# width  = [60, 54, 54, 54]

# # === 描画設定 ===
# fig, ax = plt.subplots(figsize=(6, 8))
# ax.set(xlim=(-250, 250), ylim=(-10, 500))
# ax.set_aspect('equal')

# # 動画保存用設定
# save_path = os.path.join(cluster_dir, "hand_cluster.mp4")
# writer = FFMpegWriter(fps=10, metadata=dict(artist='YNU Maeda Lab'))

# # === フレーム更新関数 ===
# def update(i):
#     ax.clear()
#     ax.set(xlim=(-250, 250), ylim=(-10, 500))
#     ax.set_aspect('equal')
#     ax.set_title(f"Frame {i}")

#     # ---- クラスタ点群 ----
#     cluster_df = pd.read_csv(cluster_files[i])
#     for cluster_id, group in cluster_df.groupby("cluster_id"):
#         ax.scatter(group["x"], group["y"], s=10, alpha=0.5, label=f"C{cluster_id}")
#     if len(cluster_df["cluster_id"].unique()) <= 5:
#         ax.legend(loc='upper right', fontsize=8)

#     # ---- ハンド描画 ----
#     t1, t2, t3, t4, t5, t6 = df.iloc[i, :6]

#     # 左手
#     AbsAngle = 0.0
#     center = [-30, 0]
#     ref = [center[0]-0.5*width[0], center[1]]
#     x, y = generate_vertex(ref, width[0], height[0], 0.0)
#     ax.plot(x, y, color='black')
#     center[0] -= height[0]*np.sin(0)
#     center[1] += height[0]*np.cos(0)

#     AbsAngle += t1
#     for theta, w, h in zip([t1, t2, t3], width[1:], height[1:]):
#         AbsAngle += theta
#         rad = deg_to_rad(AbsAngle)
#         ref = [center[0]-0.5*w*np.cos(rad), center[1]-0.5*w*np.sin(rad)]
#         x, y = generate_vertex(ref, w, h, rad)
#         ax.plot(x, y, color='black')
#         center[0] -= h*np.sin(rad)
#         center[1] += h*np.cos(rad)

#     # 右手
#     AbsAngle = 0.0
#     center = [30.01, 415]
#     ref = [center[0]+0.5*width[0], center[1]]
#     x, y = generate_vertex2(ref, width[0], height[0], 0.0)
#     ax.plot(x, y, color='black')
#     center[0] += height[0]*np.sin(0)
#     center[1] -= height[0]*np.cos(0)

#     for theta, w, h in zip([t4, t5, t6], width[1:], height[1:]):
#         AbsAngle += theta
#         rad = deg_to_rad(AbsAngle)
#         ref = [center[0]+0.5*w*np.cos(rad), center[1]+0.5*w*np.sin(rad)]
#         x, y = generate_vertex2(ref, w, h, rad)
#         ax.plot(x, y, color='black')
#         center[0] += h*np.sin(rad)
#         center[1] -= h*np.cos(rad)

# # === アニメーション作成・保存 ===
# ani = FuncAnimation(fig, update, frames=min(len(df), len(cluster_files)), repeat=False)
# print(f"動画を保存中: {save_path}")
# ani.save(save_path, writer=writer)
# print("保存完了！")
# plt.close()



# # #!/usr/bin/env python
# # # coding: utf-8

# # import os
# # import glob
# # import numpy as np
# # import pandas as pd
# # import matplotlib.pyplot as plt
# # import math
# # import time

# # # === 関数群 ===
# # def deg_to_rad(deg):
# #     return deg * np.pi / 180.0


# # def generate_vertex(ref, w, h, th):
# #     """左手の各リンクを生成"""
# #     x = [ref[0]]
# #     y = [ref[1]]
# #     x2 = ref[0] + w * np.cos(th)
# #     y2 = ref[1] + w * np.sin(th)
# #     x.append(x2); y.append(y2)
# #     x3 = x2 - h * np.sin(th)
# #     y3 = y2 + h * np.cos(th)
# #     x.append(x3); y.append(y3)
# #     x4 = x3 - w * np.cos(th)
# #     y4 = y3 - w * np.sin(th)
# #     x.append(x4); y.append(y4)
# #     x.append(ref[0]); y.append(ref[1])
# #     return x, y


# # def generate_vertex2(ref, w, h, th):
# #     """右手の各リンクを生成"""
# #     x = [ref[0]]
# #     y = [ref[1]]
# #     x2 = ref[0] - w * np.cos(th)
# #     y2 = ref[1] - w * np.sin(th)
# #     x.append(x2); y.append(y2)
# #     x3 = x2 + h * np.sin(th)
# #     y3 = y2 - h * np.cos(th)
# #     x.append(x3); y.append(y3)
# #     x4 = x3 + w * np.cos(th)
# #     y4 = y3 + w * np.sin(th)
# #     x.append(x4); y.append(y4)
# #     x.append(ref[0]); y.append(ref[1])
# #     return x, y


# # # === 入力 ===
# # cluster_date = input("クラスタフォルダの日付 (例: 2025-10-07): ").strip()
# # cluster_dir = f"../ICM_Log/path/{cluster_date}/"
# # if not os.path.exists(cluster_dir):
# #     raise FileNotFoundError(f"クラスタフォルダが見つかりません: {cluster_dir}")

# # angle_name = input("ハンド角度ファイル名 (.csv抜きで入力): ").strip()
# # angle_path = f"../ICM_Log/path/{angle_name}.csv"
# # if not os.path.exists(angle_path):
# #     raise FileNotFoundError(f"角度ファイルが見つかりません: {angle_path}")

# # # === データ読み込み ===
# # df = pd.read_csv(angle_path, header=None, skiprows=[0])  # ハンド角度
# # cluster_files = sorted(glob.glob(os.path.join(cluster_dir, "clusters_*.csv")))

# # if len(cluster_files) == 0:
# #     raise RuntimeError(f"{cluster_dir} にクラスタCSVが見つかりません。")

# # print(f"検出: {len(cluster_files)} フレーム")

# # # === ハンド寸法 ===
# # height = [39, 132, 130, 90]
# # width  = [60, 54, 54, 54]

# # # === 可視化ループ ===
# # fig = plt.figure(figsize=(6, 8))

# # for i in range(min(len(df), len(cluster_files))):
# #     ax = fig.add_subplot(1, 1, 1)
# #     ax.set(xlim=(-250, 250), ylim=(-10, 500))
# #     ax.set_aspect('equal')
# #     ax.set_title(f"Frame {i}")

# #     # ---- クラスタ点群を描画 ----
# #     cluster_df = pd.read_csv(cluster_files[i])
# #     for cluster_id, group in cluster_df.groupby("cluster_id"):
# #         ax.scatter(group["x"], group["y"], s=10, alpha=0.5, label=f"C{cluster_id}")
# #     # 凡例（クラスタ数多い場合は非表示でもOK）
# #     if len(cluster_df["cluster_id"].unique()) <= 5:
# #         ax.legend(loc='upper right', fontsize=8)

# #     # ---- ハンドを描画 ----
# #     t1, t2, t3, t4, t5, t6 = df.iloc[i, :6]

# #     # 左手
# #     AbsAngle = 0.0
# #     center = [-30, 0]
# #     ref = [center[0]-0.5*width[0], center[1]]
# #     x, y = generate_vertex(ref, width[0], height[0], 0.0)
# #     ax.plot(x, y, color='black')
# #     center[0] -= height[0]*np.sin(0)
# #     center[1] += height[0]*np.cos(0)

# #     AbsAngle += t1
# #     for theta, w, h in zip([t1, t2, t3], width[1:], height[1:]):
# #         AbsAngle += theta
# #         rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]-0.5*w*np.cos(rad), center[1]-0.5*w*np.sin(rad)]
# #         x, y = generate_vertex(ref, w, h, rad)
# #         ax.plot(x, y, color='black')
# #         center[0] -= h*np.sin(rad)
# #         center[1] += h*np.cos(rad)

# #     # 右手
# #     AbsAngle = 0.0
# #     center = [30.01, 415]
# #     ref = [center[0]+0.5*width[0], center[1]]
# #     x, y = generate_vertex2(ref, width[0], height[0], 0.0)
# #     ax.plot(x, y, color='black')
# #     center[0] += height[0]*np.sin(0)
# #     center[1] -= height[0]*np.cos(0)

# #     for theta, w, h in zip([t4, t5, t6], width[1:], height[1:]):
# #         AbsAngle += theta
# #         rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]+0.5*w*np.cos(rad), center[1]+0.5*w*np.sin(rad)]
# #         x, y = generate_vertex2(ref, w, h, rad)
# #         ax.plot(x, y, color='black')
# #         center[0] += h*np.sin(rad)
# #         center[1] -= h*np.cos(rad)

# #     # ---- 表示更新 ----
# #     if i == len(df)-1:
# #         plt.show()
# #     else:
# #         plt.pause(0.1)
# #         plt.clf()

# # plt.close()



# # #!/usr/bin/env python
# # # coding: utf-8

# # import os
# # import glob
# # import numpy as np
# # import pandas as pd
# # import matplotlib.pyplot as plt
# # import math
# # from matplotlib.animation import FuncAnimation

# # # ===== 基本関数 =====
# # def deg_to_rad(deg):
# #     return deg * np.pi / 180.0

# # def generate_vertex(ref:list, w, h, th):
# #     x, y = [], []
# #     x.append(ref[0]); y.append(ref[1])
# #     x2 = ref[0]+w*np.cos(th); y2 = ref[1]+w*np.sin(th)
# #     x.append(x2); y.append(y2)
# #     x3 = x2 - h*np.sin(th); y3 = y2 + h*np.cos(th)
# #     x.append(x3); y.append(y3)
# #     x4 = x3 - w*np.cos(th); y4 = y3 - w*np.sin(th)
# #     x.append(x4); y.append(y4)
# #     x.append(ref[0]); y.append(ref[1])
# #     return x, y

# # def generate_vertex2(ref:list, w, h, th):
# #     x, y = [], []
# #     x.append(ref[0]); y.append(ref[1])
# #     x2 = ref[0]-w*np.cos(th); y2 = ref[1]-w*np.sin(th)
# #     x.append(x2); y.append(y2)
# #     x3 = x2 + h*np.sin(th); y3 = y2 - h*np.cos(th)
# #     x.append(x3); y.append(y3)
# #     x4 = x3 + w*np.cos(th); y4 = y3 + w*np.sin(th)
# #     x.append(x4); y.append(y4)
# #     x.append(ref[0]); y.append(ref[1])
# #     return x, y


# # # ===== メイン =====
# # def visualize_hand_and_clusters(path_dir, arm_csv):
# #     """
# #     path_dir: "../ICM_Log/path/2025-10-07" のようなディレクトリ
# #     arm_csv : "../ICM_Log/path/arm_angles.csv"
# #     """
# #     df_angles = pd.read_csv(arm_csv, header=None, skiprows=[0])
# #     cluster_files = sorted(glob.glob(os.path.join(path_dir, "clusters_*.csv")))

# #     print(f"Found {len(cluster_files)} cluster files in {path_dir}")
# #     print(f"Total hand frames: {len(df_angles)}")

# #     height = [38, 130, 130, 90]
# #     width = [60, 35, 35, 35]

# #     fig, ax = plt.subplots()
# #     ax.set(xlim=(-250,250), ylim=(-10,500))
# #     ax.set_aspect('equal')
# #     hand_lines = []  # ハンドを構成する線の描画オブジェクトを保持
# #     cluster_scatter = None

# #     def update(frame):
# #         ax.cla()
# #         ax.set(xlim=(-250,250), ylim=(-10,500))
# #         ax.set_aspect('equal')
# #         ax.set_title(f"Frame {frame}")

# #         # ==== クラスタ表示 ====
# #         if frame < len(cluster_files):
# #             cluster_df = pd.read_csv(cluster_files[frame], header=None)
# #             cluster_df.columns = ["x", "y", "theta"]
# #             colors = plt.cm.tab10(np.linspace(0, 1, cluster_df["theta"].nunique()))
# #             for i, th in enumerate(sorted(cluster_df["theta"].unique())):
# #                 subset = cluster_df[cluster_df["theta"] == th]
# #                 ax.scatter(subset["x"], subset["y"], s=10, color=colors[i], alpha=0.7)

# #         # ==== ハンド表示 ====
# #         t1, t2, t3, t4, t5, t6 = df_angles.iloc[min(frame, len(df_angles)-1)][0:6]

# #         # Left
# #         AbsAngle = 0.0
# #         center = [-30 ,0]
# #         ref = [center[0]-0.5*width[0], center[1]]
# #         x, y = generate_vertex(ref, width[0], height[0], 0.0)
# #         center[0] -= height[0]*np.sin(0)
# #         center[1] += height[0]*np.cos(0)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t1; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]-0.5*width[1]*np.cos(rad), center[1]-0.5*width[1]*np.sin(rad)]
# #         x, y = generate_vertex(ref, width[1], height[1], rad)
# #         center[0] -= height[1]*np.sin(rad)
# #         center[1] += height[1]*np.cos(rad)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t2; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]-0.5*width[2]*np.cos(rad), center[1]-0.5*width[2]*np.sin(rad)]
# #         x, y = generate_vertex(ref, width[2], height[2], rad)
# #         center[0] -= height[2]*np.sin(rad)
# #         center[1] += height[2]*np.cos(rad)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t3; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]-0.5*width[3]*np.cos(rad), center[1]-0.5*width[3]*np.sin(rad)]
# #         x, y = generate_vertex(ref, width[3], height[3], rad)
# #         ax.plot(x,y, 'k')

# #         # Right hand
# #         AbsAngle = 0.0
# #         center = [30.01 ,415]
# #         ref = [center[0]+0.5*width[0], center[1]]
# #         x, y = generate_vertex2(ref, width[0], height[0], 0.0)
# #         center[0] += height[0]*np.sin(0)
# #         center[1] -= height[0]*np.cos(0)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t4; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]+0.5*width[1]*np.cos(rad), center[1]+0.5*width[1]*np.sin(rad)]
# #         x, y = generate_vertex2(ref, width[1], height[1], rad)
# #         center[0] += height[1]*np.sin(rad)
# #         center[1] -= height[1]*np.cos(rad)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t5; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]+0.5*width[2]*np.cos(rad), center[1]+0.5*width[2]*np.sin(rad)]
# #         x, y = generate_vertex2(ref, width[2], height[2], rad)
# #         center[0] += height[2]*np.sin(rad)
# #         center[1] -= height[2]*np.cos(rad)
# #         ax.plot(x,y, 'k')

# #         AbsAngle += t6; rad = deg_to_rad(AbsAngle)
# #         ref = [center[0]+0.5*width[3]*np.cos(rad), center[1]+0.5*width[3]*np.sin(rad)]
# #         x, y = generate_vertex2(ref, width[3], height[3], rad)
# #         ax.plot(x,y, 'k')

# #         return []

# #     ani = FuncAnimation(fig, update, frames=len(cluster_files), interval=100, repeat=False)
# #     plt.show()


# # # ===== 実行部 =====
# # if __name__ == "__main__":
# #     print("例: ../ICM_Log/path/2025-10-07/")
# #     path_dir = input("Enter cluster folder path: ").strip()
# #     arm_csv = input("Enter arm_angles.csv path: ").strip()
# #     visualize_hand_and_clusters(path_dir, arm_csv)

#!/usr/bin/env python3
# coding: utf-8

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

# --- 入力 ---
cluster_date = input("クラスタフォルダの日付 (例: 2025-10-07): ").strip()
cluster_dir = f"../ICM_Log/path/{cluster_date}/"

if not os.path.isdir(cluster_dir):
    raise FileNotFoundError(f"クラスタフォルダが見つかりません: {cluster_dir}")

cluster_files = sorted(glob.glob(os.path.join(cluster_dir, "clusters_*.csv")))
if not cluster_files:
    raise RuntimeError(f"{cluster_dir} に clusters_*.csv が見つかりません。")

print(f"検出: {len(cluster_files)} フレーム")

# --- 軸範囲（手動指定） ---
X_RANGE = (-250, 250)
Y_RANGE = (-250, 250)
TH_RANGE = (0, 360)

# --- カラーマップ ---
cmap = plt.get_cmap("tab20")

# --- 描画設定 ---
fig = plt.figure(figsize=(7, 6))
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect((1, 1, 0.6))
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("θ")

def update(i):
    ax.cla()
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("θ")
    ax.set_xlim(X_RANGE)
    ax.set_ylim(Y_RANGE)
    ax.set_zlim(TH_RANGE)
    ax.view_init(elev=25, azim=-60)
    ax.set_title(f"Frame {i+1}/{len(cluster_files)}")

    # --- 空ファイルをスキップ ---
    if os.path.getsize(cluster_files[i]) == 0:
        print(f"[警告] 空ファイルをスキップ: {cluster_files[i]}")
        return

    try:
        df = pd.read_csv(cluster_files[i], skiprows=1, header=None)
    except pd.errors.EmptyDataError:
        print(f"[警告] 読み込み失敗（空）: {cluster_files[i]}")
        return

    if df.shape[1] < 4:
        return
    df.columns = ["cluster_id", "x", "y", "th"]

    # 範囲外データを除外
    df = df[
        (df["x"] >= X_RANGE[0]) & (df["x"] <= X_RANGE[1]) &
        (df["y"] >= Y_RANGE[0]) & (df["y"] <= Y_RANGE[1]) &
        (df["th"] >= TH_RANGE[0]) & (df["th"] <= TH_RANGE[1])
    ]

    if df.empty:
        return

    # 各クラスタごとに色分けしてプロット
    for j, (cid, g) in enumerate(df.groupby("cluster_id")):
        ax.scatter(g["x"], g["y"], g["th"], s=20, color=cmap(j % 20), label=f"C{int(cid)}")

# --- アニメーション作成 ---
ani = FuncAnimation(fig, update, frames=len(cluster_files), repeat=False)

save_path = os.path.join(cluster_dir, "cluster_3d.mp4")
print(f"動画を保存中: {save_path}")
ani.save(save_path, writer=FFMpegWriter(fps=10))
print("保存完了:", save_path)
plt.close(fig)

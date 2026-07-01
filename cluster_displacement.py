#!/usr/bin/env python3
"""
クラスター前後間の移動量を計算するスクリプト
指定されたフォルダ内の clusters_*.csv ファイルを読み込み、
クラスター間の最大最小距離 max_{q in C_old} min_{p in C_new} d(q,p) を計算する

使用方法: 実行するとプロンプトが表示され、
クラスターフォルダの日付 (例: 2025-10-07) か、
直接フォルダパスを入力するだけで処理される。
cluster_3d.py と同じディレクトリ構造を想定。
"""

import os
import csv
import math
import sys

def euclidean_distance_3d(p1, p2):
    """
    3次元点間のユークリッド距離を計算
    p1, p2: (x, y, theta) のタプル
    """
    dx = float(p1[0]) - float(p2[0])
    dy = float(p1[1]) - float(p2[1])
    dth = float(p1[2]) - float(p2[2])
    
    # 角度の差を 0～180 に正規化（360度周期のため）
    dth = abs(dth) % 360.0
    if dth > 180.0:
        dth = 360.0 - dth
    
    return math.sqrt(dx**2 + dy**2 + dth**2)

def load_cluster_csv(filepath):
    """
    clusters_XXX.csv ファイルを読み込み、ポイントリストを返す
    csv形式: cluster_id,x,y,theta
    """
    points = []
    try:
        with open(filepath, 'r', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    theta = float(row['theta'])
                    points.append((x, y, theta))
                except (ValueError, KeyError) as e:
                    print(f"[Warning] スキップ行（{filepath}）: {row}", file=sys.stderr)
        return points
    except Exception as e:
        print(f"[Error] CSVファイルが読み込めません: {filepath}", file=sys.stderr)
        return None

def calculate_cluster_displacement(cluster_old, cluster_new):
    """
    クラスター間の移動量を計算
    式: max_{q in C_old} min_{p in C_new} d(q,p)
    
    Args:
        cluster_old: 古いクラスターのポイントリスト
        cluster_new: 新しいクラスターのポイントリスト
    
    Returns:
        最大最小距離の値
    """
    if not cluster_old or not cluster_new:
        return None
    
    max_min_dist = -1.0
    
    # 古いクラスター内の各ポイント q について
    for q in cluster_old:
        # 新しいクラスター内の最も近いポイント p までの距離の最小値を計算
        min_dist = float('inf')
        for p in cluster_new:
            dist = euclidean_distance_3d(q, p)
            min_dist = min(min_dist, dist)
        
        # 最大値を取る
        max_min_dist = max(max_min_dist, min_dist)
    
    return max_min_dist

def main():
    print("クラスター移動量計算ツール")
    print("="*50)
    
    # クラスターフォルダの日付/名前を入力（cluster_3d.py と同じ方式）
    user_input = input("クラスターフォルダの日付またはパス (例: 2025-10-07): ").strip()
    # 入力が既に有効なディレクトリであればそれを使い、そうでなければ日付文字列扱い
    if os.path.isdir(user_input):
        folder_path = user_input
    else:
        # スクリプトと同じ場所から見た相対パス
        # cluster_3d.py と同様、../ICM_Log/path/{date}/ を想定
        folder_path = os.path.join(os.path.dirname(__file__), '..', 'ICM_Log', 'path', user_input)
        folder_path = os.path.normpath(folder_path)

    if not os.path.isdir(folder_path):
        print(f"[Error] フォルダが見つかりません: {folder_path}")
        return
    
    # フォルダ内の clusters_*.csv ファイルをリストアップ
    cluster_files = sorted([f for f in os.listdir(folder_path)
                           if f.startswith('clusters_') and f.endswith('.csv')])
    
    if not cluster_files:
        print(f"[Error] クラスターファイルが見つかりません: {folder_path}")
        return
    
    print(f"[Info] 見つかったファイル数: {len(cluster_files)}")
    for f in cluster_files:
        print(f"  - {f}")
    
    # クラスターを読み込む
    clusters = {}
    for filename in cluster_files:
        filepath = os.path.join(folder_path, filename)
        points = load_cluster_csv(filepath)
        if points is not None:
            clusters[filename] = points
            print(f"[Loaded] {filename}: {len(points)} points")
    
    if len(clusters) < 2:
        print("[Error] 最低2つ以上のクラスターが必要です")
        return
    
    # 連続するクラスター間の移動量を計算
    displacements = []
    sorted_files = sorted(clusters.keys())
    
    print("\n計算結果:")
    print("-"*50)
    
    for i in range(len(sorted_files) - 1):
        old_file = sorted_files[i]
        new_file = sorted_files[i + 1]
        
        cluster_old = clusters[old_file]
        cluster_new = clusters[new_file]
        # directed: old -> new（順方向）
        displacement_old_new = calculate_cluster_displacement(cluster_old, cluster_new)
        # directed: new -> old（逆方向）
        displacement_new_old = calculate_cluster_displacement(cluster_new, cluster_old)
        # use the smaller (more representative) and also report both for diagnosis
        displacement_min = min(displacement_old_new, displacement_new_old)
        displacements.append((displacement_old_new, displacement_new_old, displacement_min))

        print(f"{old_file} -> {new_file}: old->new={displacement_old_new:.6f}, new->old={displacement_new_old:.6f}, min={displacement_min:.6f}")
    
    # 結果をCSVファイルに保存
    output_csv = os.path.join(folder_path, "cluster_displacements.csv")
    try:
        with open(output_csv, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            # Write columns: directed old->new, directed new->old, min
            writer.writerow(["disp_old_new", "disp_new_old", "disp_min"])
            for disp in displacements:
                # disp is a tuple (old_new, new_old, min)
                writer.writerow([f"{disp[0]:.6f}", f"{disp[1]:.6f}", f"{disp[2]:.6f}"])
        print(f"\n[Success] 結果を保存しました: {output_csv}")
    except Exception as e:
        print(f"[Error] CSVファイルの保存に失敗しました: {e}")

if __name__ == "__main__":
    main()

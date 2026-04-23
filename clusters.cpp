#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <filesystem>
#include <chrono>
#include <iomanip>

#include "CFreeICS.h"
#include "Node.h"
#include "clusters.h"
#include "pathshortcut.h"

namespace fs = std::filesystem;

// ===========================
// クラスタCSV保存関数
// ===========================
void save_clusters(const PointCloud& cluster,
                   const std::string& filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs) {
        std::cerr << "[Error] Cannot open file: " << filepath << std::endl;
        return;
    }

    ofs << "cluster_id,x,y,theta\n";
    for (const auto& pt : cluster) {
        ofs << 0 << "," << pt.x << "," << pt.y << "," << pt.th << "\n";
    }
    ofs.close();
    std::cout << "[Saved] " << filepath << std::endl;
}

// ===========================
// Node CSV読み込み関数
// ===========================
std::vector<Node> load_nodes_from_csv(const std::string& filename)
{
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "[Error] Cannot open node CSV: " << filename << std::endl;
        return {};
    }

    std::vector<Node> nodes;
    std::string line;
    bool first_line = true;
    while (std::getline(ifs, line)) {
        // BOM (UTF-8 Byte Order Mark) を削除
        if (first_line && line.size() >= 3) {
            if ((unsigned char)line[0] == 0xEF && 
                (unsigned char)line[1] == 0xBB && 
                (unsigned char)line[2] == 0xBF) {
                line = line.substr(3);
            }
            first_line = false;
        }
        
        // 末尾の \r\n や空白を削除
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' ' || line.back() == '\t')) {
            line.pop_back();
        }
        
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::vector<double> vals;
        std::string item;
        while (std::getline(ss, item, ',')) {
            // 空白文字を削除
            while (!item.empty() && (item.front() == ' ' || item.front() == '\t')) {
                item = item.substr(1);
            }
            while (!item.empty() && (item.back() == ' ' || item.back() == '\t')) {
                item.pop_back();
            }
            
            if (!item.empty()) {
                try {
                    vals.push_back(std::stod(item));
                } catch (const std::invalid_argument& e) {
                    std::cerr << "[Error] Cannot convert to double: '" << item << "' in line: " << line << std::endl;
                    throw;
                }
            }
        }
        if (vals.size() == Node::dof) {
            nodes.emplace_back(vals);
        } else {
            std::cerr << "[Warning] Invalid DOF line skipped: " << line << " (got " << vals.size() << " values, expected " << Node::dof << ")" << std::endl;
        }
    }
    std::cout << "[Loaded] " << nodes.size() << " nodes from " << filename << std::endl;
    return nodes;
}

// ===========================
// メイン処理
// ===========================
void run_cluster_extraction()
{
    // 1. 入力CSVファイル指定
    std::string input_csv;
    std::cout << "アーム角度CSVファイル名を入力してください: ";
    std::cin >> input_csv;

    input_csv = "../ICM_Log/path/" + input_csv + ".csv";

    // 2. 出力フォルダ作成
    std::string base_dir = "../ICM_Log/path/";

    // 日付フォルダ名を生成 (例: "2025-10-07")
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%Y-%m-%d");
    std::string date_folder = oss.str();

    fs::path save_dir = fs::path(base_dir) / date_folder;
    fs::create_directories(save_dir); // 再帰的に作成

    std::cout << "[Init] Save directory: " << save_dir << std::endl;

    // 3. ノード列読み込み
    auto nodes = load_nodes_from_csv(input_csv);
    if (nodes.empty()) return;

    // 4. 最初のノードでCFreeICSクラスタ抽出
    PointCloud prev_cluster;
    if (nodes.size() > 0) {
        Node node = nodes[0];
        CFreeICS cfree(node);

        std::cout << "[Extract] Node 0" << std::endl;
        auto clusters = cfree.extract();

        // クラスタを表示
        for (size_t i = 0; i < clusters.size(); ++i) {
            std::cout << i << ":\n" << clusters[i] << std::endl;
        }

        // ユーザーにクラスタを選ばせる
        std::cout << "Select the cluster No." << std::endl;
        std::cout << "-> ";
        int cls = 0;
        std::cin >> cls;

        if (cls < 0 || cls >= (int)clusters.size()) {
            std::cerr << "[Error] Invalid cluster number." << std::endl;
            return;
        }

        prev_cluster = clusters[cls];

        // 保存先パス
        std::ostringstream fname;
        fname << "clusters_" << std::setw(3) << std::setfill('0') << 1 << ".csv";
        fs::path filepath = save_dir / fname.str();

        // CSV出力
        save_clusters(prev_cluster, filepath.string());
    }

    // 5. 残りのノードでクラスタ追跡
    for (size_t i = 1; i < nodes.size(); ++i) {
        Node node = nodes[i];
        CFreeICS cfree(node);

        std::cout << "[Extract] Node " << i << std::endl;
        auto clusters = cfree.extract();

        if (clusters.empty()) {
            std::cerr << "[Warning] No clusters found for node " << i << ". Skipping." << std::endl;
            continue;
        }

        // 前のクラスタと最も重なり率が高いクラスタを探す
        double max_overlap = -1.0;
        size_t best_idx = 0;
        for (size_t j = 0; j < clusters.size(); ++j) {
            double overlap = overlap_ratio(prev_cluster, clusters[j]);
            if (overlap > max_overlap) {
                max_overlap = overlap;
                best_idx = j;
            }
        }

        std::cout << "[Track] Best overlap: " << max_overlap << " at cluster " << best_idx << std::endl;

        prev_cluster = clusters[best_idx];

        // 保存先パス
        std::ostringstream fname;
        fname << "clusters_" << std::setw(3) << std::setfill('0') << i+1 << ".csv";
        fs::path filepath = save_dir / fname.str();

        // CSV出力
        save_clusters(prev_cluster, filepath.string());
    }

    std::cout << "\n[Done] All tracked clusters saved in: " << save_dir << std::endl;
}

// ===========================
// クラスター移動量計算関数
// ===========================
// 3次元ユークリッド距離を計算（State3Dは int x, y, th）
double calculate_distance_3d(const State3D& p1, const State3D& p2)
{
    double dx = static_cast<double>(p1.x - p2.x);
    double dy = static_cast<double>(p1.y - p2.y);
    double dth = static_cast<double>(p1.th - p2.th);

    // 角度の差を 0～180 に正規化（360度周期のため）
    dth = std::fmod(std::abs(dth), 360.0);
    if (dth > 180.0) dth = 360.0 - dth;

    return std::sqrt(dx * dx + dy * dy + dth * dth);
}

// max_{q in C_old} min_{p in C_new} d(q, p) を計算
double calculate_cluster_displacement(const PointCloud& cluster_old, const PointCloud& cluster_new)
{
    if (cluster_old.empty() || cluster_new.empty()) {
        return -1.0;
    }

    double max_min_dist = -1.0;

    // 古いクラスター内の各ポイント q について
    for (const auto& q : cluster_old) {
        // 新しいクラスター内の最も近いポイント p までの距離の最小値を計算
        double min_dist = std::numeric_limits<double>::infinity();
        for (const auto& p : cluster_new) {
            double dist = calculate_distance_3d(q, p);
            min_dist = std::min(min_dist, dist);
        }

        // 最大値を取る
        max_min_dist = std::max(max_min_dist, min_dist);
    }

    return max_min_dist;
}

// クラスター移動量を計算して CSV に保存する関数
void calculate_and_save_displacements()
{
    // 1. フォルダパスを入力
    std::string folder_path;
    std::cout << "クラスターCSVファイルが格納されているフォルダパスを入力してください: ";
    std::getline(std::cin, folder_path);

    if (!fs::is_directory(folder_path)) {
        std::cerr << "[Error] フォルダが見つかりません: " << folder_path << std::endl;
        return;
    }

    // 2. フォルダ内の clusters_*.csv ファイルをリストアップ
    std::vector<std::string> cluster_files;
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (filename.find("clusters_") == 0 && filename.find(".csv") != std::string::npos) {
                cluster_files.push_back(filename);
            }
        }
    }

    std::sort(cluster_files.begin(), cluster_files.end());

    if (cluster_files.size() < 2) {
        std::cerr << "[Error] 最低2つ以上のクラスターが必要です" << std::endl;
        return;
    }

    std::cout << "[Info] 見つかったファイル数: " << cluster_files.size() << std::endl;

    // 3. クラスターを読み込む
    std::map<std::string, PointCloud> clusters;
    for (const auto& filename : cluster_files) {
        std::string filepath = (fs::path(folder_path) / filename).string();
        std::ifstream ifs(filepath);
        if (!ifs) {
            std::cerr << "[Warning] ファイルが開けません: " << filepath << std::endl;
            continue;
        }

        PointCloud pc;
        std::string line;
        bool first_line = true;
        while (std::getline(ifs, line)) {
            if (first_line) {
                first_line = false;
                continue; // ヘッダーをスキップ
            }

            // 末尾の改行・空白を削除
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
                line.pop_back();
            }

            if (line.empty()) continue;

            std::stringstream ss(line);
            std::string cluster_id, x_str, y_str, th_str;
            std::getline(ss, cluster_id, ',');
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, th_str, ',');

            try {
                int x = std::stoi(x_str);
                int y = std::stoi(y_str);
                int th = std::stoi(th_str);
                pc.push(State3D(x, y, th));
            } catch (const std::exception& e) {
                std::cerr << "[Warning] スキップ行（" << filepath << "）: " << line << std::endl;
            }
        }

        if (!pc.empty()) {
            clusters[filename] = pc;
            std::cout << "[Loaded] " << filename << ": " << pc.size() << " points" << std::endl;
        }
    }

    // 4. 連続するクラスター間の移動量を計算
    std::vector<double> displacements;
    auto sorted_files = cluster_files;
    std::sort(sorted_files.begin(), sorted_files.end());

    std::cout << "\n計算結果:" << std::endl;
    std::cout << "-" << std::string(49, '-') << std::endl;

    for (size_t i = 0; i < sorted_files.size() - 1; ++i) {
        const auto& old_file = sorted_files[i];
        const auto& new_file = sorted_files[i + 1];

        if (clusters.find(old_file) == clusters.end() || clusters.find(new_file) == clusters.end()) {
            std::cerr << "[Warning] クラスター読み込み失敗: " << old_file << " または " << new_file << std::endl;
            continue;
        }

        const auto& cluster_old = clusters[old_file];
        const auto& cluster_new = clusters[new_file];

        double displacement = calculate_cluster_displacement(cluster_old, cluster_new);
        displacements.push_back(displacement);

        std::cout << old_file << " -> " << new_file << ": " << displacement << std::endl;
    }

    // 5. 結果をCSVファイルに保存
    std::string output_csv = (fs::path(folder_path) / "cluster_displacements.csv").string();
    std::ofstream ofs(output_csv);
    if (!ofs) {
        std::cerr << "[Error] CSVファイルが開けません: " << output_csv << std::endl;
        return;
    }

    ofs << "displacement\n";
    for (double disp : displacements) {
        ofs << std::fixed << std::setprecision(6) << disp << "\n";
    }
    ofs.close();

    std::cout << "\n[Success] 結果を保存しました: " << output_csv << std::endl;
}

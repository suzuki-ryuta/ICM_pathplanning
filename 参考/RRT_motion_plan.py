#参考サイト https://myenigma.hatenablog.com/entry/2016/03/23/092002
import math
import copy
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import collections
import time
from numpy import linalg as LA
from mpl_toolkits.mplot3d import Axes3D
from sklearn.cluster import KMeans
from sklearn import cluster, datasets, mixture
from sklearn.metrics import silhouette_score
from sklearn.neighbors import kneighbors_graph

global CAL_NUM
CAL_NUM=0
#ここからクラス--------------------------------------------------------------------------------------------------------------------------------------
# xy平面上の点を表す．
class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

#(x,y,theta)空間上の点を表す
class State3D:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

#ロボットアームの状態を表す．RRT用のクラス(theta1,theta2,theta3,theta4),parentは，path製作用の変数
class Node:
    def __init__(self, theta1, theta2, theta3, theta4):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        self.parent = None

#2次元平面上の2点を引数とし，それらを結ぶ線分を表す．また，それら同士の干渉判定を行う関数を含む．        
class Segment:
    def __init__(self, start: Point2D, goal: Point2D):
        self.start = start
        self.goal = goal
    def judge(self, p1,p2,p3,p4):
        global CAL_NUM
        CAL_NUM+=1
        t1 = (p1.x - p2.x) * (p3.y - p1.y) + (p1.y - p2.y) * (p1.x - p3.x)
        t2 = (p1.x - p2.x) * (p4.y - p1.y) + (p1.y - p2.y) * (p1.x - p4.x)
        t3 = (p3.x - p4.x) * (p1.y - p3.y) + (p3.y - p4.y) * (p3.x - p1.x)
        t4 = (p3.x - p4.x) * (p2.y - p3.y) + (p3.y - p4.y) * (p3.x - p2.x)
        #線分の交差あり→True 交差なし→False
        if (t1 * t2 < 0 and t3 * t4 < 0):
            return True      
        else:
            return False
    def seg_collides(self, another):
        #anotherはsegment型変数
        #線分の交差あり→True 交差なし→False
        if (self.judge(self.start, self.goal, another.start, another.goal) == True):
            return True
        else:
            return False

#図形の基準点(x,y)，図形全体の傾き，辺の長さ，各辺間の角度を受け取り，一つの多角形を表す．多角形同士の干渉判定を行う関数を含む．
class Polygon:
    def __init__(self, base: Point2D, direction: float, vertex_lengthes, angles_between_segments):
        #
        # calculate 
        #
        self.base = base
        self.direction = direction
        self.vertex_lengthes = vertex_lengthes
        self.angles_between_segments = angles_between_segments
        #self.segmentは，Polygon内に存在する辺を表すSegment型変数のリスト
        self.segments=[]
        current_position = base
        current_direction = direction
        #print("vertex_lengthes", vertex_lengthes)
        for i in range(0, len(self.vertex_lengthes), 1):
            next_position=copy.deepcopy(current_position)
            next_position.x += self.vertex_lengthes[i] * np.cos(current_direction)
            next_position.y += self.vertex_lengthes[i] * np.sin(current_direction)
            #current_positionをスタートとし，next_positionをゴールとする辺を，Segment型変数"segment"として指定
            segment = Segment(current_position, next_position)
            #Segment型変数segmentを，self.segmentsリストに追加
            self.segments.append(segment)
            current_direction += self.angles_between_segments[i]
            current_position = next_position
    def collides(self, other):
        #線分の交差あり→True 交差なし→False
        for self_segment in self.segments:
            for other_segment in other.segments:
                if (self_segment.seg_collides(other_segment)):
                    return True
        return False
    def collides_with_others(self, others):
        #線分の交差あり→True 交差なし→False
        for other in others:
            if (self.collides(other)):
                return True
        return False

#ロボットハンド作成のためのクラス．リンクの長さと幅を引数とし，一つのリンクを表す．
class Link:
    def __init__(self, length, width):
        self.length = length
        self.width = width

#ロボットハンド作成のためのクラス．リンク間の角度を引数とし，一つのジョイントを表す．
class Joint:
    def __init__(self,angle):
        self.angle = angle

#ロボットハンド作成のためのクラス．LinkクラスとJointクラスのインスタンスを引数とし，一つのアームを表す
class Arm:
    def __init__(self, link: Link, joint: Joint):
        self.link = link
        self.joint = joint    

#ロボットハンド作成のためのクラス．ハンドの原点，ハンド全体の傾き，Armクラスのインスタンスのリストを引数とし，四角形で構成されるロボットのPolygonインスタンスたちを作る(!!!四角形専用!!!)．
class Robot:
    def __init__(self, origin:Point2D, direction, arms: list):
        self.origin = origin
        self.arms = arms
    def create_polygons(self):
        base_point = self.origin
        central_point = copy.deepcopy(base_point)
        direction = degree_to_radian(0)
        polygons = []
        for arm in self.arms:
            #next_reference_point = copy.deepcopy(current_reference_point)            
            direction += arm.joint.angle
            reference_point=Point2D(central_point.x - 1 / 2 * arm.link.width * np.cos(direction), central_point.y - 1 / 2 * arm.link.width * np.sin(direction))
            vertex_lengthes = [arm.link.width, arm.link.length, arm.link.width, arm.link.length]
            angles=[degree_to_radian(90), degree_to_radian(90), degree_to_radian(90), degree_to_radian(90)]          
            polygons.append(Polygon(copy.deepcopy(reference_point), direction, vertex_lengthes, angles))
            #print("current (x,y)", (central_point.x, central_point.y))
            #print("current (direction)", direction)
            #print("current (vertex_lengthes)", vertex_lengthes)
            #print("current (angles)", angles)
            central_point.x -= arm.link.length * np.sin(direction)
            central_point.y += arm.link.length * np.cos(direction)
        return polygons
    def collide_with_other_robot(self, other):
        self_polygons = self.create_polygons()
        other_polygons = other.create_polygons()
        for self_polygon in self_polygons:
            if (self_polygon.collides_with_others(other_polygons)):
                return True
        return False 

#RRTを行うクラス．goalを加える場合には，定義の段階で追加を 
class RRT:
    def __init__(self, start: Node, goal:State3D, random_sampling_area, expandation_distance, trial_times,epsilon):
        self.start = start
        self.goal = goal
        self.minimum_sampling_area = random_sampling_area[0]
        self.maximum_sampling_area = random_sampling_area[1]
        self.expandation_distance = expandation_distance
        self.trian_times = trial_times
        self.epsilon = epsilon
    
    def planning(self):
        #nodelistは，Nodeクラスインスタンスのリスト
        self.nodelist = [self.start]
        trial_time = 0
        last_node = copy.deepcopy(self.start)
        node_num = 0
        #物体の初期存在領域!!!スタート姿勢に応じて必ず変えること!!!
        current_region = [[[100, 50, -10], [100, 50, 0]]]
        outside_point = [State3D(0, 320, 0), State3D(0, 320, 60), State3D(300, 320, 0), State3D(300, 320, 60)]
        outside_point = [[290, 310, 0], [0, 310, 60], [290, 310, 0], [290, 310, 60]]
        
        cnt = 0
        while True:
            cnt = cnt + 1
            t1=time.time()
            print("-------------------------------------------------------------------------------------------------")
            print(cnt)
            #規定領域の中からランダムに点を一つ設定
            random_node = Node(random.uniform(self.minimum_sampling_area, self.maximum_sampling_area), random.uniform(self.minimum_sampling_area, self.maximum_sampling_area), random.uniform(self.minimum_sampling_area, self.maximum_sampling_area), random.uniform(self.minimum_sampling_area, self.maximum_sampling_area))
            #nodelist(一回目はスタートの座標のみ)に含まれる点の中から，ランダムに設定した点に最も近い点が，nodelistの何番目に存在するかを特定する
            nind = self.get_nearest_node_index(self.nodelist, random_node)
            #ランダム点に最も近い点をnearest_nodeとして定める
            nearest_node = self.nodelist[nind]
            #print("random.uniform",random.uniform(self.minimum_sampling_area, self.maximum_sampling_area))
            #print("nearestnode.theta1",nearest_node.theta1)
            #nearest_node通して定められた点をnew_nodeと新たに定義し直す
            new_node = copy.deepcopy(nearest_node)

            #new_nodeはNode型変数なので，これを配列に書き直す          
            new_node_vector = np.array([new_node.theta1, new_node.theta2, new_node.theta3, new_node.theta4])
            #ランダム点も同様に配列に書き直す
            random_node_vector = np.array([random_node.theta1, random_node.theta2, random_node.theta3, random_node.theta4])
            #print("new node vector", new_node_vector)
            #print("random node vector", random_node_vector)
            #new_node(nearest_node)→random_nodeのベクトルを正規化する
            unit_vector = (random_node_vector - new_node_vector) / (LA.norm(random_node_vector - new_node_vector))
            #print("unit vector", unit_vector)

            #new_nodeからrandom_nodeの方向に，長さexpandation_distanceだけ枝を伸ばす
            new_node.theta1 += unit_vector[0] * self.expandation_distance
            new_node.theta2 += unit_vector[1] * self.expandation_distance
            new_node.theta3 += unit_vector[2] * self.expandation_distance
            new_node.theta4 += unit_vector[3] * self.expandation_distance
            new_node.parent = nind
            #print("newed theta1", new_node.theta1)
            #関節角の可動範囲の設定
            if not (new_node.theta1 >= -90 and new_node.theta1 <= 90):
                print("theta1 break")
                continue
            if not (new_node.theta2 >= -90 and new_node.theta2 <= 90):
                print("theta2 break")
                continue
            if not (new_node.theta3 >= -90 and new_node.theta3 <= 90):
                print("theta3 break")
                continue
            if not (new_node.theta4 >= -90 and new_node.theta4 <= 90):
                print("theta4 break")
                continue
            
            #新たな関節角候補が決定したので，この関節角(ハンド状態)について，物体のコンフィグレーション空間を計算し，存在領域を特定する
            #-----------------------------------------------------------------------------------------------------------------
            #判定範囲，ハンド，対象物，壁などの諸値の設定
            exploration_area_bottom = State3D(0, 0, -60)
            exploration_area_top = State3D(350, 320, 60)
            range_x = 10
            range_y = 10
            range_theta=10

            base_triangle = Point2D(200.0, 25.0)
            base_wall = Point2D(150.0, -5.0)
            #三角形の大きさは，ここで指定
            triangle_size=100
            triangle = create_equilateraltriangle(base_triangle, degree_to_radian(0.0),  triangle_size)
            wall = create_rectangle(base_wall, degree_to_radian(0.0), 320.0, 5.0)

            #関節角情報をここで指定
            link0L = Link(38, 60)
            joint0L = Joint(degree_to_radian(0.0))
            arm0L = Arm(link0L, joint0L)

            link1 = Link(130, 35)
            joint1 = Joint(degree_to_radian(new_node.theta1))
            arm1 = Arm(link1, joint1)
    
            link2 = Link(90, 35)
            joint2 = Joint(degree_to_radian(new_node.theta2))
            arm2 = Arm(link2, joint2)

            link0R = Link(38, 60)
            joint0R = Joint(degree_to_radian(0.0))
            arm0R = Arm(link0R, joint0R)

            link3 = Link(130, 35)
            joint3 = Joint(degree_to_radian(new_node.theta3))
            arm3 = Arm(link3, joint3)

            link4 = Link(90, 35)
            joint4 = Joint(degree_to_radian(new_node.theta4))
            arm4 = Arm(link4, joint4)

            #関節の根元の位置をここで指定
            origin_A = Point2D(0.0, 0.0)
            origin_B = Point2D(320, 0)
            direction = degree_to_radian(90)
            robot_A = Robot(origin_A, direction, [arm0L, arm1, arm2])
            robot_B = Robot(origin_B, direction, [arm0R, arm3, arm4])
            #現在の状態を描画する
            polygons_list = []
            polygons_list.append(wall)
            #polygons_list.append(triangle)
            for i in robot_A.create_polygons():
                polygons_list.append(i)
            for i in robot_B.create_polygons():
                polygons_list.append(i)
            #候補となっている現在の関節角状態を描画するコマンド
            #draw_polygons(polygons_list)
            #-----------------------------------------------------------------------------------------
            
            #現在の関節角状態の定義は終了したので，ここからは，幾何学的判定によって，ケージングを満たしていない姿勢をあらかじめ排除するための作業を行う
            if self.cagingcheck(arm0L, arm1, arm2, arm0R, arm3, arm4, origin_A, origin_B, triangle_size) == False:
                print("No caging")
                continue
            
            #-----------------------------------------------------------------------------------------
           
            
            #干渉の判定
            if triangle.collides_with_others(robot_A.create_polygons()):
                #正三角形とロボットAとの干渉判定(初期位置での干渉判定)
                print("collision")
            elif triangle.collides_with_others(robot_B.create_polygons()):
                #正三角形とロボットBとの干渉判定(初期位置での干渉判定)
                print("collision")
            elif wall.collides_with_others(robot_A.create_polygons()):
                #壁とロボットAとの干渉判定
                print("collision (robot-wall)")
                #continue
            elif wall.collides_with_others(robot_B.create_polygons()):
                #壁とロボットBとの干渉判定
                print("collision (robot-wall)")
                #continue
            elif triangle.collides(wall):
                #正三角形と壁との干渉判定
                print("collision")
            else:
                print("no collision")

            #ロボットAとロボットBとの干渉判定(干渉が確認された場合は，やり直しでwhile文の先頭まで戻る)
            if robot_A.collide_with_other_robot(robot_B):
                print("robot collision (robot-robot)")
                continue
            elif robot_B.collide_with_other_robot(robot_A):
                print("robot collision (robot-robot)")
                continue
            else:
                print("no robot collision")
            
            
            a = Configuration(triangle, robot_A, robot_B, wall, exploration_area_bottom, exploration_area_top, range_x, range_y, range_theta, outside_point)
            #物体の現在の存在領域をbとする
            b = a.get_common_region(current_region)
            #クラスタ数が1だった場合，それ以降の処理は中止
            if (b == False):
                continue

            #print("Segmentクラスjudge関数の呼び出しは%d回" % CAL_NUM)
            t2 = time.time()
            print("今ステップの所要時間:", t2 - t1)

            #物体の現在の存在領域を，ケージングとマニピュレーション可能判定にかける
            check = a.judge(b)
            #ケージング・マニピュレーション可能判定
            if (check == True):
                #ケージング・マニピュレーション可能判定で引っかかったので，やり直し
                continue
            else:
                #ケージング・マニピュレーション可能判定をパスしたので，ゴール判定を行う
                goal = a.goal_judge(b, self.goal, self.epsilon)
                pass
            
            #ヒューリスティックの判定
            eps = 70
            dif = a.cluster_number_judge(b, current_region, eps)
            if (dif == True):
                print("ヒューリスティック判定fail")
                continue
            else:
                print("ヒューリスティック判定pass")
                pass
            
            #-----------------------------------------------------------------------------------------------------------------
            #ここまで来たものは，姿勢として認められるので，ノードリストに現在の姿勢を書き込む
            node_num += 1
            print("node",node_num,new_node.theta1,new_node.theta2,new_node.theta3,new_node.theta4)
           
            #判定をクリアした場合，new_nodeを，nodelistに追加
            self.nodelist.append(new_node)
            trial_time += 1
            #あまり意味はない
            last_node = new_node
            '''
            if (trial_time > 10):
                break
            '''
            if(goal==False):
                print("-------------------------------------------------------------------------------------------------")
                print("trial finish")
                print("goal(x,y,theta)=(%f,%f,%f),epsilon=%f" % (self.goal.x,self.goal.y,self.goal.theta,self.epsilon))
                f.close 
                break
            else:
                print("NOT GOAL")
                #次の探索ステップに行くため，current_regionを更新
                current_region = b
                
            
        
        #探索が終了したので，最後にスタートからゴールまでのpathを作る
        path = []
        last_index = len(self.nodelist) - 1
        while self.nodelist[last_index].parent is not None:
            node = self.nodelist[last_index]
            path.append([node.theta1, node.theta2, node.theta3, node.theta4])
            last_index = node.parent
        path.append([self.start.theta1, self.start.theta2, self.start.theta3, self.start.theta4])

        return path
        
    def get_nearest_node_index(self, nodelist, random_node):
        distance_list = [(node.theta1 - random_node.theta1)** 2 + (node.theta2 - random_node.theta2)** 2 + (node.theta3 - random_node.theta3)** 2 + (node.theta4 - random_node.theta4)** 2 for node in nodelist]
        minimum_index = distance_list.index(min(distance_list))
        return minimum_index

    #ある状態に対して，幾何学的にケージング成立の可否をチェックする関数
    def cagingcheck(self, arm0L: Arm, arm1: Arm, arm2: Arm, arm0R: Arm, arm3: Arm, arm4: Arm, origin_A: Point2D, origin_B: Point2D,  triangle_size):
        #calcurate each positions in start state
        L0 = arm0L.link.length
        L = origin_B.x
        x10=0
        y10=arm0L.link.length

        x20=origin_B.x
        y20=arm0R.link.length

        L1 = arm1.link.length
        L2 = arm2.link.length
        L3 = arm3.link.length
        L4 = arm4.link.length

        theta1 = arm1.joint.angle
        theta2 = arm2.joint.angle
        theta3 = arm3.joint.angle
        theta4 = arm4.joint.angle

        w=arm1.link.width

        x1=L1*np.sin(-theta1)
        y1=L0+L1*np.cos(-theta1)
        x_1NE=x1+w/2*np.cos(-theta1)
        y_1NE=y1-w/2*np.sin(-theta1)
        x_1SE=x10+w/2*np.cos(-theta1)
        y_1SE=y10-w/2*np.sin(-theta1)

        x2=x1+L2*np.sin(-theta1-theta2)
        y2=y1+L2*np.cos(-theta1-theta2)
        x_2NE=x2+w/2*np.cos(-theta1-theta2)
        y_2NE=y2-w/2*np.sin(-theta1-theta2)
        x_2SE=x1+w/2*np.cos(-theta1-theta2)
        y_2SE=y1-w/2*np.sin(-theta1-theta2)

        x3=L-L3*np.sin(theta3)
        y3=L0+L3*np.cos(theta3)
        x_3NW=x3-w/2*np.cos(theta3)
        y_3NW=y3-w/2*np.sin(theta3)
        x_3SW=x20-w/2*np.cos(theta3)
        y_3SW=y20-w/2*np.sin(theta3)

        x4=x3-L4*np.sin(theta3+theta4)
        y4=y3+L4*np.cos(theta3+theta4)
        x_4NW=x4-w/2*np.cos(theta3+theta4)
        y_4NW=y4-w/2*np.sin(theta3+theta4)
        x_4SW=x3-w/2*np.cos(theta3+theta4)
        y_4SW=y3-w/2*np.sin(theta3+theta4)

        position=np.array([[x1,x2,x3,x4],[y1,y2,y3,y4],[x_1NE,x_2NE,x_3NW,x_4NW],[y_1NE,y_2NE,y_3NW,y_4NW],[x_1SE,x_2SE,x_3SW,x_4SW],[y_1SE,y_2SE,y_3SW,y_4SW]])
        Sp1=np.array([x1,y1])
        Sp_1NE=np.array([x_1NE,y_1NE])
        Sp_1SE=np.array([x_1SE,y_1SE])
        Sp2=np.array([x2,y2])
        Sp_2NE=np.array([x_2NE,y_2NE])
        Sp_2SE=np.array([x_2SE,y_2SE])
        Sp_3=np.array([x3,y3])
        Sp_3NW=np.array([x_3NW,y_3NW])
        Sp_3SW=np.array([x_3SW,y_3SW])
        Sp_4=np.array([x4,y4])
        Sp_4NW=np.array([x_4NW,y_4NW])
        Sp_4SW=np.array([x_4SW,y_4SW])

        #print("position:",position)

        #caging check in this state
        Smindistance = self.mindis(Sp_2NE, Sp_2SE, Sp_1NE, Sp_1SE, Sp_4NW, Sp_4SW, Sp_3NW, Sp_3SW)
        print("頂点間最小距離は", Smindistance)
        if (0 <= Smindistance and Smindistance < triangle_size*np.sqrt(3)/2):
            return True #caging is established
        else:
            return False #caging is not established
    #cagingcheck関数のための関数
    def mindis(self,p_a0,p_a1,p_a2,p_a3,p_b0,p_b1,p_b2,p_b3):
        d1=np.linalg.norm(p_a0-p_b0)
        d2=np.linalg.norm(p_a0-p_b1)
        d3=np.linalg.norm(p_a0-p_b2)
        d4=np.linalg.norm(p_a0-p_b3)
        d5=np.linalg.norm(p_b0-p_a0)
        d6=np.linalg.norm(p_b0-p_a1)
        d7=np.linalg.norm(p_b0-p_a2)
        d8=np.linalg.norm(p_b0-p_a3)

        d=np.array([d1,d2,d3,d4,d5,d6,d7,d8])
        mind=min(d)
        return mind

#ある状態に関して，正三角形物体のコンフィグレーション空間の導出を行い，物体の存在領域の特定までを行う，さらに，ケージング条件，ケージングマニピュレーション可能条件の判定を行う関数を含めたクラス
class Configuration:
    def __init__(self, polygon: Polygon, polygons1: Robot, polygons2: Robot, wall: Polygon, exploration_area_bottom: State3D, exploration_area_top: State3D, range_x, range_y, range_theta, outside_point: list):
        self.polygon = polygon
        self.polygons1 = polygons1
        self.polygons2 = polygons2
        self.wall = wall
        self.exploration_area_bottom = exploration_area_bottom
        self.exploration_area_top = exploration_area_top
        self.range_x = range_x
        self.range_y = range_y
        self.range_theta = range_theta
        self.outside_point = outside_point

    #領域内で，対象物体の干渉をすべての点でチェックし，その点を描画
    def collisioncheck_for_all_state(self):
        t1 = time.time()
        config = []
        grid_list = []
        #x,y,thetaを一定量ずつ変化させながら，干渉判定を行う
        for k in range(self.exploration_area_bottom.x, self.exploration_area_top.x+1, self.range_x):
            for i in range(self.exploration_area_bottom.y, self.exploration_area_top.y+1, self.range_y):
                for theta in range(self.exploration_area_bottom.theta, self.exploration_area_top.theta + 1, self.range_theta):
                    grid_list.append([k, i, theta])
        #---------------------------------------------------------------------------------------------------------------------------
        all_polygons_list = []
        for polygon in self.polygons1.create_polygons():
            all_polygons_list.append(polygon)
        for polygon in self.polygons2.create_polygons():
            all_polygons_list.append(polygon)
        all_polygons_list.append(self.wall)
         #各状態について，all_polygons_listに含まれるPolygon型インスタンス一つ一つにアクセスしながら，対象物の中心とPolygonの中心の間の距離を測定する
        for grid in grid_list:
            k = grid[0]
            i = grid[1]
            theta = grid[2]
            base = Point2D(k, i)
            #基準点を重心から正三角形の頂点までもっていく処理!!!これらは，対象物が正三角形であるための処理である．
            Vx = base.x - self.polygon.vertex_lengthes[0] / np.sqrt(3) * np.cos(degree_to_radian(theta) + np.pi / 6)
            Vy = base.y - self.polygon.vertex_lengthes[0] / np.sqrt(3) * np.sin(degree_to_radian(theta) + np.pi / 6)
            reference_point = Point2D(Vx, Vy)           
            obj = Polygon(reference_point, degree_to_radian(theta), self.polygon.vertex_lengthes, self.polygon.angles_between_segments)
            radius_of_triangle = self.polygon.vertex_lengthes[0] / np.sqrt(3)

            has_any_collision = False
            for polygon in all_polygons_list:
                distance = distance_of_centers(Point2D(k, i), polygon.base, polygon.direction, polygon.vertex_lengthes[0], polygon.vertex_lengthes[1])
                radius_of_rectangle = (np.sqrt((polygon.vertex_lengthes[0] * polygon.vertex_lengthes[0]) + (polygon.vertex_lengthes[1] * polygon.vertex_lengthes[1]))) / 2
                #print("distance=", distance)
                #print("radius_of_rectangle + radius_of_triangle=",radius_of_rectangle + radius_of_triangle)
                if (distance < (radius_of_rectangle + radius_of_triangle)):
                    #print("check")
                    # check collision
                    has_any_collision = polygon.collides(obj)
                    if (has_any_collision):
                        break
            if (has_any_collision == False):
                config.append(State3D(k, i, theta))
        t2 = time.time()
        #print("干渉判定所要時間:", t2 - t1)
        #ここから描画フェーズ
        '''
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-20, 320)
        ax.set_ylim(-20, 320)
        ax.set_zlim(-60, 60)
        x = []
        y = []
        theta = []
        #polygonslist=[]
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("THETA")
        for c in config:
            x.append(c.x)
            y.append(c.y)
            theta.append(c.theta)
        ax.plot(x, y, theta, "o", color="#00aa00", ms=4, mew=0.5)
        plt.show()
        plt.pause(.5)
        plt.close()
        '''
        return config

    #存在可能とされた点をクラスタリングし，その結果を描画
    def dbscan_clustering_3D(self):
        t0 = time.time()
        C = self.collisioncheck_for_all_state()
        t1 = time.time()
        #State3Dのリストとしてあらわされていた存在可能領域を，フルリスト化する
        all_existable_point_3D = [len(C)]
        for i, config in enumerate(C):
            all_existable_point_3D[i] = [config.x, config.y, config.theta]
            all_existable_point_3D.append(all_existable_point_3D[i])
        #DBSCAN法
        db = cluster.DBSCAN(eps=15.0, min_samples=1, metric='euclidean')
        dataset = db.fit_predict(all_existable_point_3D)
        
        #datasetリストから，重複する要素を削除
        cluster_num = len(set(dataset))
        #print("dataset",dataset)
        #print("youso",collections.Counter(dataset))
        print("分類は%d種類" % cluster_num)

        #クラスタ数が1だった場合，これ以降の処理は無駄なので，Falseを返す
        if (cluster_num == 1):
            return False


        result = [[[] for _ in range(len(dataset))] for _ in range(len(set(dataset)))]
        
        pp=[]
        column_num = [0] * len(set(dataset))
        t2 = time.time()
        #ここから先のfor文内，3.8秒近くかかっている．
        #ここから高速化が必要
        #ここから高速化が必要
        #ここから高速化が必要
        #ここから高速化が必要
        #ここから高速化が必要
        #ここから高速化が必要
        for m, point in enumerate(all_existable_point_3D):
            for n in range(len(set(dataset))):  #nはクラスタ番号
                if dataset[m] == n:
                    result[n][column_num[n]].append(point[0])
                    result[n][column_num[n]].append(point[1])
                    result[n][column_num[n]].append(point[2])
                    column_num[n] += 1
        #ここまで高速化が必要
        #ここまで高速化が必要
        #ここまで高速化が必要
        #ここまで高速化が必要
        #ここまで高速化が必要
        #ここまで高速化が必要
        t3 = time.time()
        for s, p in enumerate(result): #mはクラスタ番号
            pp.append([w for w in p if w])  #リストpの中から，空の要素を削除
        
        time32 = t3 - t2
        time21 = t2 - t1
        time10 = t1 - t0
        #print(f"干渉判定関数呼出時間：{time10}")
        #print(f"クラスタリング時間：{time21}")
        #print(f"重複要素削除時間：{time32}")
        #ここから描画フェーズ
        ''' 
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim(-20, 320)
        ax.set_ylim(-20, 320)
        ax.set_zlim(-60, 60)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('THETA')
        colorlist = ["m", "y", "k", "w","r", "g", "b", "c"]
        for s, p in enumerate(pp): #mはクラスタ番号
            result_x = []
            result_y = []
            result_z = [] 
            for t, q in enumerate(p):
                result_x.append(q[0])
                result_y.append(q[1])
                result_z.append(q[2])
            ax.plot(result_x, result_y, result_z, marker="o", linestyle='None', color=colorlist[s % 8])
        plt.show()
        plt.pause(.5)
        plt.close()
        '''
        return pp

    #前の存在領域から，現在物体が存在するクラスタを特定し，それを描画
    def get_common_region(self, last_region: list):
        common_region_1 = []
        common_region_2 = []

        new_region = []
        exist_cluster_number = []
        t0 = time.time()
        
        current_clustered_region = self.dbscan_clustering_3D()
        #クラスタ数が1だった場合，本関数は処理を中止し，Falseを返す
        if (current_clustered_region == False):
            return False
        #
        t1 = time.time()
        for i, u in enumerate(current_clustered_region):
            for j, v in enumerate(last_region):
                for k, z in enumerate(v):
                    if (z in u):
                        common_region_1.append(u)
                        exist_cluster_number.append(i)
        for m, w in enumerate(common_region_1):
            common_region_2.append(list(map(tuple, w)))
        common_region_3 = list(map(tuple, common_region_2))
        subnew_region = list(set(list(map(tuple, common_region_3))))
        for n, x in enumerate(subnew_region):
            new_region.append(list(map(list, x)))
        #print("new region=", new_region)
        #new_regionは3次元リストなので，これを二次元にする
        exist_cluster_number=set(exist_cluster_number)
        for i, u in enumerate(exist_cluster_number):
            print("物体が存在する領域は，クラスタ",u)
        for i, u in enumerate(new_region):
            print("要素数は", len(u))
        t2 = time.time()
        time21 = t2 - t1
        time10 = t1 - t0
        #print(f"クラスタリング関数呼出時間：{time10}")
        #print(f"クラスタ特定時間：{time21}")
        #ここから描画フェーズ
        '''
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim(-20, 320)
        ax.set_ylim(-20, 320)
        ax.set_zlim(-60, 60)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('THETA')
        colorlist = ["r", "r", "r", "r", "r", "r", "r", "r"]
        for s, p in enumerate(new_region): #mはクラスタ番号
            x = []
            y = []
            z = [] 
            #print("p=", p)
            #P=copy.deepcopy(p)
            for t, q in enumerate(p):
                x.append(q[0])
                y.append(q[1])
                z.append(q[2])
            ax.plot(x, y, z, marker="o", linestyle='None', color=colorlist[s % 8])
        plt.show()
        plt.pause(.5)
        plt.close()
        '''
        #print(new_region)
        return new_region
    #--------------------------------------------------------------------------------------------------------------
    #ここからは，ケージング・ゴールなどの各種判定を行う関数．したがって，クラス内の他の関数を参照することはない．
    #ケージング条件，ケージングマニピュレーション可能条件のチェック
    def judge(self,new_region:list):
        #new_region = self.get_common_region()
        #print(new_region)
        #ケージング条件のチェック(物体の存在領域が，ハンド外部に設定した点を含んでいないことで確認)
        for i, u in enumerate(new_region):
            #print("u=",u)
            for j, v in enumerate(self.outside_point):
                #print("v=",v)
                if (v in u):
                    print("NO CAGING")
                    return True
        print("CAGING")
        #ケージングマニピュレーション可能条件のチェック(物体の存在領域のクラスタ数が1つであることで確認(改良の余地あり?))
        doublecount_of_cluster = 0
        '''
        #クラスタが上下に分割している場合の考慮
        theta0 = []
        theta120 = []
        for i, u in enumerate(new_region):
            for j, v in enumerate(u):
                if (0 in v):
                    theta0.append([v[0], v[1]])
                elif (120 in v):
                    theta120.append([v[0], v[1]])
        if (set(map(tuple, theta0)) == set(map(tuple, theta120))):
            doublecount_of_cluster += 1
        '''
        cluster_num = len(new_region)-doublecount_of_cluster              
                    
        print("cluster_num=", cluster_num)
        if not (cluster_num == 1):
            print("UN MANIPURABLE")
            return True
        print("MANIPURABLE")
        return False
    
    def goal_judge(self,new_region:list, goal: State3D, epsilon):
        #閾値以下であれば(ゴールであれば)，Falseを返す
        for i,u in enumerate(new_region):
            for j,v in enumerate(u):
                if(abs(v[0]-goal.x)>epsilon):
                    return True
                elif(abs(v[1]-goal.y)>epsilon):
                    return True
                elif(abs(v[2]-goal.theta)>epsilon):
                    return True
        return False

    #コンフィグレーション空間の急激な減少を抑えるためのヒューリスティック関数(引数は親領域と子領域，)
    def heuristic(self, child_region: list, parent_region: list, eps):
        x_parent = []
        y_parent = []
        theta_parent = []
        x_child = []
        y_child = []
        theta_child = []
        for i, u in enumerate(parent_region):
            for j, v in enumerate(u):
                x_parent.append(v[0])
                y_parent.append(v[1])
                theta_parent.append(v[2])
        for k, s in enumerate(child_region):
            for l, t in enumerate(s):
                x_child.append(t[0])
                y_child.append(t[1])
                theta_child.append(t[2])

        x_parent_min = min(x_parent)
        x_parent_max = max(x_parent)
        y_parent_min = min(y_parent)
        y_parent_max = max(y_parent)
        theta_parent_min = min(theta_parent)
        theta_parent_max = max(theta_parent)

        x_child_min = min(x_child)
        x_child_max = max(x_child)
        y_child_min = min(y_child)
        y_child_max = max(y_child)
        theta_child_min = min(theta_child)
        theta_child_max = max(theta_child)
        
        #x,y,theta方向それぞれの領域の減少量をスカラーで導出
        if (x_parent_min <= x_child_min):
            x_decrease = x_child_min - x_parent_min
        else:
            x_decrease = x_parent_max - x_child_max
        
        if (y_parent_min <= y_child_min):
            y_decrease = y_child_min - y_parent_min
        else:
            y_decrease = y_parent_max - y_child_max

        if (theta_parent_min <= theta_child_min):
            theta_decrease = theta_child_min - theta_parent_min
        else:
            theta_decrease = theta_parent_max - theta_child_max

        decrease = max([x_decrease, y_decrease, theta_decrease])
        
        #減少量が閾値を超えていたらTrue，下回っていたらFalseを返す
        if (decrease > eps):
            return True
        else:
            False

    #前姿勢と現姿勢のグリッド数の減少量による判定
    def cluster_number_judge(self, child_region: list, parent_region: list, eps):
        for i, u in enumerate(child_region):
            point_number_child = len(u)
        for k, v in enumerate(parent_region):
            point_number_parent = len(v)
        
        dif = point_number_parent - point_number_child
        print("前姿勢から現姿勢へのポイント数減少量は", dif)
        
        if (dif > eps):
            return True
        else:
            False
                                 
#ここから関数----------------------------------------------------------------------------------------------------------------------------------------    
def draw_polygons(polygons: list):
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.set_xlim(-20, 370)
    ax.set_ylim(-20, 320)
    for polygon in polygons:
        vertexes=[]
        for segment in polygon.segments:
            vertexes.append((segment.start.x, segment.start.y))
        p = pat.Polygon(xy=vertexes)
        #print("vertexes",vertexes)
        ax.add_patch(p)
    #plt.show()
    plt.pause(.5)
    plt.close()

def degree_to_radian(degrees: float):
    return degrees / 180.0 * math.pi

#この関数は，引数の時点で単位を[rad]にしておくこと!!関数内では角度は[rad]として扱っている．    
def create_equilateraltriangle(center_of_gravity: Point2D, direction, vertex_length):
    Vx = center_of_gravity.x - vertex_length / np.sqrt(3) * np.cos(direction + np.pi / 6)
    Vy = center_of_gravity.y - vertex_length / np.sqrt(3) * np.sin(direction + np.pi / 6)
    reference_point = Point2D(Vx, Vy)
    return Polygon(reference_point, direction, [vertex_length, vertex_length, vertex_length], [degree_to_radian(120), degree_to_radian(120), degree_to_radian(120)])

def create_rectangle(reference_point: Point2D, direction, vertex_length_horizontal, vertex_length_vertical):
    base = Point2D(reference_point.x - 1 / 2 * vertex_length_horizontal * np.cos(direction), reference_point.y - 1 / 2 * vertex_length_horizontal * np.sin(direction))
    return Polygon(base, direction, [vertex_length_horizontal, vertex_length_vertical, vertex_length_horizontal, vertex_length_vertical],[degree_to_radian(90), degree_to_radian(90), degree_to_radian(90), degree_to_radian(90)] )

#任意の点(base)と，長方形の中心点の間の距離を求める関数．引数は任意の点の座標に加え，長方形の基準点の座標と傾きと辺長である．戻り値は中心間距離．
def distance_of_centers(base: Point2D, v_base: Point2D, direction, edge_length_horizontal, edge_length_vertical):
    circumcenter = Point2D(v_base.x + (edge_length_horizontal * np.cos(direction) - edge_length_vertical * np.sin(direction)) / 2, v_base.y + (edge_length_horizontal * np.sin(direction) + edge_length_vertical * np.cos(direction)) / 2)
    distance = np.sqrt((base.x - circumcenter.x) * (base.x - circumcenter.x) + (base.y - circumcenter.y) * (base.y - circumcenter.y))
    return distance

if __name__ == '__main__':
    t_start=time.time()
    #スタート時の関節角をここで指定!!!RRTクラス内の対象物の初期領域の変更を忘れずに!!!
    start = Node(-25, -25, 25, 25)
    #ゴール時の状態(x,y,theta)をここで指定
    goal = State3D(0, 40, 0)
    #関節角のサンプリング範囲を指定
    random_sampling_area = [-90, 90]
    #4次元空間内での探索木の枝の長さをここで指定
    expandation_distance = 2
    trial_times = 100
    #ゴール時の閾値をここで指定
    epsilon = 10
    #------------------------------------------------------------------------------------------
    print('type filename')
    filename = input()
    f = open(filename + '.csv', 'w')
    RRT = RRT(start, goal, random_sampling_area, expandation_distance, trial_times, epsilon)
    path = RRT.planning()
    for node in path:
        f.write('%s,%s,%s,%s\n' % (node[0], node[1], node[2], node[3]))
    
    t_goal = time.time()
    totaltime = t_goal - t_start
    print("動作計画にかかった時間：",totaltime)
    
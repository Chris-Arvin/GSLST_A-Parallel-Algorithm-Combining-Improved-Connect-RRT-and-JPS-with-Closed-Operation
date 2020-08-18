import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import numpy as np
import tkinter as tk
import time
import cv2 as cv
import multiprocessing as mp
import JPS_without_goal as J
import jps_GSLST as G

# 注意，不同的线程的self的东西不是共享的

"""
Attention:
key version.
这是一个稳定（最终路径较短，保证了interfaces间的最优性）但是稍慢一点的版本。慢的原因是依然在最后调用了A_figure，用JPS去求解了最终路径中interfaces间的行走方式
和Improved_GSLST_4相同
"""

"""
Log
1. 更改了构建local dynamic link tree的方法，把生成所有inrerfaces的时间从1.2s缩减到了0.4s
2. 更改了JPS生成的点，现在只会把关键点放到生成路径中，而不是步长为1的所有点（JPS得到的结果可以直接拿来用了）
3. 对匹配border和使用JPS进行了合并，在匹配border时使用无目标点的JPS，直接得到局部树
4. 通过双min_node，阻止了一些点被加入到global sampling trees中
5. 增加了check_collision函数，封装性加强了一点；删除了end_limitation()本身也没意义；增加了run_one_step和run_one_more_step，更改了spring，增强了封装性
6. 在4.的基础上，对run_one_step进行进一步泛化

todo 剩self.results(),self.optimal()和self.link()没改了
todo 目前不能执行的原因是由生成的ldlt的单元都是node，results不能进行处理
todo 是否可以把局部树放到一个二维的list中
"""

"""
Tricks:
1. 双min_node，来减少节点的生成
2. ldlt相关：用增量式方法+(r,c,r0,c0)，一次遍历生成interface和narrow passage地图；用局部树+无目标点JPS/dijkstra的方法来求建立局部树，一次遍历进行匹配和路径求解
"""

"""
这一版本主要是针对匹配interfaces进行了结构性优化
待解决的问题有：局部树如何建立、列表式构建树、rewire、树的合并、多层闭运算
"""

# each node has varieties:row,col,father
class node:
    def __init__(self, r=0, c=0, f=None, flag=0):
        self.row = r
        self.col = c
        self.father = f
        self.distance = 0
        self.flag = flag
        father = self.father
        while True:
            if father == None:
                break
            self.distance += np.sqrt((r - father.row) ** 2 + (c - father.col) ** 2)
            r = father.row
            c = father.col
            father = father.father


class rrt:
    # initial the start, end, map
    def __init__(self):
        # initial map & window
        self.height = 800
        self.width = 800
        # initial extend limitation and ede limitation
        self.step_size = 80
        self.end_lim = 50

        self.start = node(50, 50, None)
        self.end = node(700, 700, None)
        self.col_map = np.zeros([self.height, self.width])
        self.del_radius=2

        # node list
        self.list1 = []
        self.list2 = []
        self.list1.append(self.start)
        self.list2.append(self.end)
        # initialize windows
        self.window = tk.Tk()
        self.window.title('rrt')
        self.window.geometry('%dx%d' % (self.width, self.height + 100))
        self.robot_R = 5

    # initla window, canvas, collision and begin to extend
    def init_map(self):

        # initialize canvas
        self.canvas = tk.Canvas(self.window, bg='white', height=self.height, width=self.width)
        self.canvas.place(x=0, y=0, anchor='nw')
        self.canvas.create_oval(self.start.col - 3, self.start.row - 3, self.start.col + 3, self.start.row + 3,
                                fill='red')
        self.canvas.create_oval(self.end.col - 3, self.end.row - 3, self.end.col + 3, self.end.row + 3, fill='red')
        self.add_collision()
        self.canvas.update()
        # q用来team交互
        self.q = mp.Queue()
        # p1用来交互原始路径
        self.p1 = mp.Queue()
        # p2用来交互skip后的路径
        self.p2 = mp.Queue()

        self.m = mp.Queue()

        self.mp_one = mp.Process(target=self.extend)
        self.mp_two = mp.Process(target=self.dilate)
        self.mp_one.start()
        self.mp_two.start()
        # self.mp_one.join()

        self.res = self.p1.get()
        self.path = self.p2.get()
        self.draw()

        # 共三个线程：主线程只是用来画图的，真正进行运算的是两个副线程。
        tk.mainloop()

    # add collisions
    def add_collision(self):

        # map1
        '''
        for r in range(300,500):
            for c in range(0,300):
                self.col_map[r][c]=255
            for c in range(310,799):
                self.col_map[r][c]=255
        '''

        # map2
        '''
        for r in range(200, 350):
            for c1 in range(0, 300):
                self.col_map[r][c1] = 255
            for c2 in range(309, 799):
                self.col_map[r][c2] = 255

        for r in range(450, 525):
            for c1 in range(0, r - 200):
                self.col_map[r][c1] = 255
            for c2 in range(r - 190, 500):
                self.col_map[r][c2] = 255

        for r in range(525, 600):
            for c1 in range(0, 325 + 525 - r):
                self.col_map[r][c1] = 255
            for c2 in range(335 + 525 - r, 500):
                self.col_map[r][c2] = 255

        for r in range(450, 525):
            for c1 in range(500, r - 200 + 300):
                self.col_map[r][c1] = 255
            for c2 in range(r - 190 + 300, 799):
                self.col_map[r][c2] = 255

        for r in range(525, 600):
            for c1 in range(500, 275 + 475 - r + 400):
                self.col_map[r][c1] = 255
            for c2 in range(285 + 475 - r + 400, 799):
                self.col_map[r][c2] = 255
        '''

        # map3
        '''
        for r in range(0,799):
            for c in range(300,600):
                self.col_map[r][c]=255

        for r in range(305,310):
            for c in range(300,355):
                self.col_map[r][c]=0
            for c in range(405,455):
                self.col_map[r][c]=0
            for c in range(505,555):
                self.col_map[r][c]=0
        for r in range(380,385):
            for c in range(350,400):
                self.col_map[r][c]=0
            for c in range(450,500):
                self.col_map[r][c]=0
            for c in range(550,600):
                self.col_map[r][c]=0

        for r in range(305,385):
            for c in range(350,355):
                self.col_map[r][c]=0
            for c in range(450,455):
                self.col_map[r][c]=0
            for c in range(550,555):
                self.col_map[r][c]=0

            for c in range(400,405):
                self.col_map[r][c]=0
            for c in range(500,505):
                self.col_map[r][c]=0
        '''

        # compare_map1
        '''
        for r in range(0,600):
            for c in range(100,200):
                self.col_map[r][c]=255
            for c in range(480,580):
                self.col_map[r][c]=255

        for r in range(300,800):
            for c in range(210,310):
                self.col_map[r][c]=255
            for c in range(590,690):
                self.col_map[r][c]=255
        '''

        # compare_map2
        '''
        #start=[350,750],end=[750,750]
        for r in range(100,300):
            for c in range(100,300):
                self.col_map[r][c]=255
        for r in range(0,350):
            for c in range(600,700):
                self.col_map[r][c]=255
        for r in range(300,560):
            for c in range(490,590):
                self.col_map[r][c]=255
        for r in range(560,660):
            for c in range(150,800):
                self.col_map[r][c]=255
        '''

        # compare_map3
        '''
        #start=[500,480],end=[550,650]
        for r in range(100,300):
            for c in range(100,300):
                self.col_map[r][c]=255
        for r in range(200,600):
            for c in range(500,520):
                self.col_map[r][c]=255
        for r in range(600,620):
            for c in range(250,520):
                self.col_map[r][c]=255
        for r in range(300,400):
            for c in range(600,800):
                self.col_map[r][c]=255
        '''

        # compare_map4
        '''
        #start=[50,400], end=[750,400]
        for r in range(0, 200):
            for c in range(250,300):
                self.col_map[r][c]=255
        for r in range(0, 20):
            for c in range(500,550):
                self.col_map[r][c]=255
        for r in range(30, 200):
            for c in range(500,550):
                self.col_map[r][c]=255
        for r in range(200,250):
            for c in range(250,550):
                self.col_map[r][c]=255


        for r in range(600, 800):
            for c in range(250,300):
                self.col_map[r][c]=255
        for r in range(600, 800):
            for c in range(500,550):
                self.col_map[r][c]=255
        for r in range(550,600):
            for c in range(250,300):
                self.col_map[r][c]=255
            for c in range(350,550):
                self.col_map[r][c]=255
        '''

        # compare_map5
        '''
        # start=[50,400], end=[750,400]
        for r in range(400):
            for c in range(400):
                if np.sqrt(r ** 2 + c ** 2) < 300:
                    self.col_map[r][c] = 255
        for r in range(400, 800):
            for c in range(400, 800):
                if np.sqrt((r - 800) ** 2 + (c - 800) ** 2) < 300:
                    self.col_map[r][c] = 255
        for r in range(400, 800):
            for c in range(400):
                if np.sqrt((r - 800) ** 2 + c ** 2) < 300:
                    self.col_map[r][c] = 255
        for r in range(400):
            for c in range(400, 800):
                if np.sqrt(r ** 2 + (c - 800) ** 2) < 300:
                    self.col_map[r][c] = 255
        for r in range(100, 700):
            for c in range(100, 700):
                if np.sqrt((r - 400) ** 2 + (c - 400) ** 2) < 250:
                    self.col_map[r][c] = 255
        '''

        # compare_map6
        '''
        # start=[50,50], end=[750,750]
        for r in range(300):
            for c in range(100, 200):
                self.col_map[r][c] = 255
        for r in range(310, 600):
            for c in range(100, 200):
                self.col_map[r][c] = 255
        for r in range(610, 800):
            for c in range(100, 200):
                self.col_map[r][c] = 255

        for r in range(500):
            for c in range(260, 360):
                self.col_map[r][c] = 255
        for r in range(510, 800):
            for c in range(260, 360):
                self.col_map[r][c] = 255

        for r in range(200):
            for c in range(420, 520):
                self.col_map[r][c] = 255
        for r in range(210, 800):
            for c in range(420, 520):
                self.col_map[r][c] = 255

        for r in range(100):
            for c in range(570, 670):
                self.col_map[r][c] = 255
        for r in range(110, 400):
            for c in range(570, 670):
                self.col_map[r][c] = 255
        for r in range(410, 800):
            for c in range(570, 670):
                self.col_map[r][c] = 255
        '''

        # compare_map7
        '''
        #start=[50,50], end=[750,750]
        for r in range(800):
            for c in range(100,200):
                self.col_map[r][c]=255
        for r in range(300,310):
            for c in range(100,150):
                self.col_map[r][c] = 0
        for r in range(280,290):
            for c in range(140,200):
                self.col_map[r][c] = 0
        for r in range(320,330):
            for c in range(140,200):
                self.col_map[r][c] = 0
        for r in range(280,330):
            for c in range(140,150):
                self.col_map[r][c] = 0

        for r in range(600,610):
            for c in range(100,150):
                self.col_map[r][c] = 0
        for r in range(580,590):
            for c in range(140,200):
                self.col_map[r][c] = 0
        for r in range(620,630):
            for c in range(140,200):
                self.col_map[r][c] = 0
        for r in range(580,630):
            for c in range(140,150):
                self.col_map[r][c] = 0


        for r in range(500):
            for c in range(260,360):
                self.col_map[r][c] = 255
        for r in range(510,800):
            for c in range(260,360):
                self.col_map[r][c] = 255

        for r in range(200):
            for c in range(420,520):
                self.col_map[r][c] = 255
        for r in range(210,800):
            for c in range(420,520):
                self.col_map[r][c] = 255

        for r in range(100):
            for c in range(570,670):
                self.col_map[r][c] = 255
        for r in range(110,400):
            for c in range(570,670):
                self.col_map[r][c] = 255
        for r in range(410,800):
            for c in range(570,670):
                self.col_map[r][c] = 255
        '''

        # compare_map8

        # start=[50,50], end=[750,750]
        for r in range(800):
            for c in range(100, 200):
                self.col_map[r][c] = 255
        for r in range(300, 310):
            for c in range(100, 150):
                self.col_map[r][c] = 0
        for r in range(280, 290):
            for c in range(140, 200):
                self.col_map[r][c] = 0
        for r in range(320, 330):
            for c in range(140, 200):
                self.col_map[r][c] = 0
        for r in range(280, 330):
            for c in range(140, 150):
                self.col_map[r][c] = 0

        for r in range(600, 610):
            for c in range(100, 150):
                self.col_map[r][c] = 0
        for r in range(580, 590):
            for c in range(140, 200):
                self.col_map[r][c] = 0
        for r in range(620, 630):
            for c in range(140, 200):
                self.col_map[r][c] = 0
        for r in range(580, 630):
            for c in range(140, 150):
                self.col_map[r][c] = 0

        for r in range(800):
            for c in range(260, 360):
                self.col_map[r][c] = 255
        for r in range(500, 510):
            for c in range(260, 360):
                self.col_map[r][c] = 0
        for r in range(550, 560):
            for c in range(260, 360):
                self.col_map[r][c] = 0
        for r in range(500, 560):
            for c in range(300, 310):
                self.col_map[r][c] = 0

        for r in range(800):
            for c in range(420, 520):
                self.col_map[r][c] = 255
        for i in range(400, 520):
            for j in range(8):
                self.col_map[i][i + j] = 0

        for r in range(100):
            for c in range(570, 670):
                self.col_map[r][c] = 255
        for r in range(110, 400):
            for c in range(570, 670):
                self.col_map[r][c] = 255
        for r in range(410, 800):
            for c in range(570, 670):
                self.col_map[r][c] = 255

        for i in range(800):
            for j in range(800):
                if self.col_map[i][j] == 255:
                    self.canvas.create_rectangle(j, i, j, i, fill='black')

    def run_one_step(self, new_r, new_c, list_temp):
        # "Near". find rule:only the distance
        # 两个min_node节点，s是最小的，用于考虑连接；b是第二小的，用于考虑delete
        min_node_s = 1000000
        min_node_b = 10000000
        father_node = node()
        for i in range(len(list_temp)):
            temp = list_temp[i]
            dis_r = temp.row - new_r
            dis_c = temp.col - new_c
            distance = dis_r ** 2 + dis_c ** 2

            if np.sqrt(distance) < min_node_s and distance > 0:
                father_node = temp
                min_node_b = min_node_s
                min_node_s = distance
        # 新生成的点几何距离2内至少有两个已经生成的点，太密集了，舍弃
        # 生成失败，附近已经有很多节点了，返回False和一个已经存在的节点
        if min_node_b <= self.del_radius:
            return False, father_node
        # "Steer" and "Edge". link nodes
        distance = np.sqrt(min_node_s)

        if distance <= self.step_size:
            new_node = node(new_r, new_c, father_node)

        else:
            add_row = (new_r - father_node.row) * self.step_size / distance + father_node.row
            add_col = (new_c - father_node.col) * self.step_size / distance + father_node.col
            new_node = node(add_row, add_col, father_node)

        # check collision
        is_collision = self.check_collision(father_node, new_node)
        # 撞击障碍物，返回False
        if is_collision:
            return False, 'collision'
        return True, new_node

    # 重构，指定father node，不需要再去搜索了
    # 仅用于持续扩展，不检测周围已存在的节点个数
    def run_one_more_step(self, new_r, new_c, father_node):
        # "Steer" and "Edge". link nodes
        distance = np.sqrt((new_r - father_node.row) ** 2 + (new_c - father_node.col) ** 2)

        if distance <= self.step_size:
            new_node = node(new_r, new_c, father_node)

        else:
            add_row = (new_r - father_node.row) * self.step_size / distance + father_node.row
            add_col = (new_c - father_node.col) * self.step_size / distance + father_node.col
            new_node = node(add_row, add_col, father_node)

        # check collision
        is_collision = self.check_collision(father_node, new_node)
        # 撞击障碍物，返回False
        if is_collision:
            return False, 'collision'
        return True, new_node

    # figure out the nearest node
    def spring(self, flag):
        new_r = int(self.height * np.random.rand())
        new_c = int(self.width * np.random.rand())
        # 第一棵global sampling tree的生长
        if flag == 1:
            list_temp = self.list1
        if flag == 2:
            list_temp = self.list2

        is_run, new_node = self.run_one_step(new_r, new_c, list_temp)
        temp_new_node = new_node
        # 生成失败
        if is_run == False:
            return False, None, None
        # append+翻转flag
        if flag == 1:
            self.list1.append(new_node)
            list_temp=self.list2
        if flag == 2:
            self.list2.append(new_node)
            list_temp=self.list1

        # 第二棵global sampling tree的生长
        is_run, new_node = self.run_one_step(temp_new_node.row, temp_new_node.col, list_temp)
        # collision生成失败
        if is_run == False and new_node == 'collision':
            return False, None, None
        # 生成成功或目标点附近已经有了至少两个点
        else:
            # 反向append
            if flag == 1:
                self.list2.append(new_node)
            if flag == 2:
                self.list1.append(new_node)
            while True:
                is_run, new_node = self.run_one_more_step(temp_new_node.row, temp_new_node.col, new_node)
                # 撞击障碍物
                if is_run == False:
                    return False, None, None
                # 反向append
                if flag == 1:
                    self.list2.append(new_node)
                if flag == 2:
                    self.list1.append(new_node)

                # 两棵树连接成功
                if new_node.row == temp_new_node.row and new_node.col == temp_new_node.col:
                    if flag == 1:
                        return True, temp_new_node, new_node
                    if flag == 2:
                        return True, new_node, temp_new_node

    # expend nodes, flag is to figure whether to limit the new springed node's position
    def extend(self):
        # 如果extend的时间较大，大概率是因为此路径无法再优化了（椭圆内障碍物太多），这时直接退出就可以了;
        # 如果前后两次路径的差值小于1，则已收敛了
        self.go = time.time()
        print('thread one start time:', self.go)
        k = 0
        while True:
            k += 1
            now = time.time()
            # 规划失败
            if now - self.go > 10:
                # draw new node and link
                for i in range(len(self.list1)):
                    self.canvas.create_rectangle(self.list1[i].col - 2, self.list1[i].row - 2, self.list1[i].col + 2,
                                                 self.list1[i].row + 2,
                                                 fill='green')
                    if self.list1[i].father != None:
                        self.canvas.create_line(self.list1[i].col, self.list1[i].row, self.list1[i].father.col,
                                                self.list1[i].father.row)
                for i in range(len(self.list2)):
                    self.canvas.create_rectangle(self.list2[i].col - 2, self.list2[i].row - 2, self.list2[i].col + 2,
                                                 self.list2[i].row + 2,
                                                 fill='green')
                    if self.list2[i].father != None:
                        self.canvas.create_line(self.list2[i].col, self.list2[i].row, self.list2[i].father.col,
                                                self.list2[i].father.row)
                self.canvas.update()

                print('no path')
                time.sleep(5)
                exit()
            """
            从这里开始，都是线程一的规划时间小于10s时，才发生的事
            """
            if self.q.qsize() != 0 and k % 50 == 0:
                self.link()
            # if len(self.list1)<=len(self.list2):
            if k % 2 == 0:
                param = self.spring(1)
            else:
                param = self.spring(2)
            # 有3个param，说明connect-RRT成功了
            if param[0] == True:
                print('总，连接成功')
                print('总，总采样次数:', k)
                print('总，总采样成功次数：', len(self.list1) + len(self.list2))  # list1和list2的本质意义实际上是为了帮助新生成的点找父亲节点
                self.path = self.results(t1=param[1], t2=param[2])
                print('总，路径长度:', len(self.path))
                print("总，执行时间为：", time.time() - self.go)
                # for nod in self.path:
                #     print("[",nod.row,nod.col,"]")
                # self.draw()
                break

    def draw(self):
        for temp in range(len(self.path)):
            self.canvas.create_rectangle(self.path[temp].col - 2, self.path[temp].row - 2, self.path[temp].col + 2,
                                         self.path[temp].row + 2,
                                         fill='red', outline='red')
            if temp == len(self.path) - 1:
                break
            self.canvas.create_line(self.path[temp].col, self.path[temp].row, self.path[temp + 1].col,
                                    self.path[temp + 1].row, fill='red', width=3)

        self.canvas.update()

        self.window2 = tk.Tk()
        self.window2.title('rrt')
        self.window2.geometry('%dx%d' % (self.width, self.height + 100))
        self.canvas2 = tk.Canvas(self.window2, bg='white', height=self.height, width=self.width)
        self.canvas2.place(x=0, y=0, anchor='nw')
        self.canvas2.create_oval(self.start.col - 3, self.start.row - 3, self.start.col + 3, self.start.row + 3,
                                 fill='red')
        self.canvas2.create_oval(self.end.col - 3, self.end.row - 3, self.end.col + 3, self.end.row + 3, fill='red')

        for i in range(800):
            for j in range(800):
                if self.col_map[i][j] == 255:
                    self.canvas2.create_rectangle(j, i, j, i, fill='black')

        for temp in range(len(self.res)):
            self.canvas2.create_rectangle(self.res[temp].col - 2, self.res[temp].row - 2, self.res[temp].col + 2,
                                          self.res[temp].row + 2,
                                          fill='green', outline='green')
            if temp == len(self.res) - 1:
                break
            self.canvas2.create_line(self.res[temp].col, self.res[temp].row, self.res[temp + 1].col,
                                     self.res[temp + 1].row, fill='black', width=2)

        for temp in range(len(self.path)):
            self.canvas2.create_rectangle(self.path[temp].col - 2, self.path[temp].row - 2, self.path[temp].col + 2,
                                          self.path[temp].row + 2,
                                          fill='red', outline='red')
            if temp == len(self.path) - 1:
                break
            self.canvas2.create_line(self.path[temp].col, self.path[temp].row, self.path[temp + 1].col,
                                     self.path[temp + 1].row, fill='red', width=3)

        self.canvas2.update()

    # optimal path
    def optim_path(self, path):
        if len(path) > 3:
            t = 0
            while True:
                flag = True
                temp1 = path[t]
                temp3 = path[t + 2]
                # check collision the second time: whether the path is in the collision!
                is_collision = self.check_collision(temp1, temp3)
                if is_collision:
                    flag = False
                if flag:
                    path.pop(t + 1)
                else:
                    t += 1
                if t + 2 == len(path):
                    break
        return path

    def A_figure(self, start, end):
        # 这个０是为了从元组中把ｐａｔｈ这个数组取出来
        temp = self.m.get()
        path = G.find_path([start.row, start.col], [end.row, end.col], temp)[0]
        self.m.put(temp)
        return path

    # when make it, go back to find the relavently low cost path
    def results(self, t1, t2):
        # create the path list from start node to temp_all[0]
        temp = t1
        res2 = []
        res2.append(temp)
        while temp != self.start:
            temp = temp.father
            res2.append(temp)
        # reverse the results
        res = []
        l = len(res2) - 1
        for i in range(len(res2)):
            count = l - i
            res.append(res2[count])

        # create the path list from temp_all[1] to end node
        temp = t2
        res.append(temp)
        while temp != self.end:
            temp = temp.father
            res.append(temp)
        num = 0
        while num != len(res) - 1:
            if res[num].flag != 0 and res[num + 1].flag != 0 and res[num].flag == res[num + 1].flag:
                print('总，到连接成功的时间为:', time.time() - self.go)
                path = list(self.A_figure(res[num], res[num + 1]))
                path.pop(0)
                path.pop(-1)
                for i in range(len(path)):
                    res.insert(num + 1 + i, node(path[i][0], path[i][1], res[num + i]))
                num += len(path) + 1
            else:
                num += 1
        # return self.optim_path(res)
        r = res.copy()
        self.res = self.optim_path(r)
        self.p1.put(res)
        self.p2.put(self.res)
        return res

    def dilate(self):
        # 初始地图：self.col_map
        # 闭运算后的地图：self.col_map_21
        self.start_time = time.time()
        print('tread two start time:', self.start_time)
        map = self.col_map
        kernel = np.ones((2 * self.robot_R + 1, 2 * self.robot_R + 1), np.uint8)

        map = cv.dilate(src=map, kernel=kernel)
        self.col_map_21 = cv.erode(src=map, kernel=kernel)
        print("2,到闭运算的时间为：", time.time() - self.start_time)

        # new method
        '''
        border=[
             [ [r,c,r0,c0],...   ]    #[更新的边界r，更新的边界c，原始的边界r，原始的边界c]
             [ [r,c,r0,c0],...   ]
             ]
        '''
        border = []  # 保存Local Dynamic Link Tree
        self.col_map_31 = np.zeros([self.height, self.width])
        for r in range(self.height):
            for c in range(self.width):
                if self.col_map[r][c] == 0 and self.col_map_21[r][c] != 0:
                    self.col_map_31[r][c] = 1  # 该点在narrow passage中
                    # 地图扩散：左到右、上到下、右上到左下
                    """
                    0 * *
                    * x 0
                    0 0 0
                    """
                    if self.col_map_31[r][c - 1] != 0 or self.col_map_31[r - 1][c] != 0 or self.col_map_31[r - 1][
                        c + 1] != 0:
                        flag_state = 1  # 标记[r,c]来自于扩散
                    else:
                        flag_state = 2  # 可能产生了新的narrow passage

                    # 判断是否为边界点
                    # or前：水平方向可通行且存在narrow passage
                    # or后：竖直方向可通行且存在narrow passage
                    if ((self.col_map[r][c - 1] == 0 and self.col_map[r][c + 1] == 0) and
                        (self.col_map_21[r][c - 1] == 0 and self.col_map_21[r][c + 1] != 0 or self.col_map_21[r][
                            c - 1] != 0 and self.col_map_21[r][c + 1] == 0)) \
                            or ((self.col_map[r - 1][c] == 0 and self.col_map[r + 1][c] == 0) and
                                (self.col_map_21[r - 1][c] == 0 and self.col_map_21[r + 1][c] != 0 or
                                 self.col_map_21[r - 1][c] != 0 and self.col_map_21[r + 1][c] == 0)):
                        # 不可斜角穿越，仅考虑从上到下、从左到右、从右上到左下的扩散情况
                        # 根据flag，判断某个边界点是被扩散产生的还是重新产生的
                        if flag_state == 2:
                            # 创建一个新的border
                            border.append([r, c, r, c])
                            continue
                        if flag_state == 1:
                            now_index = -1
                            # 遍历border的所有已经有的点，扩展border
                            for temp in border:
                                now_index = now_index + 1
                                if temp[0] + 1 == r and temp[1] == c:
                                    # 竖直方向扩展
                                    border[now_index][0] = r
                                    break
                                if temp[1] + 1 == c and temp[0] == r:
                                    # 水平方向扩展边界
                                    border[now_index][1] = c
                                    break
                            else:
                                # 该边界点是不与任何已存在的border相连接，独自构成一个新的边界
                                border.append([r, c, r, c])
        self.border = []  # 从[r,c,r0,c0]到[r_middle,c_middle]
        for i in border:
            r = int(i[0] / 2 + i[2] / 2)
            c = int(i[1] / 2 + i[3] / 2)
            self.col_map_31[r][c] = 2
            self.border.append([r, c])
        print("2,到所有interface生成的时间为：", time.time() - self.start_time)
        # for i in self.border:
        #     self.canvas.create_rectangle(i[1]-2,i[0]-2,i[1]+2,i[0]+2,fill='red')
        # self.canvas.update()

        self.m.put(self.col_map_31)

        # border不为空，说明还有interface没有进行匹配
        while self.border:
            # 进行匹配并建立local dynamic link tree
            jps = J.JPS(width=800, height=800, map=self.col_map_31)
            s_pos = self.border.pop()
            ldlt = jps.create_ldlt(s_pos=s_pos)
            # for i in ldlt:
            #     print(i.pos)
            #     self.canvas.create_rectangle(i.pos[1]-2,i.pos[0]-2,i.pos[1]+2,i.pos[0]+2,fill='red')
            # self.canvas.update()
            # 去除已经匹配成功的interface
            for j in range(1, len(ldlt)):
                self.border.remove(ldlt[j].pos)
            # 去除dead path
            if len(ldlt) == 1:
                continue
            l = []
            for i in ldlt:
                l.append(i.pos)
            self.q.put(l)
            # print(ldlt)
            # self.q.put(ldlt)
            print("2,到生成一个local dynamic link tree的时间为：", time.time() - self.start_time)

    # True表示撞到了障碍物
    def check_collision(self, p1, p2):
        col = np.linspace(p1.col, p2.col, int(self.step_size), endpoint=True)
        row = np.linspace(p1.row, p2.row, int(self.step_size), endpoint=True)
        for j in range(min(len(col), len(row))):
            if self.col_map[int(row[j])][int(col[j])] > 100:
                return True
        return False

    def link(self):
        m = 0
        fail = []
        while self.q.qsize() != 0:
            team = self.q.get()
            is_success = 0
            for num in range(len(team)):
                # list1
                new_r = team[num][0]
                new_c = team[num][1]
                flag1 = 1
                flag2 = 1
                min_node = 1000000
                temp_node = node()
                for i in range(len(self.list1)):
                    temp = self.list1[i]
                    dis_r = temp.row - new_r
                    dis_c = temp.col - new_c
                    distance = dis_r ** 2 + dis_c ** 2

                    if np.sqrt(distance) < min_node and distance > 0:
                        temp_node = temp
                        min_node = distance

                # "Steer" and "Edge". link nodes
                distance = np.sqrt(min_node)

                new_node = node(new_r, new_c, temp_node, flag=m + 1)

                # check collision
                is_collision = self.check_collision(temp_node, new_node)
                if is_collision:
                    flag1 = 0
                if flag1 == 1:
                    self.list1.append(new_node)
                    for i in range(len(team)):
                        if i == num:
                            continue
                        self.list1.append(node(team[i][0], team[i][1], new_node, flag=m + 1))

                # list2
                if flag1 == 0:
                    min_node = 1000000
                    temp_node = node()
                    for i in range(len(self.list2)):
                        temp = self.list2[i]
                        dis_r = temp.row - new_r
                        dis_c = temp.col - new_c
                        distance = dis_r ** 2 + dis_c ** 2

                        if np.sqrt(distance) < min_node and distance > 0:
                            temp_node = temp
                            min_node = distance

                    # "Steer" and "Edge". link nodes
                    distance = np.sqrt(min_node)

                    new_node = node(new_r, new_c, temp_node, flag=m + 1)

                    # check collision
                    is_collision = self.check_collision(temp_node, new_node)
                    if is_collision:
                        flag2 = 0
                    if flag2 == 1:
                        self.list2.append(new_node)
                        for i in range(len(team)):
                            if i == num:
                                continue
                            self.list2.append(node(team[i][0], team[i][1], new_node, flag=m + 1))

                if flag1 == 1 or flag2 == 1:
                    is_success = 1
                    break
            if is_success == 0:
                fail.append(team)
            m += 1
        for i in fail:
            self.q.put(i)


if __name__ == '__main__':
    rrt_agent = rrt()
    rrt_agent.init_map()


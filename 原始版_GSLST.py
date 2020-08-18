import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import numpy as np
import tkinter as tk
import time
import cv2 as cv
import multiprocessing as mp
import jps_GSLST as J

#注意，不同的线程的self的东西不是共享的

"""
Log
1. 通过border中的点被检索后，remove该点，来减去了team的去重操作
2. 更改了JPS生成的点，现在只会把关键点放到生成路径中，而不是步长为1的所有点（JPS得到的结果可以直接拿来用了）
"""

# each node has varieties:row,col,father
class node:
    def __init__(self, r=0, c=0, f=None,flag=0):
        self.row = r
        self.col = c
        self.father = f
        self.distance=0
        self.flag=flag
        father=self.father
        while True:
            if father == None:
                break
            self.distance+=np.sqrt((r-father.row)**2+(c-father.col)**2)
            r=father.row
            c=father.col
            father=father.father

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
        self.col_map=np.zeros([self.height,self.width])

        # node list
        self.list1 = []
        self.list2 = []
        self.list1.append(self.start)
        self.list2.append(self.end)
        # initialize windows
        self.window = tk.Tk()
        self.window.title('rrt')
        self.window.geometry('%dx%d' % (self.width, self.height+100))
        self.robot_R = 5

    def A_figure(self,start,end):
        #这个０是为了从元组中把ｐａｔｈ这个数组取出来
        temp=self.m.get()
        path = J.find_path([start.row,start.col], [end.row,end.col], temp)[0]
        self.m.put(temp)
        if len(path)>3:
            t=0
            while True:
                flag=True
                temp1=path[t]
                temp3=path[t+2]
                # check collision the second time: whether the path is in the collision!
                col = np.linspace(temp1[1], temp3[1], int(np.sqrt((temp1[0]-temp3[0])**2+(temp1[1]-temp1[1])**2)), endpoint=True)
                row = np.linspace(temp1[0], temp3[0], int(np.sqrt((temp1[0]-temp3[0])**2+(temp1[1]-temp1[1])**2)), endpoint=True)
                for j in range(min(len(col), len(row))):
                    #在检查的时候发现，这里ｃｏｌ和ｒｏｗ弄反了！！
                    if self.col_map[int(row[j])][int(col[j])] > 100:
                        flag = False
                if flag:
                    path.pop(t+1)
                else:
                    t+=1
                if temp3 == path[-1]:
                    break
        return path




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
        #q用来team交互
        self.q=mp.Queue()
        #m用来map交互(self.col_map_31_copy)
        self.m=mp.Queue()
        #p1用来交互原始路径
        self.p1=mp.Queue()
        #p2用来交互skip后的路径
        self.p2=mp.Queue()

        self.mp_one=mp.Process(target=self.extend)
        self.mp_two=mp.Process(target=self.dilate)
        self.mp_one.start()
        self.mp_two.start()
        # self.mp_one.join()

        self.res=self.p1.get()
        self.path=self.p2.get()
        self.draw()

        #共三个线程：主线程只是用来画图的，真正进行运算的是两个副线程。






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

        for r in range(800):
            for c in range(260,360):
                self.col_map[r][c] = 255
        for r in range(500,510):
            for c in range(260,360):
                self.col_map[r][c]=0
        for r in range(550,560):
            for c in range(260,360):
                self.col_map[r][c]=0
        for r in range(500,560):
            for c in range(300,310):
                self.col_map[r][c]=0


        for r in range(800):
            for c in range(420,520):
                self.col_map[r][c] = 255
        for i in range(420,520):
            for j in range(8):
                self.col_map[i][i+j] = 0

        for r in range(100):
            for c in range(570,670):
                self.col_map[r][c] = 255
        for r in range(110,400):
            for c in range(570,670):
                self.col_map[r][c] = 255
        for r in range(410,800):
            for c in range(570,670):
                self.col_map[r][c] = 255


        for i in range(800):
            for j in range(800):
                if self.col_map[i][j] == 255:
                    self.canvas.create_rectangle(j, i, j, i, fill='black')


    # figure out the nearest node
    def spring(self, flag, mk_dir_flag=1):
        new_r = int(self.height * np.random.rand())
        new_c = int(self.width * np.random.rand())

        if flag == 2:
            self.list1,self.list2=self.list2,self.list1
        # "Near". find rule:only the distance
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

        if distance <= self.step_size:
            new_node = node(new_r, new_c, temp_node)

        else:
            add_row = (new_r - temp_node.row) * self.step_size / distance + temp_node.row
            add_col = (new_c - temp_node.col) * self.step_size / distance + temp_node.col
            new_node = node(add_row, add_col, temp_node)




        # check collision
        col = np.linspace(temp_node.col, new_node.col, int(self.step_size ), endpoint=True)
        row = np.linspace(temp_node.row, new_node.row, int(self.step_size ), endpoint=True)
        for j in range(min(len(col), len(row))):
            if self.col_map[int(row[j])][int(col[j])]>100:
                if flag == 2:
                    self.list1, self.list2 = self.list2, self.list1
                return False

        self.list1.append(new_node)

        # the tree birthed from the end node;
        # 在第一颗树和新节点作用完成后，去考虑另一个树，从原来的树开始一直往new node连接，一直到撞到障碍物或者连接到new node（搜索结束）
        min_node = 1000000
        temp_node = node()
        for i in range(len(self.list2)):
            temp = self.list2[i]
            dis_r = temp.row - new_node.row
            dis_c = temp.col - new_node.col
            distance = dis_r ** 2 + dis_c ** 2

            if distance < min_node and distance > 0:
                temp_node = temp
                min_node = distance

        # "Steer" and "Edge". link nodes
        distance = np.sqrt(min_node)
        if distance <= self.step_size:
            new_node2 = node(new_node.row, new_node.col, temp_node)
        else:
            add_row = (new_node.row - temp_node.row) * self.step_size / distance + temp_node.row
            add_col = (new_node.col - temp_node.col) * self.step_size / distance + temp_node.col
            new_node2 = node(add_row, add_col, temp_node)

        # check collision: whether the path is in the collision!
        col = np.linspace(temp_node.col, new_node2.col, int(self.step_size ), endpoint=True)
        row = np.linspace(temp_node.row, new_node2.row, int(self.step_size ), endpoint=True)
        for j in range(min(len(col), len(row))):
            if self.col_map[int(row[j])][int(col[j])]>100:
                if flag == 2:
                    self.list1, self.list2 = self.list2, self.list1
                return False

        # self.canvas.create_rectangle(new_node2.col - 2, new_node2.row - 2, new_node2.col + 2, new_node2.row + 2,
        #                              fill='green')
        # self.canvas.create_line(new_node2.col, new_node2.row, temp_node.col, temp_node.row)
        # self.canvas.update()

        # add the new node into node list
        self.list2.append(new_node2)

        # 如果走一步就到了新node，就直接退出了
        if new_node2 == new_node:
            if flag == 2:
                self.list1, self.list2 = self.list2, self.list1
            return True
        else:
            while True:
                distance = np.sqrt((new_node2.col - new_node.col) ** 2 + (new_node2.row - new_node.row) ** 2)
                if distance <= self.step_size:
                    new_node3 = node(new_node.row, new_node.col, new_node2)
                else:
                    add_row = (new_node.row - new_node2.row) * self.step_size / distance + new_node2.row
                    add_col = (new_node.col - new_node2.col) * self.step_size / distance + new_node2.col
                    new_node3 = node(add_row, add_col, new_node2)

                # check collision the second time: whether the path is in the collision!
                col = np.linspace(new_node2.col, new_node3.col, int(self.step_size ), endpoint=True)
                row = np.linspace(new_node2.row, new_node3.row, int(self.step_size ), endpoint=True)
                for j in range(min(len(col), len(row))):
                    if self.col_map[int(row[j])][int(col[j])] > 100:
                        if flag == 2:
                            self.list1, self.list2 = self.list2, self.list1
                        return False

                # self.canvas.create_rectangle(new_node3.col - 2, new_node3.row - 2, new_node3.col + 2,
                #                              new_node3.row + 2,
                #                              fill='green')
                # self.canvas.create_line(new_node2.col, new_node2.row, new_node3.col, new_node3.row)
                # self.canvas.update()

                # add the new node into node list
                self.list2.append(new_node3)
                # 结束标志，同上
                if new_node3.row == new_node.row and new_node3.col == new_node.col:
                    if flag == 2:
                        self.list1, self.list2 = self.list2, self.list1
                    return True
                # 更换new_node2，进行迭代
                new_node2 = new_node3



    # end requirement,返回的是能连接两个tree，且使得总长度最小的两个点
    def end_limitation(self):
        #t1,t2是两个可连接的节点
        t1 = None
        t2 = None
        path_all_length = np.inf
        #list1和list2是两个tree
        for temp1 in self.list1:
            for temp2 in self.list2:
                dis = np.inf
                if (temp1.row - temp2.row) ** 2 + (temp1.col - temp2.col) ** 2 <= self.step_size ** 2:
                    # calculate the length of all path
                    temp_node = temp1
                    dis = 0
                    while True:
                        if temp_node == self.start:
                            break
                        dis += np.sqrt(
                            (temp_node.row - temp_node.father.row) ** 2 + (temp_node.col - temp_node.father.col) ** 2)
                        temp_node = temp_node.father
                    temp_node = temp2
                    while True:
                        if temp_node == self.end:
                            break
                        dis += np.sqrt(
                            (temp_node.row - temp_node.father.row) ** 2 + (temp_node.col - temp_node.father.col) ** 2)
                        temp_node = temp_node.father
                    dis += np.sqrt((temp1.row - temp2.row) ** 2 + (temp1.col - temp2.col) ** 2)
                if dis < path_all_length:
                    t1 = temp1
                    t2 = temp2
        if t1 == None:
            return False
        return t1, t2

    # expend nodes, flag is to figure whether to limit the new springed node's position
    def extend(self, flag=0):
        #如果extend的时间较大，大概率是因为此路径无法再优化了（椭圆内障碍物太多），这时直接退出就可以了;
        #如果前后两次路径的差值小于1，则已收敛了
        self.go=time.time()
        print('thread one start time:',self.go)
        self.is_success=True
        k=0
        while True:
            k+=1
            now=time.time()
            if now-self.go>10:
                # draw new node and link
                for i in range(len(self.list1)):
                    self.canvas.create_rectangle(self.list1[i].col -2, self.list1[i].row - 2, self.list1[i].col + 2, self.list1[i].row + 2,
                                            fill='green')
                    if self.list1[i].father!=None:
                        self.canvas.create_line(self.list1[i].col, self.list1[i].row, self.list1[i].father.col, self.list1[i].father.row)
                for i in range(len(self.list2)):
                    self.canvas.create_rectangle(self.list2[i].col -2, self.list2[i].row - 2, self.list2[i].col + 2, self.list2[i].row + 2,
                                            fill='green')
                    if self.list2[i].father!=None:
                        self.canvas.create_line(self.list2[i].col, self.list2[i].row, self.list2[i].father.col, self.list2[i].father.row)
                self.canvas.update()

                print('no path')
                time.sleep(5)
                exit()
            if self.q.qsize()!=0 and k%50==0:
                self.link()
            # if len(self.list1)<=len(self.list2):
            if k%2==0:
                is_success=self.spring(1, flag)
            else:
                is_success=self.spring(2, flag)
            if is_success:
                print('总，连接成功')
                print('总，总采样次数:',k)
                temp = self.end_limitation()
                if temp != False:
                    self.path = self.results(temp)
                    print('总，路径长度:',len(self.path))
                    print("总，执行时间为：",time.time()-self.go)
                    # for nod in self.path:
                    #     print("[",nod.row,nod.col,"]")
                    # self.draw()
                    break

    def draw(self):
        # for temp in self.list1:
        #     self.canvas.create_rectangle(temp.col - 2, temp.row - 2, temp.col + 2,
        #                                  temp.row + 2,
        #                                  fill='green',outline='green')
        #     if temp.father:
        #         self.canvas.create_line(temp.col, temp.row, temp.father.col, temp.father.row,fill='black',width=2)
        #
        # for temp in self.list2:
        #     self.canvas.create_rectangle(temp.col - 2, temp.row - 2, temp.col + 2,
        #                                  temp.row + 2,
        #                                  fill='green',outline='green')
        #     if temp.father:
        #         self.canvas.create_line(temp.col, temp.row, temp.father.col, temp.father.row,fill='black',width=2)


        for temp in range(len(self.path)):
            self.canvas.create_rectangle(self.path[temp].col - 2, self.path[temp].row - 2, self.path[temp].col + 2,
                                         self.path[temp].row + 2,
                                         fill='red',outline='red')
            if temp==len(self.path)-1:
                break
            self.canvas.create_line(self.path[temp].col, self.path[temp].row, self.path[temp+1].col, self.path[temp+1].row,fill='red',width=3)


        self.canvas.update()





        self.window2 = tk.Tk()
        self.window2.title('rrt')
        self.window2.geometry('%dx%d' % (self.width, self.height+100))
        self.canvas2 = tk.Canvas(self.window2, bg='white', height=self.height, width=self.width)
        self.canvas2.place(x=0, y=0, anchor='nw')
        self.canvas2.create_oval(self.start.col - 3, self.start.row - 3, self.start.col + 3, self.start.row + 3,
                                fill='red')
        self.canvas2.create_oval(self.end.col - 3, self.end.row - 3, self.end.col + 3, self.end.row + 3, fill='red')


        for i in range(800):
            for j in range(800):
                if self.col_map[i][j] == 255:
                    self.canvas2.create_rectangle(j, i, j, i, fill='black')

        # for temp in self.list1:
        #     self.canvas2.create_rectangle(temp.col - 2, temp.row - 2, temp.col + 2,
        #                                  temp.row + 2,
        #                                  fill='green',outline='green')
        #     if temp.father!=None:
        #         self.canvas2.create_line(temp.col, temp.row, temp.father.col, temp.father.row,width=2)
        #
        # for temp in self.list2:
        #     self.canvas2.create_rectangle(temp.col - 2, temp.row - 2, temp.col + 2,
        #                                  temp.row + 2,
        #                                  fill='green',outline='green')
        #     if temp.father!=None:
        #         self.canvas2.create_line(temp.col, temp.row, temp.father.col, temp.father.row,width=2)


        for temp in range(len(self.res)):
            self.canvas2.create_rectangle(self.res[temp].col - 2, self.res[temp].row - 2, self.res[temp].col + 2,
                                         self.res[temp].row + 2,
                                         fill='green',outline='green')
            if temp==len(self.res)-1:
                break
            self.canvas2.create_line(self.res[temp].col, self.res[temp].row, self.res[temp+1].col, self.res[temp+1].row,fill='black',width=2)

        for temp in range(len(self.path)):
            self.canvas2.create_rectangle(self.path[temp].col - 2, self.path[temp].row - 2, self.path[temp].col + 2,
                                         self.path[temp].row + 2,
                                         fill='red',outline='red')
            if temp==len(self.path)-1:
                break
            self.canvas2.create_line(self.path[temp].col, self.path[temp].row, self.path[temp+1].col, self.path[temp+1].row,fill='red',width=3)



        self.canvas2.update()


    #optimal path
    def optim_path(self,path):
        if len(path)>3:
            t=0
            while True:
                flag=True
                temp1=path[t]
                temp3=path[t+2]
                # check collision the second time: whether the path is in the collision!
                col = np.linspace(temp1.col, temp3.col, int(np.sqrt((temp1.col-temp3.col)**2+(temp1.row-temp1.row)**2)), endpoint=True)
                row = np.linspace(temp1.row, temp3.row, int(np.sqrt((temp1.col-temp3.col)**2+(temp1.row-temp1.row)**2)), endpoint=True)
                for j in range(min(len(col), len(row))):
                    #在检查的时候发现，这里ｃｏｌ和ｒｏｗ弄反了！！
                    if self.col_map[int(row[j])][int(col[j])] > 100:
                        flag = False
                if flag:
                    path.pop(t+1)
                else:
                    t+=1
                if t+2 == len(path):
                    break
        return path


    # when make it, go back to find the relavently low cost path
    def results(self, temp_all):
        # create the path list from start node to temp_all[0]
        temp = temp_all[0]
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
        temp = temp_all[1]
        res.append(temp)
        while temp != self.end:
            temp = temp.father
            res.append(temp)
        num=0
        while num!=len(res)-1:
            if res[num].flag!=0 and res[num+1].flag!=0 and res[num].flag==res[num+1].flag:
                print('总，到连接成功的时间为:',time.time()-self.go)
                path=list(self.A_figure(res[num],res[num+1]))
                path.pop(0)
                path.pop(-1)
                for i in range(len(path)):
                    res.insert(num+1+i,node(path[i][0],path[i][1],res[num+i]))
                num+=len(path)+1
            else:
                num+=1
        # return self.optim_path(res)
        r=res.copy()
        self.res=self.optim_path(r)
        self.p1.put(res)
        self.p2.put(self.res)
        return res

    def dilate(self):
        self.start_time = time.time()
        print('tread two start time:',self.start_time)
        map = self.col_map
        kernel = np.ones((2 * self.robot_R + 1, 2 * self.robot_R + 1), np.uint8)

        map = cv.dilate(src=map, kernel=kernel)
        self.col_map_21 = cv.erode(src=map, kernel=kernel)
        print("2,到闭运算的时间为：", time.time() - self.start_time)

        self.col_map_31 = np.zeros([self.height, self.width])
        for r in range(self.height):
            for c in range(self.width):
                if self.col_map_21[r][c]!=0 and self.col_map[r][c]==0:
                    self.col_map_31[r][c] = 1
        self.col_map_31_copy=self.col_map_31.copy()
        self.m.put(self.col_map_31_copy)
        # cv.imshow('a',self.col_map_31_copy)
        # cv.waitKey(0)
        print("2,到绿色通路为止的时间为：", time.time() - self.start_time)
        # self.canvas.update()

        self.col_map_4 = np.zeros([self.height, self.width])
        for r in range(1, self.height - 1):
            for c in range(1, self.width - 1):
                if self.col_map[r][c] == 0 and self.col_map_31[r][c] > 0:
                    # attention 下面这一块导致了转角处为０
                    if self.col_map[r - 1][c] == 0 and self.col_map_31[r - 1][c] == 0 or self.col_map[r + 1][c] == 0 and self.col_map_31[r + 1][c] == 0 or self.col_map[r][c - 1] == 0 and self.col_map_31[r][c - 1] == 0 or self.col_map[r][c + 1] == 0 and self.col_map_31[r][c + 1] == 0:
                        self.col_map_4[r][c] = 1
        print("2,到边界生成的时间为：", time.time() - self.start_time)

        # 加copy很重要
        self.col_map_4_cop = self.col_map_4.copy()

        #这里通过取中间点的方式，将线合并成了点
        border = []
        r = 0
        while r < self.height:
            c = 0
            while c < self.height:
                if self.col_map_4[r][c] > 0:
                    k = 1
                    r_n = r
                    while self.col_map_4[r + k][c] == 1:
                        self.col_map_4[r + k][c] = 0
                        r_n = r + k
                        k += 1

                    k = 1
                    c_n = c
                    while self.col_map_4[r][c + k] == 1:
                        self.col_map_4[r][c + k] = 0
                        c_n = c + k
                        k += 1
                    # 取中点
                    border.append([int(r_n / 2 + r / 2), int(c_n / 2 + c / 2)])
                    self.col_map_31[int(r_n / 2 + r / 2)][int(c_n / 2 + c / 2)] = 2

                    c = c_n
                c += 1
            r += 1

        print(border)
        print("2,到求解border时间为：", time.time() - self.start_time)
        # 这里之前的时间浪费主要是在ｃｌｏｓｅｌｉｓｔ，改成遍历地图之后，快了超级多
        # 这里主要是给border做配对，以某个border点为起点，进行扩散（被扩展到的点，在col_map_31中的值被改成9）
        for i in border:
            if self.col_map_31[i[0]][i[1]] != 2:
                continue
            team = []
            team.append([i[0], i[1]])
            open_list = []
            self.col_map_31[i[0]][i[1]]=9
            # 扩展周围节点
            for j in range(-1, 2):
                for k in range(-1, 2):
                    if j == 0 and k == 0:
                        continue
                    if self.col_map_31[i[0] + j][i[1] + k] != 0:
                        open_list.append([i[0] + j, i[1] + k])
            while len(open_list) != 0:
                node = open_list.pop(0)
                self.col_map_31[node[0]][node[1]]=9
                for j in range(-1, 2):
                    for k in range(-1, 2):
                        if j == 0 and k == 0:
                            continue
                        if self.col_map_31[node[0] + j][node[1] + k]==9 or [node[0] + j, node[1] + k] in open_list:
                            continue
                        # 被扩展到的border点被添加到team中
                        if self.col_map_31[node[0] + j][node[1] + k] == 2:
                            self.col_map_31[node[0] + j][node[1] + k] = 0
                            team.append([node[0] + j, node[1] + k])
                            # 在border中剔除该点
                            # border.remove([node[0]+j,node[1]+k])
                            continue
                        if self.col_map_31[node[0] + j][node[1] + k] == 1:
                            open_list.append([node[0] + j, node[1] + k])
            #dead path
            if len(team) == 1:
                continue

            print(team)
            self.q.put(team)
            print("2,到生成一个局部树的时间为：", time.time() - self.start_time)

    def link(self):
        m=0
        fail=[]
        while self.q.qsize()!=0:
            team=self.q.get()
            is_success=0
            for num in range(len(team)):
                # list1
                new_r=team[num][0]
                new_c=team[num][1]
                flag1=1
                flag2=1
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

                new_node = node(new_r, new_c, temp_node,flag=m+1)

                # check collision
                col = np.linspace(temp_node.col, new_node.col, int(self.step_size), endpoint=True)
                row = np.linspace(temp_node.row, new_node.row, int(self.step_size), endpoint=True)
                for j in range(min(len(col), len(row))):
                    if self.col_map[int(row[j])][int(col[j])] > 100:
                        flag1=0
                if flag1==1:
                    self.list1.append(new_node)
                    for i in range(len(team)):
                        if i==num:
                            continue
                        self.list1.append(node(team[i][0],team[i][1],new_node,flag=m+1))

                #list2
                if flag1==0:
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

                    new_node = node(new_r, new_c, temp_node,flag=m+1)

                    # check collision
                    col = np.linspace(temp_node.col, new_node.col, int(self.step_size), endpoint=True)
                    row = np.linspace(temp_node.row, new_node.row, int(self.step_size), endpoint=True)
                    for j in range(min(len(col), len(row))):
                        if self.col_map[int(row[j])][int(col[j])] > 100:
                            flag2 = 0
                    if flag2 == 1:
                        self.list2.append(new_node)
                        for i in range(len(team)):
                            if i==num:
                                continue
                            self.list2.append(node(team[i][0], team[i][1], new_node,flag=m+1))

                if flag1==1 or flag2==1:
                    is_success=1
                    break
            if is_success==0:
                fail.append(team)
            m+=1
        for i in fail:
            self.q.put(i)









if __name__ == '__main__':
    rrt_agent = rrt()
    rrt_agent.init_map()


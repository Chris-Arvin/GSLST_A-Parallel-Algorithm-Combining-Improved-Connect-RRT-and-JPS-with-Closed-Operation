#!/usr/bin/env python
# -*- coding: utf-8 -*-

#对比jps_v1和v0，去掉了所有的print和画图。把地图改成了狭窄路径地图

"""
Log
1. change make_path(), delete the middle nodes.

"""

"""
interesting operations:
and or
enumerate
del
"""

import numpy as np
import tkinter as tk
import time

# 地图数据；0：能通过；1:障碍，不能通过
# map[x][y]


# 可行走方向
g_dir = [[1, 0], [0, 1], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]]


class Node:
    def __init__(self, parent, pos, g, h):
        self.parent = parent
        self.pos = pos  # (x,y)
        self.g = g
        self.h = h
        self.f = g + h

    def get_direction(self):
        #parent is [x,y], separately operate x and y for each "and" or "or"
        # parent and sth. or [0,0]
        # and中含0，返回0； 均为非0时，返回后一个值，
        # or中， 至少有一个非0时，返回第一个非0,

        # move on the "x" from parent to current point.
        # move on the "y" from parent to current point.
        # return self.parent and [self.pos[0] != self.parent.pos[0] and np.sign(self.pos[0] - self.parent.pos[0]),
        #                         self.pos[1] != self.parent.pos[1] and np.sign(self.pos[1] - self.parent.pos[1])]
        return self.parent and [self.pos[0] != self.parent.pos[0] and (self.pos[0] - self.parent.pos[0])/abs(self.pos[0] - self.parent.pos[0]),
                                self.pos[1] != self.parent.pos[1] and (self.pos[1] - self.parent.pos[1])/abs(self.pos[1] - self.parent.pos[1])]

# test_map = []
class JPS:

    # 注意w,h两个参数，如果你修改了地图，需要传入一个正确值或者修改这里的默认参数
    def __init__(self, width, height, map):
        self.s_pos = None
        self.e_pos = Node
        self.map_test=map
        self.width = width
        self.height = height
        self.open = []
        self.close = []
        self.path = []

    def prune_neighbours(self, c):
        nbs = []
        # 不是起始点
        if c.parent:
            # 进入的方向
            dir = c.get_direction()
            if self.is_pass(c.pos[0] + dir[0], c.pos[1] + dir[1]):
                nbs.append([c.pos[0] + dir[0], c.pos[1] + dir[1]])
            # 对角线行走; eg:右下(1, 1)
            # virtual run
            if dir[0] != 0 and dir[1] != 0:
                # 下（0， 1）
                if self.is_pass(c.pos[0], c.pos[1] + dir[1]):
                    nbs.append([c.pos[0], c.pos[1] + dir[1]])
                # 右（1， 0）
                if self.is_pass(c.pos[0] + dir[0], c.pos[1]):
                    nbs.append([c.pos[0] + dir[0], c.pos[1]])
                # 左不能走且下可走
                if not self.is_pass(c.pos[0] - dir[0], c.pos[1]) and self.is_pass(c.pos[0], c.pos[1] + dir[1]):
                    # 左下（-1， 1）
                    nbs.append([c.pos[0] - dir[0], c.pos[1] + dir[1]])
                # 上不能走且右可走
                if not self.is_pass(c.pos[0], c.pos[1] - dir[1]) and self.is_pass(c.pos[0] + dir[0], c.pos[1]):
                    # 右上（1， -1）
                    nbs.append([c.pos[0] + dir[0], c.pos[1] - dir[1]])
            else:  # 直行
                # 垂直走
                if dir[0] == 0:
                    # 右不能走
                    if not self.is_pass(c.pos[0] + 1, c.pos[1]):
                        # 右下
                        nbs.append([c.pos[0] + 1, c.pos[1] + dir[1]])
                    # 左不能走
                    if not self.is_pass(c.pos[0] - 1, c.pos[1]):
                        # 左下
                        nbs.append([c.pos[0] - 1, c.pos[1] + dir[1]])

                else:  # 水平走，向右走为例
                    # 下不能走
                    if not self.is_pass(c.pos[0], c.pos[1] + 1):
                        # 右下
                        nbs.append([c.pos[0] + dir[0], c.pos[1] + 1])
                    # 上不能走
                    if not self.is_pass(c.pos[0], c.pos[1] - 1):
                        # 右上
                        nbs.append([c.pos[0] + dir[0], c.pos[1] - 1])
        # if the node is start node, extend on 8 directions.
        else:
            for d in g_dir:
                if self.is_pass(c.pos[0] + d[0], c.pos[1] + d[1]):
                    nbs.append([c.pos[0] + d[0], c.pos[1] + d[1]])
        return nbs

    # ↑ ↓ ← → ↖ ↙ ↗ ↘
    # 沿着父子方向斜着持续走，每走一步，检测在3类情况下，该点是否为关键点，如果是，则返回该点
    # 这个函数只判断now点是否为关键点，或进行斜线行走，不负责改变行走方向
    def jump_node(self, now, pre):
        # dir = [a != b and np.sign(a - b) or 0 for a, b in zip(now, pre)]
        # direction from pre to now
        dir = [a != b and (a - b)/abs(a-b) or 0 for a, b in zip(now, pre)]      # a=now_x,b=pre_x; a=now_y,b=pre_y
        dir[0] = int(dir[0])
        dir[1] = int(dir[1])

        if now == self.e_pos:
            return now

        if self.is_pass(now[0], now[1]) is False:
            return None
        '''
        pre 1   0       
        1   now 0       
        0   0   0       

        '''
        # 父子斜着走,周围存在特殊障碍物分布
        if dir[0] != 0 and dir[1] != 0:
            # 左下能走且左不能走，或右上能走且上不能走
            if (self.is_pass(now[0] - dir[0], now[1] + dir[1]) and not self.is_pass(now[0] - dir[0], now[1])) or (
                    self.is_pass(now[0] + dir[0], now[1] - dir[1]) and not self.is_pass(now[0], now[1] - dir[1])):
                return now
        # 父子横着或竖着走,周围存在特殊障碍物分布
        else:
            # 水平方向
            if dir[0] != 0:
                # 右下能走且下不能走， 或右上能走且上不能走
                '''
                * 1 0       0 0 0
                0 → 0       0 0 0
                * 1 0       0 0 0

                '''

                if (self.is_pass(now[0] + dir[0], now[1] + 1) and not self.is_pass(now[0], now[1] + 1)) or (
                        self.is_pass(now[0] + dir[0], now[1] - 1) and not self.is_pass(now[0], now[1] - 1)):
                    return now
            else:  # 垂直方向
                # 右下能走且右不能走，或坐下能走且左不能走
                '''
                0 0 0
                1 ↓ 1
                0 0 0

                '''
                if (self.is_pass(now[0] + 1, now[1] + dir[1]) and not self.is_pass(now[0] + 1, now[1])) or (
                        self.is_pass(now[0] - 1, now[1] + dir[1]) and not self.is_pass(now[0] - 1, now[1])):
                    return now


        # 该点沿着水平或直线扩张后，存在特殊点
        if dir[0] != 0 and dir[1] != 0:
            t1 = self.jump_node([now[0] + dir[0], now[1]], now)
            t2 = self.jump_node([now[0], now[1] + dir[1]], now)
            if t1 or t2:
                return now
        # 该点周围不存在特殊点，now被抛弃，继续沿着原父子的方向斜着走
        if self.is_pass(now[0] + dir[0], now[1]) or self.is_pass(now[0], now[1] + dir[1]):
            t = self.jump_node([now[0] + dir[0], now[1] + dir[1]], now)
            if t:
                return t

        return None

    #extend the point c's neightbor points.
    # 首先扩展该关节点，找到关键邻居节点（及行走方向）；对于每个邻居节点，沿着走，如果能遇到新的关键节点，把新节点添加或更新到openlist中
    def extend_round(self, c):
        # 选择关键点的邻居节点
        nbs = self.prune_neighbours(c)
        for n in nbs:
            # 跳点得到下一个关键点
            jp = self.jump_node(n, [c.pos[0], c.pos[1]])
            if jp:
                if self.node_in_close(jp):
                    continue
                g = self.get_g(jp, c.pos)   # local g
                h = self.get_h(jp, self.e_pos)  # global h
                node = Node(c, jp, c.g + g, h)  # father node, position, g, h
                i = self.node_in_open(node)     # whether in open_list
                if i != -1:
                    # 新节点在open_list
                    if self.open[i].g > node.g:
                        # 现在的路径到比以前到这个节点的路径更好~
                        # 则使用现在的路径
                        self.open[i].parent = c
                        self.open[i].g = node.g
                        self.open[i].f = node.g + self.open[i].h
                    continue
                #新节点不在open_list
                self.open.append(node)

    # the point is within the map which is free or the target node.
    def is_pass(self, x, y):
        x = int(x)
        y = int(y)
        return x >= 0 and x < self.width and y >= 0 and y < self.height and (self.map_test[x][y] != 0 or x == self.e_pos[0] and y == self.e_pos[1])

    # 查找路径的入口函数
    def find_path(self, s_pos, e_pos):
        self.s_pos, self.e_pos = s_pos, e_pos
        # 构建开始节点: 父亲节点，(ｘ,ｙ)，ｇ＝０，ｈ＝城区距离
        p = Node(None, self.s_pos, 0, abs(self.s_pos[0] - self.e_pos[0]) + abs(self.s_pos[1] - self.e_pos[1]))
        self.open.append(p)
        while True:
            # 扩展F值最小的节点
            # 如果开放列表为空，则不存在路径，返回
            if not self.open:
                return "not find"
            # 获取F值最小的节点: id, node
            idx, p = self.get_min_f_node()
            # 找到路径，生成路径，返回
            if self.is_target(p):
                self.make_path(p)     # traverse the whole path
                return
            self.extend_round(p)    # 首先扩展该关节点，找到关键邻居（及行走方向）；对于每个邻居节点，沿着走，如果能遇到新的关键节点，把新节点添加或更新到openlist中
            # 把此节点压入关闭列表，并从开放列表里删除
            self.close.append(p)
            del self.open[idx]      # just same to remove

    # def make_path(self, p):
    #     # 从结束点回溯到开始点，开始点的parent == None
    #     while p:
    #         if p.parent:
    #             dir = p.get_direction()
    #             n = p.pos
    #             while n != p.parent.pos:
    #                 self.path.append(n)
    #                 n = [n[0] - dir[0], n[1] - dir[1]]
    #         else:
    #             #just for start node
    #             self.path.append(p.pos)
    #         p = p.parent
    #     self.path.reverse()

    def make_path(self, p):
        # 从结束点回溯到开始点，开始点的parent == None
        while p:
            self.path.append(p.pos)
            p = p.parent
        self.path.reverse()

    def is_target(self, n):
        return n.pos == self.e_pos

    def get_min_f_node(self):
        best = None
        bv = -1
        bi = -1
        for idx, node in enumerate(self.open):
            # value = self.get_dist(i)  # 获取F值
            if bv == -1 or node.f < bv:  # 比以前的更好，即F值更小
                best = node
                bv = node.f
                bi = idx
        return bi, best

    # 计算g值；直走=1；斜走=1.4
    def get_g(self, pos1, pos2):
        if pos1[0] == pos2[0]:
            return abs(pos1[1] - pos2[1])
        elif pos1[1] == pos2[1]:
            return abs(pos1[0] - pos2[0])
        else:
            return abs(pos1[0] - pos2[0]) * 1.4

    # 计算h值
    def get_h(self, pos1, pos2):
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def node_in_close(self, node):
        for i in self.close:
            if node == i.pos:
                return True
        return False

    def node_in_open(self, node):
        for i, n in enumerate(self.open):
            if node == n.pos:
                return i
        return -1


def find_path(s_pos, e_pos, map):
    jps = JPS(800, 800, map)
    err = jps.find_path(s_pos, e_pos)
    path = jps.path

    return path, err


def check_pos_valid(x, y):
    jps = JPS(None, x, y, x, y)
    return jps.is_valid_coord(x, y)


def draw():
    global window
    window = tk.Tk()
    window.title('rrt')
    window.geometry('%dx%d' % (800, 800))

    # initialize canvas
    global canvas
    canvas = tk.Canvas(window, bg='white', height=800, width=800)
    canvas.place(x=0, y=0, anchor='nw')
    canvas.create_oval(50 - 3, 50 - 3, 50 + 3, 50 + 3, fill='red', outline='red')
    canvas.create_oval(700 - 3, 700 - 3, 700 + 3, 700 + 3, fill='red', outline='red')
    for x in range(800):
        for y in range(800):
            if map_test[x][y] != 0:
                canvas.create_rectangle(x, y, x, y)
        canvas.update()
    for pos in path:
        canvas.create_rectangle(pos[0]-2, pos[1]-2, pos[0]+2, pos[1]+2, fill='green', outline='green')
    canvas.update()

    tk.mainloop()

# 注意，1是free区域（黑色），这里是为了和GSLST做适配
map_test = np.ones([800, 800])
for y in range(300, 500):
    for x in range(0, 300):
        map_test[x][y] = 0
    for x in range(310, 799):
        map_test[x][y] = 0

if __name__ == "__main__":
    time_start=time.time()
    path,err = find_path([50, 50], [700, 700],map_test)
    if err!=None:
        print(path)
    print('running time:',time.time()-time_start)
    draw()

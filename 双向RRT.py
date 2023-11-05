import matplotlib.pyplot as plt
import time
import math
import random
import copy

show_animation = True
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Bi_RRT:
    # 初始化
    def __init__(self, start, goal, obstacle_list, rand_area):
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.obsracle_list = obstacle_list
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        # 步长
        self.expandDis = 1.0
        # self.goalSampleRate = 0.05  # 选择终点的概率是0.05
        self.maxIter = 500
        self.nodeList = [self.start]
        self.endlist = [self.end]

    # 随机节点
    def random_node(self):
        """
        产生随机节点
        :return:
        """
        # 在-2,10的范围中产生
        node_x = random.uniform(self.min_rand, self.max_rand)
        node_y = random.uniform(self.min_rand, self.max_rand)
        node = [node_x, node_y]

        return node

    # 最近下标
    @staticmethod
    def get_nearest_list_index(node_list, rnd):
        """
        :param node_list:
        :param rnd:
        :return:
        """
        # 计算各个节点到rnd的距离
        d_list = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in node_list]
        # 找出最近的距离的下标
        min_index = d_list.index(min(d_list))
        return min_index

    @staticmethod
    def collision_check(new_node, obstacle_list):
        a = 1
        # 便利整个障碍物
        for (ox, oy, size) in obstacle_list:
            dx = ox - new_node.x
            dy = oy - new_node.y
            # 判断距障碍物距离
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                a = 0  # collision

        return a  # safe

    def newNode(self, rnd, nearest_node):
        # 计算角度
        # 返回弧度制  随机点和最近点的角度
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node = copy.deepcopy(nearest_node)
        # ex为拓展距离
        new_node.x += self.expandDis * math.cos(theta)
        new_node.y += self.expandDis * math.sin(theta)
        return new_node

    def planning(self):
        path1 = []
        path2 = []
        rnd1 = [self.end.x, self.end.y]
        rnd2 = [self.start.x, self.start.y]
        while True:
            min_index1 = self.get_nearest_list_index(self.nodeList, rnd1)
            min_index2 = self.get_nearest_list_index(self.endlist, rnd2)

            # expand tree 起点树中最近节点
            nearest_node1 = self.nodeList[min_index1]
            # expand tree 终点树中最近节点
            nearest_node2 = self.endlist[min_index2]

            new_node1 = self.newNode(rnd1, nearest_node1)
            new_node1.parent = min_index1

            new_node2 = self.newNode(rnd2, nearest_node2)
            new_node2.parent = min_index2

            if not self.collision_check(new_node1, self.obsracle_list):
                rnd1 = self.random_node()
                continue

            if not self.collision_check(new_node2, self.obsracle_list):
                rnd2 = self.random_node()
                continue
            self.nodeList.append(new_node1)
            self.endlist.append(new_node2)
            dx = new_node1.x - new_node2.x
            dy = new_node1.y - new_node2.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                path1 = [[new_node2.x, new_node2.y]]
                path2 = [[new_node1.x, new_node1.y]]
                break

            if True:
                self.draw_graph(rnd1, rnd2)

            rnd1 = [new_node2.x, new_node2.y]
            rnd2 = [new_node1.x, new_node1.y]

        last_index1 = len(self.nodeList) - 1
        last_index2 = len(self.endlist) - 1
        # 将节点倒序加入起点路径
        while self.nodeList[last_index1].parent is not None:
            node = self.nodeList[last_index1]
            path1.append([node.x, node.y])
            last_index1 = node.parent

        # 将节点倒序加入终点树序列
        while self.endlist[last_index2].parent is not None:
            node = self.endlist[last_index2]
            path2.append([node.x, node.y])
            last_index2 = node.parent
        path1.append([self.start.x, self.start.y])
        path2.append([self.end.x, self.end.y])
        path1 = path1[::-1]
        path = path1+ path2
       # path = list(set(path))
        cleaned_path = []
        for point in path:
            if point not in cleaned_path:
                cleaned_path.append(point)
        print(cleaned_path)
        return cleaned_path

    def draw_graph(self, rnd1=None, rnd2=None):
        """
        Draw Graph
        """

        plt.clf()  # 清除上次画的图
        if rnd1 is not None:
            # print(rnd.x,rnd.y)
            plt.plot(rnd1[0], rnd1[1], "^g")
        if rnd2 is not None:
            # print(rnd.x,rnd.y)
            plt.plot(rnd2[0], rnd2[1], "^b")

        for node in self.nodeList:
            if node.parent is not None:
                # 连接点和父节点
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")
        for node in self.endlist:
            if node.parent is not None:
                # 连接点和父节点
                plt.plot([node.x, self.endlist[node.parent].x], [
                    node.y, self.endlist[node.parent].y], "-g")
        # 绘制障碍物
        for (ox, oy, size) in self.obsracle_list:
            plt.plot(ox, oy, "sk", ms=10 * size)
        # 起点和终点
        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        # 坐标系
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        # 显示网格线
        plt.grid(True)
        plt.pause(0.03)

    def draw_static(self, path):
        """
        画出静态图像
        :return:
        """
        plt.clf()  # 清除上次画的图

        for node in self.nodeList:
            if node.parent is not None:
                # 连接点和父节点
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")
        for node in self.endlist:
            if node.parent is not None:
                # 连接点和父节点
                plt.plot([node.x, self.endlist[node.parent].x], [
                    node.y, self.endlist[node.parent].y], "-g")

        for (ox, oy, size) in self.obsracle_list:
            plt.plot(ox, oy, "sk", ms=10 * size)

        plt.plot(self.start.x, self.start.y, "^r")
        plt.plot(self.end.x, self.end.y, "^b")
        # xmin,xmax,ymin,ymax
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])

        plt.plot([data[0] for data in path], [data[1] for data in path], '-r')
        plt.grid(True)
        plt.show(block=True)


def main():
    print("start RRT path planning")
    # 障碍物定义
    obstacle_list = [
        (5, 1, 1),
        (3, 6, 2),
        (3, 8, 2),
        (1, 1, 2),
        (3, 5, 2),
        (9, 5, 2)]
    t = time.time() * 1000
    # Set Initial parameters
    rrt = Bi_RRT(start=[0, 0], goal=[8, 9], rand_area=[-2, 10], obstacle_list=obstacle_list)
    path = rrt.planning()
    print(path)

    # Draw final path
    if show_animation:
        plt.close()
        rrt.draw_static(path)
    print(str(time.time() * 1000 - t) + 'ms')


if __name__ == '__main__':
    main()

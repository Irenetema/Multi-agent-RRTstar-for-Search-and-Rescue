"""
RRT* path planning implementation with python
"""
from map import *
import random
import math


class RRTStarPlanner(PathPlanner):
    """
    Path Planner using RRT* algorithm.
    """

    def __init__(self, env_map, start_pos, end_pos, iterations=1e4, epsilon=0.2, stepSize=10):
        PathPlanner.__init__(self)
        self.nodeList = []
        self.map = env_map
        self.iterations = iterations
        self.epsilon = epsilon
        self.stepSize = stepSize

        self.start = start_pos
        self.target = end_pos
        self.curr_position = self.start

    def plan(self, start=None, target=None):
        if start is None:
            start = self.start
        if target is None:
            target = self.target

        self.nodeList = []
        self.nodeList.append(Node(start))
        for iteration in range(int(self.iterations)):
            # random sample
            randNode = Node(self.randomSample(self.epsilon, target))
            # find the nearest node
            nearestNode = self.findNearestNode(randNode)
            # expand the tree
            theta = math.atan2(randNode.pos.y - nearestNode.pos.y, randNode.pos.x - nearestNode.pos.x)
            newNode = Node(nearestNode.pos + Polar2Vector(self.stepSize, theta), nearestNode,
                           nearestNode.cost + self.stepSize)

            # outside the map
            if self.map.out_of_map(newNode.pos):
                continue
            # in the node list
            if self.inNodeList(newNode):
                continue
            # meet obstacles
            if self.check_obstacle(self.map, newNode.pos):
                continue

            # choose best parent
            radius = 100.0 * math.sqrt(math.log(len(self.nodeList)) / len(self.nodeList))
            self.chooseBestParent(newNode, radius)
            # rewire
            self.rewire(newNode, radius)

            self.nodeList.append(newNode)
            if newNode.pos.dist(target) < self.stepSize:
                print("final")
                len_final_path = len(self.finalPath)
                if len_final_path == 0:
                    self.finalPath = []
                    self.finalPath.append(target)
                    currentNode = self.nodeList[-1]
                    self.finalPath.append(currentNode.pos)
                    while currentNode.parent is not None:
                        self.finalPath.append(currentNode.parent.pos)
                        currentNode = currentNode.parent
                    self.finalPath.reverse()

                    break
                else:
                    new_path = [target]
                    currentNode = self.nodeList[-1]
                    new_path.append(currentNode.pos)
                    while currentNode.parent is not None:
                        new_path.append(currentNode.parent.pos)
                        currentNode = currentNode.parent
                    new_path.reverse()

                    if len(new_path) < len_final_path:
                        self.finalPath = new_path

                    break

    def update_start(self, new_start):
        self.start = new_start

    def update_target(self, new_target):
        self.target = new_target

    def rewire(self, newNode, radius):
        # find near nodes
        nearNodes = [nearNode for nearNode in self.nodeList if newNode.dist(nearNode) < radius]
        # rewire
        for nearNode in nearNodes:
            new_cost = newNode.cost + newNode.dist(nearNode)
            if new_cost < nearNode.cost:
                nearNode.cost = new_cost
                nearNode.parent = newNode

    def chooseBestParent(self, newNode, radius):
        # find near nodes
        nearNodes = [nearNode for nearNode in self.nodeList if newNode.dist(nearNode) < radius]
        # choose best parent
        for nearNode in nearNodes:
            new_cost = nearNode.cost + newNode.dist(nearNode)
            if self.check_line_collision(newNode.pos, nearNode.pos):
                continue
            if new_cost < newNode.cost:
                newNode.cost = new_cost
                newNode.parent = nearNode

    def findNearestNode(self, node):
        minDist = 1e10
        nearestNode = None
        for candidate in self.nodeList:
            if node.pos.dist(candidate.pos) < minDist:
                minDist = node.pos.dist(candidate.pos)
                nearestNode = candidate
        return nearestNode

    def randomSample(self, epsilon, target):
        if random.random() >= epsilon:
            return Point(self.map.left + self.map.length * random.random(),
                         self.map.down + self.map.width * random.random())
        else:
            return target

    def inNodeList(self, node):
        for candidate in self.nodeList:
            if candidate.pos == node.pos:
                return True
        return False

    def check_obstacle(self, env_map, pos):
        for obs in env_map.obstacles:
            if obs.check_collision(pos, 10):
                return True
        return False

    def check_line_collision(self, pos1, pos2):
        testNum = pos1.dist(pos2) / 10
        for i in range(int(testNum) + 1):
            testPos = pos1 + Polar2Vector(i * 10, pos1.dir(pos2))
            if self.check_obstacle(self.map, testPos):
                return True
        return False

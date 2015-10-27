class TwoDimensionalAvoidanceSolver(object):
    def __init__(self):
        self.world = []
        self.accuracy = 3.1
        self.accuracy_2 = 3.11
    
    def set_accuracy(self, accuracy=3.1):
        self.accuracy = accuracy

    def is_close(self, p1, p2):
        diffy = abs(p1[1] - p2[1])
        diffx = abs(p1[0] - p2[0])
        return (diffx < self.accuracy) and (diffy < self.accuracy)

    def check_obstacle(self, point, trajector):
        for obj in self.world:
            if obj != trajector.name:
                worldObj = getattr(self.world, obj)
                pos = [worldObj.pos.x, worldObj.pos.y]
                a = self.is_close(pos, point)
                if a:
                    #print('object avant',obj,'object position',pos,'point position',point)
                    new_point = self.calcul_new_point(pos, point, trajector)
                    return new_point
                #elif diffx < 4:# and obj != trajector.name:
                #    return ([point[0] + 3, point[1]])
                #elif diffy < 4:
                #    return ([point[0], point[1] + 3])
                    #return check_obstacle([point[0] + 3, point[1]], trajector)
        return point

    def calcul_new_point(self, pos,point, trajector):
        diffy = pos[1] - point[1]
        diffx = pos[0] - point[0]

        if abs(diffx) < self.accuracy and abs(diffy) < self.accuracy :
            
            if diffy < 0 and diffx < 0 :
                point[1] = pos[1] + self.accuracy_2 
                point[0] = pos[0] + self.accuracy_2 
            elif diffy >= 0 and diffx < 0 :
                point[1] = pos[1] - self.accuracy_2
                point[0] = pos[0] + self.accuracy_2 
            elif diffy < 0 and diffx >= 0:
                point[1] = pos[1] + self.accuracy_2
                point[0] = pos[0] - self.accuracy_2 
            elif diffy >= 0 and diffx >= 0 :
                point[1] = pos[1] - self.accuracy_2
                point[0] = pos[0] - self.accuracy_2 
        else :
            if diffx < 0 and abs(diffx) < self.accuracy:
                point[0] = pos[0] + self.accuracy_2
            elif diffx >= 0 and abs(diffx) < self.accuracy :
                point[0] = pos[0] - self.accuracy_2
            elif diffy < 0 and abs(diffy) < self.accuracy :
                point[1] = pos[1] + self.accuracy_2 
            elif diffy >= 0 and abs(diffy) < self.accuracy :
                point[1] = pos[1] - self.accuracy_2
        #print(point)
        return point
        #return self.check_obstacle(point, trajector)

    def get_slope(self, origin, destination):
        y = destination[1] - origin[1]
        x = destination[0] - origin[0]
        if x ==0:
            return None
        return float(y/x)

    def compute_line(self, origin, destination, trajector):
        slope = self.get_slope(origin, destination)
        if slope or slope == 0:
            points = []
            initialX = origin[0]
            c = origin[1] - (origin[0] * slope)
            if initialX < destination[0]:
                while initialX < destination[0]:
                    y = slope*initialX + c
                    p = [initialX, y]
                    points.append(self.check_obstacle(p, trajector))
                    #if check_obstacle(p):
                    #   points.append(check_obstacle(p), trajector)
                    #else:

                    #   points.append([initialX, y])
                    initialX += .1
            else:
                while initialX > destination[0]:
                    y = slope*initialX + c
                    p = [initialX, y]
                    points.append(self.check_obstacle(p, trajector))
                    #if check_obstacle(p):
                    #   points.append(check_obstacle(p)trajector)
                    #else:
                    #   points.append([initialX, y])
                    initialX -= .1
            points.append(destination)
            return points
        else: 
            points = []
            initialX = origin[0]
            initialY = origin[1]
            if initialY < destination[1]:
                while initialY < destination[1]:
                    p = [initialX, initialY]
                    points.append(self.check_obstacle(p, trajector))
                    initialY += .1
            else:
                while initialY >= destination[1]:
                    p = [initialX, initialY]
                    points.append(self.check_obstacle(p, trajector))
                    initialY -= .1
            return points

    def on_slope(self, p1, p2, slope):
        slope2 = self.get_slope(p1, p2)
        if slope2 is None or slope is None:
            return False
        diff = abs(slope - self.get_slope(p1, p2))
        #return slope == self.get_slope(p1, p2))
        return diff < .1

    def smooth_trajectory(self, points):
        index = 0
        new_points = []
        new_points.append(points[0])
        while index < (len(points) - 2):
            new_slope = self.get_slope(points[index], points[index+1])
            p = points[index+2]
            if not self.on_slope(points[index+1], p, new_slope):
                if not points[index] in new_points:
                    new_points.append(points[index])
                if not points[index+1] in new_points:
                    new_points.append(points[index+1])
                if not p in new_points:
                    new_points.append(p)
            index += 1
        new_points.append(points.pop())
        return new_points


import numpy as np
from matplotlib import pyplot as plt_source
import time
import csv
import math


class Visualisation:
    def __init__(self):
        self.plt=plt_source
        self.font_size_node_text=20
        self.size_node_point=20
        self.point_size=2
        self.color_node='black'

   
    def coordinates_from_x_y_distance_and_angle(self,angle, distance,orientation_angle=0,x_base=0, y_base=0, ):
        """
        :param orientation_angle: which direction in the world is oriented the robot
        :param x_base: where is lidar on x
        :param y_base: where is lidar on y
        :param angle: in degree ,0 is +y
        :param distance: from lidar to the point
        :return: new coordiantes of the found point
        """

        new_x = distance * math.sin(math.radians(angle+orientation_angle))
        new_y = distance * math.cos(math.radians(angle+orientation_angle))




        final_x = x_base + new_x
        final_y = y_base + new_y
        final_x = round(final_x, 2)
        final_y = round(final_y, 2)
        if final_x == 0 or final_x == -0:
            final_x = 0
        if final_y == 0 or final_y == -0:
            final_y = 0
        return final_x, final_y
   
   
   
   
    def extract_point_csv(self,path='current_scan.csv'):
        list_=[]
        with open(path, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                ang,dist=row[0].split("|")
                list_.append([float(ang),float(dist)])
        
        return list_

   
    def convert_points(self,points:list):
        new_x_y=[]
        for point in points:
            x,y=self.coordinates_from_x_y_distance_and_angle(point[0],point[1])
            new_x_y.append([x,y])
            
        return new_x_y

    def show_all_points(self,list_coord:list):
        """
        in plot
        show all nodes location
        show path between the node
        show all detected points for coresponding to each node
        """
        data = np.array(list_coord)
        x, y = data.T
        plt_source.scatter(x, y, color=f'green', s=self.point_size)
        plt_source.show()
        


if __name__=="__main__":
    v=Visualisation()
    a=v.extract_point_csv()
    b=v.convert_points(a)
    v.show_all_points(b)

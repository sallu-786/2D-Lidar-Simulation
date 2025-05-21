import math
import pygame
import numpy as np

def add_uncertainty(distance,angle,sigma):                          #sigma=s.deviation
    mean = np.array([distance,angle])
    covariance = np.diag(sigma**2)                                  #diagonal matrix with s.d values
    distance, angle = np.random.multivariate_normal(mean,covariance)#create a random noisy sample for given mean and s.d 
    distance=max(distance,0)                                        #purge negative values
    angle=max(angle,0)
    return [distance, angle]


class LidarSenor:
    def __init__(self,range,map,uncertainty):
        self.range=range
        self.speed=4                                                #num of scans per second
        self.sigma= np.array([uncertainty[0],uncertainty[1]])
        self.position = (0,0)
        self.map= map
        self.w,self.h=pygame.display.get_surface().get_size()
        self.sensed_obstacle = []
    
    def cal_dist(self,osbtacleposition):                            #Euclidean distance
        px=(osbtacleposition[0]-self.position[0])**2
        py=(osbtacleposition[1]-self.position[1])**2
        return math.sqrt(px+py)
    
    def sense_obstacle(self):
        data=[]
        x1,y1 = self.position[0], self.position[1]                  #get current position
        for angle in np.linspace(0,2*math.pi,60,False):             #evenly spaced samples from o to 360, endpoint false
            x2,y2 =(x1 + self.range * math.cos(angle), y1- self.range * math.sin(angle))    #endpoint (polar to cartesian) 
            for i in range(0,100):                                  #divide the ray in 100 segment
                u=i/100
                x =int(x2*u +x1*(1-u))                              #linear interpolation
                y =int(y2*u +y1*(1-u))
                if 0<x<self.w and 0<y<self.h:                       #check if point is inside screen
                    color = self.map.get_at((x,y))                  #check if color black/obstacle

                    if (color[0],color[1],color[2])==(0,0,0):
                        distance=self.cal_dist((x,y))               #distance from self.position
                        output =add_uncertainty(distance,angle,self.sigma) #approximate for noise
                        output.append(self.position)
                        #store the measurement
                        data.append(output)
                        break
        if len(data)>0:
            return data
        else:
            return False
 
class buildEnvironment:
    def __init__(self,MapDimensions):
        pygame.init()
        self.pointCloud=[]
        self.externalMap=pygame.image.load('map.png')
        self.maph,self.mapw =MapDimensions
        self.MapWindowName = "2D-Lidar Mapping"
        pygame.display.set_caption(self.MapWindowName)
        self.map=pygame.display.set_mode((self.mapw,self.maph))
        self.map.blit(self.externalMap,(0,0))
        #colors
        self.black=(0,0,0)
        self.grey=(70,70,70)
        self.blue=(0,0,255)
        self.green=(0,255,0)
        self.red=(255,0,0)
        self.white=(255,255,255)

    def AD2pos(self,distance,angle,robotPosition):                 #position of obstacle relative to robot            
        x = distance * math.cos(angle) + robotPosition[0]
        y= -distance * math.sin(angle) + robotPosition[1]
        return (int(x),int(y))

    def dataStorage(self,data):
        for element in data:
            point=self.AD2pos(element[0],element[1],element[2])    #dist, angle, robot_position
            if point not in self.pointCloud:
                self.pointCloud.append(point)

    def showSensorData(self):
        self.infomap=self.map.copy()
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]),int(point[1])),(255,0,0))


environment = buildEnvironment((600,1200))
running=True
environment.originalMap = environment.map.copy()
lidar = LidarSenor(200,environment.originalMap, uncertainty=(0.5,0.01))
environment.map.fill((0,0,0))
environment.infomap = environment.map.copy()

while running:
    sensorON =False
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
        if pygame.mouse.get_focused():
            sensorON=True
        elif not pygame.mouse.get_focused():
            sensorON=False    
    if sensorON:
        position=pygame.mouse.get_pos()
        lidar.position=position   
        sensor_data=lidar.sense_obstacle()

        environment.dataStorage(sensor_data)
        environment.showSensorData()
 
    environment.map.blit(environment.infomap,(0,0))
    pygame.display.update()
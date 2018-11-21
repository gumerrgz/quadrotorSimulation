#TH Koeln
#Robotic Lab
#Gumer Rodriguez
#Quadrotor control simulation - Graphics
#The pygame code structure was taken from the "Rotating 3D Cube using Python and Pygame" project
#and then modified to fit our goals. You will find the link of the original project in the next line:
#http://codentronix.com/2011/05/12/rotating-3d-cube-using-python-and-pygame/

#====================================================================================================
# Main program to display the simulation in a graphic environment
#====================================================================================================

import sys, math, pygame
from pygame.locals import *
from operator import itemgetter
from RTsimclass import *
 
#====================================================================================================
# Point3D class
#====================================================================================================

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)
 
    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)
 
    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in radians. """
        rad = angle
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)
 
    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

#====================================================================================================
# Simulation class
#====================================================================================================
 
class Simulation:
    def __init__(self, win_width = 1080, win_height = 640):
        pygame.init()
 
        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("Simulation of a quadrotor stabilitation control")
 
        self.clock = pygame.time.Clock()
 
        self.vertices = [
            Point3D(-1,-0.1,1),
            Point3D(1,-0.1,1),
            Point3D(1,-0.1,-1),
            Point3D(-1,-0.1,-1),
            Point3D(-1,0.1,1),
            Point3D(1,0.1,1),
            Point3D(1,0.1,-1),
            Point3D(-1,0.1,-1)
        ]

        self.ground = [
            Point3D(-10,-2,10),
            Point3D(10,-2,10),
            Point3D(10,-2,-1),
            Point3D(-10,-2,-1),
        ]
 
        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces  = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]
 
        # Define colors for each face
        #                down                   up
        self.colors = [(0,0,255),(255,255,0),(0,0,255),(255,255,0),(255,255,0),(255,255,0)]
 
        self.angle = 0

        # Create the quadrotor environment
        self.t=0.01
        self.quadSystem = QuadSim(self.t)

    def switch_k(self,argument):
        """ Auxiliar function to check the pressed keys"""
        switcher = {
            273: "Up",
            274: "Down",
            275: "Right",
            276: "Left",
            262: "Turn right",
            260: "Turn left",
        }
        print (switcher.get(argument, "Use the number pad"))

    def switch_keys(self,argument):
        """ Converts pressed keys into disturbances """
        switcher = {
            264: [0,-1,0],
            258: [0,1,0],
            262: [1,0,0],
            260: [-1,0,0],
            265: [0,0,3],
            263: [0,0,-3]
        }
        return switcher.get(argument, [0,0,0])
 
    def run(self):
        """ Main Loop """
        while 1:
            kicks = [0,0,0]
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                if event.type == KEYDOWN:
                    #print  str(event.dict['key'])
                    kicks = self.switch_keys(event.dict['key'])
            pygame.event.pump()
 
            self.clock.tick(int(1/self.t))
            self.screen.fill((0,32,0))
            
            # Get the system response
            quadrotor = self.quadSystem.run(kicks[0],kicks[1],kicks[2])

            pos_x = quadrotor[0]
            pos_y = quadrotor[1]
            pos_z = quadrotor[2]
            theta = quadrotor[3]
            phi = quadrotor[4]
            psi = quadrotor[5]

            # They will hold transformed vertices.
            t = []
            floor = []

            # Draw the ground
            for v in self.ground:
                p = v.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                floor.append(p)
 
            pointlist = [(floor[0].x, floor[0].y), (floor[1].x, floor[1].y),
                         (floor[1].x, floor[1].y), (floor[2].x, floor[2].y),
                         (floor[2].x, floor[2].y), (floor[3].x, floor[3].y),
                         (floor[3].x, floor[3].y), (floor[0].x, floor[0].y)]
            pygame.draw.polygon(self.screen,(150,150,150),pointlist,0)

            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis, and finally around Z axis.
                # Also converts the coordenade drone system to the screen's one
                r = v.rotateX(-phi).rotateY(-psi).rotateZ(-theta)
                # Traslation
                aux_x = r.x + pos_x
                aux_y = r.y + pos_z - 2
                aux_z = r.z + pos_y
                aux_point = Point3D(aux_x,aux_y,aux_z)
                # Transform the point from 3D to 2D
                p = aux_point.project(self.screen.get_width(), self.screen.get_height(), 256, 4)
                # Put the point in the list of transformed vertices
                t.append(p)
 
            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z)/4
                avg_z.append([i,z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)
 
            pygame.display.flip()

#====================================================================================================
# Run instruction
#====================================================================================================

if __name__ == "__main__":
    Simulation().run()
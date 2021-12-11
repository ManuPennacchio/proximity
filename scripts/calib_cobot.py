#! /usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import numpy as np
from numpy.core.defchararray import array
from numpy.core.fromnumeric import prod
#librarie per movimento
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from rospy.impl.transport import DeadTransport
from tf_conversions import posemath
from intera_interface import Limb

import argparse
from re import A, X
import sys
import json
import intera_interface


import matplotlib as mpl

from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d




##############################################################################################################################################
#DEFINIZIONE ELEMENTI DEL WS
##############################################################################################################################################
#offset(definito in base alla distanza letta dai proximity)
offs_0=[0,0,0]
offs_1=[0,0,0]
offs_2=[0,0,0]
offs_3=[0,0,0]
offs_4=[0,0,0]
#punti in coordinate ws
p0u=[0,0,0]+offs_0
p1u=[2*math.sqrt(5),0,0]+offs_1
p2u=[0,2*math.sqrt(5),0]+offs_2
p3u=[0,0,0]+offs_3
p4u=[0,0,0]+offs_4
#limiti del WS
x_lim=(-0.5,1.5)
y_lim=(-0.5,1.5)
z_lim=(-0.5,1.5)


##############################################################################################################################################
#CLASSE Calib
##############################################################################################################################################
#classe Waypoints
'''descrizione clase Waypoints: 
    la classe Waypoints deriva dalla modifica da uno script di base in Intera_examples
    permette di registrare una lista di punti nello spaizio dei giunti e di convertirli
    in Pose=[[x,y,z],[r,p,y]] grazie ad una funzione modificata da Limb e ad un metodo
    di conversione da Quaternion a [r,p,y]'''
class Waypoints(object):
    #costruttore
    def __init__(self, speed, accuracy, limb="right"):
        self.pose=[]
        # Create intera_interface limb instance
        self._arm = limb
        self._limb = intera_interface.Limb(self._arm)

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Recorded waypoints
        self._waypoints = list()
        self.waypoints=[]
        self._pose=list()

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator = intera_interface.Navigator()
    
    #metodo di inizio procedura di registrazione dei punti
    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            self._waypoints.append((self._limb.joint_angles()))
            
           
            self.print_waypoint(self._limb.joint_angles())
            self.waypoints.append([[self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).position],[self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).orientation]])
            
            
    
    #metodo che permette di salvare una posizione in un file                #
    def save_waypoints(self,waypoints):
        file=open("cartesian_pose.txt","w")
        for i in range(len(waypoints)):
            print("\n\n\nP"+str(i))
                #print("\nPosizione:"+"\n")
            print(waypoints[i])
                #print("\nOrientamento:"+str(i)+"\n")
                #print(waypoints[i].orientation)
        file.close()
        #original_stdout=sys.stdout
        #with open('positions.txt','w') as file:
            #for i in range(len(waypoints)):
                #print("\n\n\nP"+str(i))
                #print("\nPosizione:"+"\n")
                #print(waypoints[i])
                #print("\nOrientamento:"+str(i)+"\n")
                #print(waypoints[i].orientation)
        #sys.stdout=file
        #sys.stdout=original_stdout

    #metodo che permette di registrare una posizione
    def register_waypoint(self,waypoint):
        self.append(self._limb.joint_angles_to_cartesian_pose(waypoint))
        #self.rpy.append(self.from_quaternion_to_rpy(self._limb.joint_angles_to_cartesian_pose(pose).orientation))
    
    #metodo che permette di visualizzare una posizione
    def print_waypoint(self,waypoint):
        print("\nposition:\n"+str(self._limb.joint_angles_to_cartesian_pose(waypoint).position))
        print("\norientation:\n"+str(self._limb.joint_angles_to_cartesian_pose(waypoint).orientation)+"\n\n")
    

    #funzione che definisce l'interruzione della registrazione
    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            print("Recording Stopped")
            self._is_recording = False
    
    #metodo che permette di registrare i punti salvati
    def record(self,n_punti):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new cartesian position")
        # Connect Navigator callbacks
        # Navigator scroll wheel button press
        ok_id = self._navigator.register_callback(self._record_waypoint, 'right_button_ok')
        # Navigator Rethink button press
        show_id = self._navigator.register_callback(self._stop_recording, 'right_button_show')

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while not rospy.is_shutdown() and self._is_recording:
            rospy.sleep(1.0)

        # We are now done with the navigator callbacks, disconnecting them
        self._navigator.deregister_callback(ok_id)
        self._navigator.deregister_callback(show_id)
        #self.save_waypoints(self.waypoints)
        return self.waypoints



'''descrizione classe Calib:
    classe che permette di applicare i metodi di base per trovare la terna
    di trasformazione da coordinate work space a coordinate assolute del robot
    '''
class Calib:
    #costruttore
    def __init__(self):
        return
    
    #metodo che permette di avviare la procedura di calibrazione
    def get_points(self):
        """
        Records joint positions each time the navigator 'OK/wheel'
        button is pressed.
        il processo termina premendo il pulsante 'Rethink'
        """
        #inserimento numero punti di calibrazione
        controllo=False
        while controllo==False :
            print('definisci quanti punti usare per la calibrazione: ')
            n_punti=int(input())
            if n_punti==0 or n_punti==3 or n_punti==5: 
                controllo=True
        #acquisizione dei punti
        if(n_punti==3 or n_punti==5):
            arg_fmt = argparse.RawDescriptionHelpFormatter
            parser = argparse.ArgumentParser(formatter_class=arg_fmt)
            parser.add_argument(
                '-s', '--speed', default=0.3, type=float,
                help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
            )
            parser.add_argument(
                '-a', '--accuracy',
                default=intera_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
                help='joint position accuracy (rad) at which waypoints must achieve'
            )
            args = parser.parse_args(rospy.myargv()[1:])

            print("Initializing node... ")
            rospy.init_node("get_points_node", anonymous=True)
            waypoints = Waypoints(args.speed, args.accuracy)
            waypoints.record(n_punti)
            #waypoints.salva_punti(waypoints)


        if n_punti==5:
            p0=waypoints.waypoints[0]
            p1=waypoints.waypoints[1]
            p2=waypoints.waypoints[2]
            p3=waypoints.waypoints[3]
            p4=waypoints.waypoints[4]

            p5=[p0,p1,p2,p3,p4]
            return p5
        

        if n_punti==3:
            p0=waypoints.waypoints[0]
            p1=waypoints.waypoints[1]
            p2=waypoints.waypoints[2]

            p3=[p0,p1,p2]
            return p3
        
        if n_punti==0:
            return [[[],[]]]
    
    #metodo per trovare il piano che meglio approssima la nuvola di punti
    def fit_piano(self,p):
        xs=[]
        ys=[]
        zs=[]
        for i in range(len(p)):
            xs.append(p[i][0])
            ys.append(p[i][1])
            zs.append(p[i][2])
        tmp_A = []
        tmp_b = []
        for i in range(len(xs)):
            tmp_A.append([xs[i], ys[i], 1])
            tmp_b.append(zs[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)
        print("\npiano approssimato: %f x + %f y -z +%f = 0" % (fit[0], fit[1], fit[2]))
        print('\nscarti:\n')
        print('\nsomma residui quadratici:'+str(residual)+'\n')
        return ([float(fit[0]),float(fit[1]),float(-1),float(fit[2])])
    
    #metodo per proiettare una lista di punti su un piano definito
    def proiezione(self,p,piano):
        a=float(piano[0])
        b=float(piano[1])
        c=float(piano[2])
        d=float(piano[3])
        A=np.array([[0,c,-b],[c,0,-a],[a,b,c]])
        p1=[]
        print("proiezione sul piano: "+str(piano[0])+'x + '+str(piano[1])+'y + '+str(piano[2]) +'z + '+str(piano[3]) + "= 0 \n")
        for i in range(len(p)):
            xp=p[i][0]
            yp=p[i][1]
            zp=p[i][2]
            B=(np.array([[float(c*yp-b*zp)],[float(c*xp-a*zp)],[float(-d)]]))
            p1.append([np.linalg.solve(A,B).T[0][0],np.linalg.solve(A,B).T[0][1],np.linalg.solve(A,B).T[0][2]])
            print('\npunto '+str(i)+': '+str(p[i])+'---> '+str(p1[i]))
        return p1
    
    #metodo per ricavare la matrice di trasformazione
    def get_matrix(self,p):

        def get_matrix_3p(p):
            origin=p[0]
            v01=np.array([p[1][0]-p[0][0],p[1][1]-p[0][1],p[1][2]-p[0][2]])
            v02=np.array([p[2][0]-p[0][0],p[2][1]-p[0][1],p[2][2]-p[0][2]])
            k=(np.array(np.cross(v01,v02)/np.linalg.norm(np.cross(v01,v02))))
            i=(v01/np.linalg.norm(v01))
            j=(np.cross(k,i))

            rotazione=PyKDL.Rotation(i[0],i[1],i[2],j[0],j[1],j[2],k[0],k[1],k[2])
            posizione=PyKDL.Vector(origin[0],origin[1],origin[2])
            frame=PyKDL.Frame(rotazione,posizione)
            return frame

        def get_matrix_5p(p):
            calib=Calib()
            pp=calib.proiezione(p,self.fit_piano(p))
            return(get_matrix_3p([pp[0],pp[1],pp[2]]))
        if len(p)==3:
            return get_matrix_3p(p)
        if len(p)==5:
            return get_matrix_5p(p)
        if len(p)==0:
            return get_matrix_3p([[0,0,0,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
    




##############################################################################################################################################
#Classe Util
##############################################################################################################################################
'''descrizione clase Util: 
    la classe util contiene tutti i metodi necessari alla manipolazione di base
    degli elementi necessari ad altre classi'''
class Util:
    #costruttore
    def __init__(self):
        return

    #metodo per estrarre vettore di punti
    def from_pose_to_array(self,pose):
        p_position=[]
        p_orientation=[]
        for i in range(len(pose)):
            p_position.append(([(pose[i])[0][0].x,(pose[i])[0][0].y,(pose[i])[0][0].z]))
            p_orientation.append([(pose[i])[1][0].x,(pose[i])[1][0].y,(pose[i])[1][0].z,(pose[i])[1][0].w])
        return[p_position,p_orientation]


    #metodo per convertire da quaternione a rpy
    def from_quaternion_to_rpy(self,quaternion):
        x=quaternion.x
        y=quaternion.y
        z=quaternion.z
        w=quaternion.w

        t0=+2.0*(w*x+y*z)
        t1=+1.0-2.0*(x*x+y*y)
        r=math.atan2(t0,t1)

        t2=+2.0*(w*y-z*x)
        t2=+1.0 if t2>1.0 else t2
        t2=-1.0 if t2<-1.0 else t2
        p=math.asin(t2)

        t3=+2.0*(w*z+x*y)
        t4=+1.0-2.0*(y*y-z*z)
        y=math.atan2(t3,t4)

        return([r,p,y])

    #metodo convertire un Pose in un vettore [x,y,z]
    def from_pose_to_xyz(p):
        xyz=[]
        for i in range(len(p)):
            xyz.append([[p.position[i].x,p.position[i].y,p.position[i].z]])
        return xyz
    #metodo convertire un Pose in un vettore [r,p,y]
    '''def from_pose_to_rpy(p):
        rpy=[]
        for i in range(len(p)):
            rpy.append([[p.rpy[i].x,p.rpy[i].y,p.rpy[i].z]])
        return rpy'''
    def from_pose_to_quaternion(p):
        quaternion=[]
        for i in range(len(p)):
            quaternion.append([[p.orientation[i].x,p.orientation[i].y,p.orientation[i].z]])
        return quaternion

    
    #metodo per eseguire M*p
    def prodotto_mp(self,M,p):
        result=[]
        for i in range(len(p)):
            xu=p[i][0]
            yu=p[i][1]
            zu=p[i][2]
            xr=M[0,0]*xu+M[0,1]*yu+M[0,2]*zu+M[0,3]
            yr=M[1,0]*xu+M[1,1]*yu+M[1,2]*zu+M[1,3]
            zr=M[2,0]*xu+M[2,1]*yu+M[2,2]*zu+M[2,3]
            print(str(xr)+','+str(yr)+','+str(zr))
            result.append(np.array([xr,yr,zr]))
            return result
    
    #metodo per convertire un frame in una matrice
    def from_frame_to_matrix(self,frame):
        return(np.matrix([[frame[0,0],frame[0,1],frame[0,2],frame[0,3]],
                        [frame[1,0],frame[1,1],frame[1,2],frame[1,3]],
                        [frame[2,0],frame[2,1],frame[2,2],frame[0,3]],
                        [frame[3,0],frame[3,1],frame[3,2],frame[3,3]]]))

    #metodo per stampare la matrice
    def print_matrix(self,matrice):
	    for row in matrice:
		    print(row)
    


##############################################################################################################################################
#CLASSE Plot
##############################################################################################################################################
'''descrizione classe Plot: 
    la classe plot permette di visualizzare nello spazio tridimensionale i seguenti elementi:
    -liste di punti: plot_p(lista_punti[x,y,z])
    -piani: plot_plane (lista coeff di definizione del piano [a,b,c,d])
    -terne di trasformazione plot_frame(matrice_di_trasformazione 4X4)
    utilizzo:
    -inizializzare l'oggetto di classe Plot
    -eseguire le procedure necessarie
    -utilizzare il metodo plt.show() per vedere il risultato'''

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

class Plot:

    #costruttore della classe
    def __init__(self):
        self.ax=plt.axes(projection='3d')
        return None

    #metodo che permette di visualizzare una lista di punti
    def plot_p(self,p):
        ax=self.ax
        xs=[]
        ys=[]
        zs=[]
        for i in range(len(p)):
            ax.scatter(p[i][0], p[i][1], p[i][2], color='b')
            index="p"+str(i)
            ax.text(p[i][0], p[i][1], p[i][2], index)    
    
    #metodo che permette di visualizzare un pinao definito come [a,b,c,d]
    def plot_plane(self,piano):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        X,Y = np.meshgrid(np.arange(0.5, 0.5),
                          np.arange(0.5, 0.5))
        Z = np.zeros(X.shape)
        for r in range(X.shape[0]):
            for c in range(X.shape[1]):
                Z[r,c] = piano[0] * X[r,c] + piano[1] * Y[r,c] + piano[3]
        self.ax.plot_wireframe(X,Y,Z,color='w')

        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
    
    #metodo che permette di visualizzare una terna di trasformazione in forma matriciale M(4x4)
    def plot_frame(self,M):

        ax=self.ax
        arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0)

        xr = Arrow3D([0, 1], [0, 0], [0, 0], **arrow_prop_dict, color='r')
        ax.add_artist(xr)
        yr = Arrow3D([0, 0], [0, 1], [0, 0], **arrow_prop_dict, color='b')
        ax.add_artist(yr)
        zr = Arrow3D([0, 0], [0, 0], [0, 1], **arrow_prop_dict, color='g')
        ax.add_artist(zr)

        x_ws = Arrow3D([M[0,3], M[0,3]+M[0,0]], [M[1,3], M[1,3]+M[0,1]], [M[2,3], M[2,3]+M[0,2]], **arrow_prop_dict, color='r')
        ax.add_artist(x_ws)
        y_ws = Arrow3D([M[0,3], M[0,3]+M[1,0]], [M[1,3], M[1,3]+M[1,1]], [M[2,3], M[2,3]+M[1,2]], **arrow_prop_dict, color='b')
        ax.add_artist(y_ws)
        z_ws = Arrow3D([M[0,3], M[0,3]+M[2,0]], [M[1,3], M[1,3]+M[2,1]], [M[2,3], M[2,3]+M[2,2]], **arrow_prop_dict, color='g')
        ax.add_artist(z_ws)
        t = Arrow3D([0, M[0,3]], [0, M[1,3]], [0, M[2,3]], **arrow_prop_dict, color='y')
        ax.add_artist(t)

        ax.text(0.0, 0.0, -0.1, r'$o_R$')
        ax.text(1.1, 0, 0, r'$x_R$')
        ax.text(0, 1.1, 0, r'$y_R$')
        ax.text(0, 0, 1.1, r'$z_R$')

        ax.text(M[0,3]-0.1*M[0,3], M[1,3]-0.1*M[1,3], M[2,3]-M[2,3]*0.1, r'$o(WS)$')
        ax.text(M[0,3]+M[0,0]+0.1*M[0,0], M[1,3]+M[0,1]+0.1*M[0,1], M[2,3]+M[0,2]+0.1*M[0,2], r'$x(WS)$')
        ax.text(M[1,3]+M[1,0]+0.1*M[1,0], M[1,3]+M[1,1]+0.1*M[1,1], M[2,3]+M[1,2]+0.1*M[1,2], r'$y(WS)$')
        ax.text(M[1,3]+M[2,0]+0.1*M[2,0], M[1,3]+M[2,1]+0.1*M[2,1], M[2,3]+M[2,2]+0.1*M[2,2], r'$z(WS)$')

        ax.view_init(azim=20, elev=10)
        #ax.set_axis_off()
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_zlim(z_lim)




##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():

    calib=Calib()
    util=Util()
    plot=Plot()

    p=calib.get_points()
    [p_position,p_orientation]=util.from_pose_to_array(p)
    #print(p_orientation)
    M=calib.get_matrix(p_position)
    
    plot.plot_frame(M)
    plot.plot_p(p_position)
    #plot.plot_plane(calib.fit_piano(p_position))
    plt.show()



    
    
if __name__ == '__main__':
    main()
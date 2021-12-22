#! /usr/bin/env python3
from re import M
import rospy
import numpy as np
import intera_interface
from intera_interface import Limb
import math
import argparse
import sys
import json

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import tf_conversions.posemath as pm

import PyKDL
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf_conversions import posemath
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from geometry_msgs.msg import PoseStamped



##############################################################################################################################################
#DEFINIZIONE ELEMENTI DEL WS
##############################################################################################################################################
#offset da tenere dai marker durante le movimentazioni (in metri)
offs=0.01
grasp=0.1
speed_movements=0.3
speed_grasp=0.1

#punti in coordinate x,y,z del sistema di riferimento ws
p_ws=list()
p_ws.append([0,0,0])
p_ws.append([0.2,0,0])
p_ws.append([0,0.2,0])
p_ws.append([0.2,0,2,0])
p_ws.append([0.25,0.25,0])

#creazione dei dati Pose utilizzati per le movimentazioni
pose_ws=list()
pose_ws_grasp=list()
#quaternione di base, utilizzato solo per la creazione del dato Pose
o_ws=Quaternion(1,0,0,0)
for i in range(len(p_ws)):
    pose_ws.appen(Pose(Point(p_ws[i][0]+offs,-(p_ws[i][1]+offs),p_ws[i][2]+grasp),o_ws))
    pose_ws_grasp.append(Pose(Point(p_ws[i][0]+offs,-(p_ws[i][1]+offs),p_ws[i][2]+grasp),o_ws))
pose_ws_home= Pose(Point(p_ws[i][0]+offs,-(p_ws[i][1]+offs),p_ws[i][2]+2*grasp),o_ws)


##############################################################################################################################################
#CLASSE Mover
##############################################################################################################################################

class CartesianMover():
    '''
    classe che consenta di realizzare una movimentazione di tipo cartesiano mantenedo un orientamento costante
    è possibile decidere tra due tipi di movimentazione:
    - 1 = veloce, per arrivare alla posizione di grasp
    - 2 = lenta, per muoversi dalla posizione di grasp a quella desiderata
    '''
    def __init__(self,linear_speed,Linear_acceleration):
        self.linear_speed=linear_speed
        self.linear_accel=Linear_acceleration
        self.rotational_speed = 1.57
        self.rotational_accel = 1.57
        self.timeout=None
        self.tip_name = 'right_hand'
        self.util=Util()


    def MoveToPosition(self,p,M,orientation):
        try:
            #rospy.init_node('go_to_cartesian_pose_py')
            limb = Limb()
            traj_options = TrajectoryOptions()
            traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
            traj = MotionTrajectory(trajectory_options=traj_options, limb=limb)


            wpt_opts = MotionWaypointOptions(max_linear_speed=self.linear_speed,
                                             max_linear_accel=self.linear_accel,
                                             max_rotational_speed=self.rotational_speed,
                                             max_rotational_accel=self.rotational_accel,
                                             max_joint_speed_ratio=1.0)
            waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)


            p.position.y=(-1)*p.position.y
            pose=posemath.toMsg(M*(posemath.fromMsg(p)))

            pose.orientation=orientation

            poseStamped = PoseStamped()

            poseStamped.pose = pose
            
            # using current joint angles for nullspace bais if not provided
            joint_angles = limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(
                poseStamped, self.tip_name, joint_angles)

            rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=self.timeout)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return

            if result.result:
                rospy.loginfo(
                    'Motion controller successfully finished the trajectory!')
            else:
                rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                             result.errorId)
        except rospy.ROSInterruptException:
            rospy.logerr(
                'Keyboard interrupt detected from the user. Exiting before trajectory completion.')



##############################################################################################################################################
#CLASSE Waypoints
##############################################################################################################################################
'''descrizione clase Waypoints: 
    la classe Waypoints deriva dalla modifica da uno script di base in Intera_examples
    permette di registrare una lista di punti nello spaizio dei giunti e di convertirli
    in Pose=[[x,y,z],[x,y,z,w]] grazie alla funzione joint_angles_to_cartesian_pose contenuta in Limb'''
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

        # Recorded Poses
        self.poses=list()

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
    def _record_Pose(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("\nWaypoint Recorded\n")
            real_time_joint_position=self._limb.joint_angles()
           
            #offs_x=float(input('\nvalore offset asse x: '))
            #offs_y=float(input('\nvalore offset asse y: '))
            #offs_z=float(input('\nvalore offset asse z: '))

            offs_x=0
            offs_y=0
            offs_z=0

            real_time_position=self._limb.joint_angles_to_cartesian_pose(real_time_joint_position).position

            position=Point(real_time_position.x+offs_x,real_time_position.y+offs_y,real_time_position.z+offs_z)
            orientation=self._limb.joint_angles_to_cartesian_pose(real_time_joint_position).orientation

            self.poses.append(Pose(position,orientation))
            
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
    def record(self):
        """
        Records joint position poses upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Poses Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new Pose")
        # Connect Navigator callbacks
        # Navigator scroll wheel button press
        ok_id = self._navigator.register_callback(self._record_Pose, 'right_button_ok')
        # Navigator Rethink button press
        show_id = self._navigator.register_callback(self._stop_recording, 'right_button_show')

        # Set recording flag
        self._is_recording = True

        # Loop until poes are done being recorded ('Rethink' Button Press)
        while not rospy.is_shutdown() and self._is_recording:
            rospy.sleep(1.0)

        # We are now done with the navigator callbacks, disconnecting them
        self._navigator.deregister_callback(ok_id)
        self._navigator.deregister_callback(show_id)
        return self.poses



##############################################################################################################################################
#CLASSE Calib
##############################################################################################################################################
'''descrizione classe Calib:
    classe che permette di applicare i metodi di base per trovare la terna
    di trasformazione da coordinate work space a coordinate assolute del robot
    '''
class Calib:
    #costruttore
    def __init__(self):
        self.util=Util()
        self.speed=0.3
        self.accuracy=intera_interface.settings.JOINT_ANGLE_TOLERANCE
        return
    
    #metodo che permette di avviare la procedura di calibrazione
    def get_points(self):
        """
        Records joint positions each time the navigator 'OK/wheel'
        button is pressed.
        il processo termina premendo il pulsante 'Rethink'
        """

        #descrizione e istruzioni del metodo
        print('\n\n\ACQUISIZIONE DEI PUNTI DI CALIBRAZIONE:')
        print('\nISTRUZIONI:')
        print('\n- il primo punto definisce l\'origine e l\'orientamento dell\'end effector')
        print('\n- il secondo punto definisce l\'asse x')
        print('\n(i punti extra registrati non verranno considerati)\n\n')
        #input numero punti di calibrazione
        controllo=False
        while controllo==False :
            print('\n\nDefinisci quanti punti usare per la calibrazione (almeno 3): ')
            n_punti_calibrazione=int(input())
            if n_punti_calibrazione==0 or n_punti_calibrazione>=3: 
                controllo=True
            else:
                print('\nvalore inserito non accettabile\n\n')
        #acquisizione dei punti
        if(n_punti_calibrazione>=3):

            print("\nInizializzazione del nodo...")
            rospy.init_node("get_points_node", anonymous=True)
            waypoints = Waypoints(self.speed, self.accuracy)

            waypoints.record()
            
            poses=list()
            orientation=poses[0].orientation
            for i in range(n_punti_calibrazione):
                poses.append(waypoints.waypoints[i])


            return [poses,orientation]
    
    #metodo per trovare il piano che meglio approssima la nuvola di punti
    def fit_piano(self,poses):
        [p_position,p_orientation]=self.util.from_poses_to_array(poses)
        xs=[]
        ys=[]
        zs=[]
        for i in range(len(p_position)):
            xs.append(p_position[i][0])
            ys.append(p_position[i][1])
            zs.append(p_position[i][2])
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
    def proiezione(self,poses,piano):
        [p,orientation]=self.util.from_poses_to_array(poses)
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
    def get_matrix(self,poses):
        [p,orientation]=self.util.from_poses_to_array(poses)
        calib=Calib()
        plot=Plot()
        def get_matrix_3p(p):
            origin=p[0]
            v01=np.array([p[1][0]-p[0][0],p[1][1]-p[0][1],p[1][2]-p[0][2]])
            v02=np.array([p[2][0]-p[0][0],p[2][1]-p[0][1],p[2][2]-p[0][2]])
            k=(np.array(np.cross(v01,v02)/np.linalg.norm(np.cross(v01,v02))))
            i=(v01/np.linalg.norm(v01))
            j=(np.cross(i,k))

            rotazione=PyKDL.Rotation(i[0],i[1],i[2],j[0],j[1],j[2],k[0],k[1],k[2])
            posizione=PyKDL.Vector(origin[0],origin[1],origin[2])
            frame=PyKDL.Frame(rotazione,posizione)
            
            plot.plot_frame(frame)
            plot.plot_p(p)
            plot.plot_plane(calib.fit_piano(poses))
            plt.show()
            return frame

        def get_matrix_np(p):
            calib=Calib()
            pp=calib.proiezione(poses,calib.fit_piano(poses))
            plot.plot_p(p)
            return(get_matrix_3p([pp[0],pp[1],pp[2]]))
            
        def get_matrix_0p():
            return(get_matrix_3p([[0,0,0],[1,0,0],[0,1,0]]))
        
        if len(p)==3:
            return get_matrix_3p(p)
        if len(p)>3:
            return get_matrix_np(p)
        if len(p)==0:
            return get_matrix_0p()
    


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
        p_position=[pose.position.x,pose.position.y,pose.position.z]
        p_orientation=[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        return[p_position,p_orientation]

    #metodo per estrarre vettore di punti
    def from_poses_to_array(self,pose):
        p_position=[]
        p_orientation=[]
        for i in range(len(pose)):
            p_position.append([(pose[i]).position.x,(pose[i]).position.y,(pose[i]).position.z])
            p_orientation.append([(pose[i]).orientation.x,(pose[i]).orientation.y,(pose[i]).orientation.z,(pose[i]).orientation.w])
        return[p_position,p_orientation]

    #metodo che blocca l'esecuzione fino che non viene premuto un tasto
    def wait_key():
        input('Premi un pulsante qualsiasi per continuare...')
        return None


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

    #limiti della rappresentazione del ws
    x_lim=(-0.5,1.5)
    y_lim=(-0.5,1.5)
    z_lim=(-0.5,1.5)
    dim_ax=0.5

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
            ax.scatter(p[i][0], p[i][1], p[i][2], color='black')
            index="p"+str(i)
            ax.text(p[i][0], p[i][1], p[i][2]+0.075, index)    
    
    #metodo che permette di visualizzare un pinao definito come [a,b,c,d]
    def plot_plane(self,piano):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x=np.arange(xlim[0],xlim[1],1)
        y=np.arange(ylim[0],ylim[1],1)
        X,Y = np.meshgrid(x,y)
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
        arrow_prop_dict_t = dict(mutation_scale=20, arrowstyle='-', shrinkA=0, shrinkB=0)

        xr = Arrow3D([0, 1*self.dim_ax], [0, 0], [0, 0], **arrow_prop_dict, color='r')
        ax.add_artist(xr)
        yr = Arrow3D([0, 0], [0, 1*self.dim_ax], [0, 0], **arrow_prop_dict, color='b')
        ax.add_artist(yr)
        zr = Arrow3D([0, 0], [0, 0], [0, 1*self.dim_ax], **arrow_prop_dict, color='g')
        ax.add_artist(zr)

        x_ws = Arrow3D([M[0,3], M[0,3]+(M[0,0]*self.dim_ax)], [M[1,3], M[1,3]+(M[0,1]*self.dim_ax)], [M[2,3], M[2,3]+(M[0,2]*self.dim_ax)], **arrow_prop_dict, color='r')
        ax.add_artist(x_ws)
        y_ws = Arrow3D([M[0,3], M[0,3]+(M[1,0]*self.dim_ax)], [M[1,3], M[1,3]+(M[1,1]*self.dim_ax)], [M[2,3], M[2,3]+(M[1,2]*self.dim_ax)], **arrow_prop_dict, color='b')
        ax.add_artist(y_ws)
        z_ws = Arrow3D([M[0,3], M[0,3]+(M[2,0]*self.dim_ax)], [M[1,3], M[1,3]+(M[2,1]*self.dim_ax)], [M[2,3], M[2,3]+(M[2,2]*self.dim_ax)], **arrow_prop_dict, color='g')
        ax.add_artist(z_ws)


        t0 = Arrow3D([0, M[0,3]*(1-self.dim_ax)], [0, M[1,3]*(1-self.dim_ax)], [0, M[2,3]*(1-self.dim_ax)], **arrow_prop_dict_t, color='y')
        ax.add_artist(t0)
        t1 = Arrow3D([M[0,3]*(1-self.dim_ax),M[0,3]], [M[1,3]*(1-self.dim_ax),M[1,3]], [M[2,3]*(1-self.dim_ax),M[2,3]], **arrow_prop_dict, color='y')
        ax.add_artist(t1)

        ax.text(-self.dim_ax/10, 0.0, -self.dim_ax/10, r'$o_R$',color='y')
        ax.text(self.dim_ax+self.dim_ax/10, 0, 0, r'$x_R$',color='r')
        ax.text(0, self.dim_ax+self.dim_ax/10, 0, r'$y_R$',color='b')
        ax.text(0, 0, self.dim_ax+self.dim_ax/10, r'$z_R$',color='g')

        ax.text(M[0,3]+(M[0,0]+self.dim_ax/10*M[0,0])*self.dim_ax, M[1,3]+(M[0,1]+self.dim_ax/10*M[0,1])*self.dim_ax, M[2,3]+(M[0,2]+self.dim_ax/10*M[0,2])*self.dim_ax, r'$x(WS)$',color='r')
        ax.text(M[1,3]+(M[1,0]+self.dim_ax/10*M[1,0])*self.dim_ax, M[1,3]+(M[1,1]+self.dim_ax/10*M[1,1])*self.dim_ax, M[2,3]+(M[1,2]+self.dim_ax/10*M[1,2])*self.dim_ax, r'$y(WS)$',color='b')
        ax.text(M[2,3]+(M[2,0]+self.dim_ax/10*M[2,0])*self.dim_ax, M[1,3]+(M[2,1]+self.dim_ax/10*M[2,1])*self.dim_ax, M[2,3]+(M[2,2]+self.dim_ax/10*M[2,2])*self.dim_ax, r'$z(WS)$',color='g')
        #ax.text(M[0,3],M[1,3],M[2,3]-self.dim_ax/10, r'$o(WS)$',color='y')
        
        ax.view_init(azim=20, elev=10)
        #ax.set_axis_off()
        ax.set_xlim(self.x_lim)
        ax.set_ylim(self.y_lim)
        ax.set_zlim(self.z_lim)


##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():

    calib=Calib()
    util=Util()

    [poses,orientation]=calib.get_points()
    M=calib.get_matrix(poses)


    #definizione del percorso
    movimento_veloce=CartesianMover(0.1,0.6)
    movimento_lento=CartesianMover(0.01,0.1)
    util.wait_key()
    movimento_veloce.MoveToPosition(pose_ws_home,M,orientation)
    #util.wait_key()
    while i<30:
        movimento_veloce.MoveToPosition(pose_ws_grasp[0],M,orientation)
        movimento_lento.MoveToPosition(pose_ws[0],M,orientation)
        #util.wait_key()
        movimento_veloce.MoveToPosition(pose_ws_grasp[1],M,orientation)
        movimento_lento.MoveToPosition(pose_ws[1],M,orientation)
        #util.wait_key()
        movimento_veloce.MoveToPosition(pose_ws_grasp[2],M,orientation)
        movimento_lento.MoveToPosition(pose_ws[2],M,orientation)
        #util.wait_key()
        movimento_veloce.MoveToPosition(pose_ws_grasp[3],M,orientation)
        movimento_lento.MoveToPosition(pose_ws[3],M,orientation)
        #util.wait_key()
        movimento_veloce.MoveToPosition(pose_ws_grasp[4],M,orientation)
        movimento_lento.MoveToPosition(pose_ws[4],M,orientation)
        #util.wait_key()
        i=i+1
    movimento_veloce.MoveToPosition(pose_ws_home,M,orientation)
    
if __name__ == '__main__':
    main()
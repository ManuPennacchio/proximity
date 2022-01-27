#! /usr/bin/env python3

from turtle import position
import rospy
import numpy as np
import intera_interface
from intera_interface import Limb
import math
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
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
#oparametri del workspace (in metri)
offs=0.03
grasp=0.1
l=0.468
h=0.595
alpha=math.atan(h/l)
a=0.1*math.sqrt(l**2+h**2)*math.cos(alpha)
b=0.1*math.sqrt(l**2+h**2)*math.sin(alpha)
#parametri di traiettoria
relative_orientation=Quaternion(0,1,0,0)
offs_x=-0.01
offs_y=-0.01
offs_z=0.01
grasp_x=-0
grasp_y=-0
grasp_z=0.05
v_spostamenti=1.5
v_avvicinamenti=0.1
a_spostamenti=1.0
a_avvicinamenti=0.01
#limiti del WS (utilizzato per il plot)
x_lim=(-0.5,1.5)
y_lim=(-0.5,1.5)
z_lim=(-0.5,1.5)
dim_ax=0.5


##############################################################################################################################################
#CLASSE Mover
##############################################################################################################################################

class Cartesian_mover():
    """ create a cartesian mover thread and move cobot to position=[x,y,z] coordinats in meters. When cobot is in position it calls the given callback function  """
    def __init__(self,speed,accel):
        self.util=Util()
        self.DefaultValues()
        self.linear_speed=speed
        self.linear_accel=accel
        
    
    def run(self):
        self.MoveToPosition()
        self.callback(self)


    def DefaultValues(self):
        self.rotational_speed = 1.57
        self.rotational_accel = 1.57
        self.timeout=None
        self.tip_name = 'right_hand'

    def move_to_position(self,p,M):
        try:
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

            #trasformazione position e orientation
            pose=Pose()
            pose.position.x = p.position.x*M[0][0]+p.position.y*M[0][1]+p.position.z*M[0][2]+M[0][3]
            pose.position.y = p.position.x*M[1][0]+p.position.y*M[1][1]+p.position.z*M[1][2]+M[1][3]
            pose.position.z = p.position.x*M[2][0]+p.position.y*M[2][1]+p.position.z*M[2][2]+M[2][3]
            pose.orientation.x=p.orientation.x*M[0][0]+p.orientation.y*M[0][1]+p.orientation.z*M[0][2]+p.orientation.w*M[0][3]
            pose.orientation.y=p.orientation.x*M[1][0]+p.orientation.y*M[1][1]+p.orientation.z*M[1][2]+p.orientation.w*M[1][3]
            pose.orientation.z=p.orientation.x*M[2][0]+p.orientation.y*M[2][1]+p.orientation.z*M[2][2]+p.orientation.w*M[2][3]
            pose.orientation.w=p.orientation.x*M[3][0]+p.orientation.y*M[3][1]+p.orientation.z*M[3][2]+p.orientation.w*M[3][3]
            poseStamped = PoseStamped()
            poseStamped.pose.position = pose.position
            poseStamped.pose.orientation = pose.orientation
            
            # using current joint angles for nullspace bais if not provided
            joint_angles = limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(poseStamped, self.tip_name, joint_angles)

            #rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=self.timeout)
            if result is None:
                rospy.logerr('Trajectory FAILED to send')

                return
            
            if result.result:
                rospy.loginfo('Traiettoria completata con successo')
            else:
                rospy.logerr('Triettoria non completata %s',result.errorId)
                input('Premi invio per continuare la traiettoria saltanto il prossimo target...')
        except rospy.ROSInterruptException:
            rospy.logerr('')



##############################################################################################################################################
#CLASSE Waypoints
##############################################################################################################################################
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
        if value:
            print("Pose registrata!")
            p_position=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).position
            position_offs=Point(p_position.x,p_position.y,p_position.z)
            orientation=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).orientation
            self.waypoints.append(Pose(position_offs,orientation))

            print("\nposition:\n"+str(position))
            print("\norientation:\n"+str(orientation)+"\n\n")


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
        print("Press Navigator 'OK/Wheel' button to record a new cartesian pose")
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
        return self.waypoints



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
        return
    
    #metodo che permette di avviare la procedura di calibrazione
    def get_points(self):
        speed=0.3
        accuracy=0.008726646
        """
        Records joint positions each time the navigator 'OK/wheel'
        button is pressed.
        il processo termina premendo il pulsante 'Rethink'
        """
        #inserimento numero punti di calibrazione
        controllo=False
        while controllo==False :
            print('definisci quanti punti usare per la calibrazione\n(0=nessuna calibrazione): ')
            n_punti=int(input())
            if n_punti==0 or n_punti>=3: 
                controllo=True
        #acquisizione dei punti
        if(n_punti==0 or n_punti>=3):
            print("Initializing node... ")
            rospy.init_node("get_points_node", anonymous=True)
            waypoints = Waypoints(speed, accuracy)
            waypoints.record(n_punti)
            
            poses=[]
            for i in range(len(waypoints.waypoints)):
                poses.append(waypoints.waypoints[i])
            return [poses,n_punti]
    
    #metodo per trovare il piano che meglio approssima la nuvola di punti utilizzando il metodo degli scarti quadratici minimi
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
    
    #metodo per proiettare una lista di Pose su un piano definito
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
        '''
                ┌────────────────┐
                │P2            P3│      ◆ nessuna calibrazione: la metrice di trasformazine corrisponde ad un'identità
                │ \            / │      ◆ calibrazione a 3 punti: P0,P1,P2 definiscono il piano in modo esatto
                │  \          /  │      ◆ calibrazione a n punti: P0,P1,...,pn definiscono il piano con il metodo degli scarti quadratici minimi
                │   \        /   │      
                │    \      /    │      P0=nuova origine
                │     \    /     │      P0-->P1=direzione dell'asse x
                │      \  /      │      z=asse passante per P0 e perpendicolare al piano
                │       \/       │      y=asse pertpendicolare a x e y (NB: non passa esattamente per P2)
                │       P4       │
                │      /  \      │
                │     /    \     │      NB: SELEZIONARE I PRIMI 3 PUNTI NEL SEGUENTE ORDINE: P0,P1,P2
                │    /      \    │
                │   /        \   │
        y       │  /          \  │
        ▲       │P0            P1│
        ┃       └────────────────┘       
        ┗━━➤x    
    
        '''
        def get_matrix_3p(p):
            origin=p[0]
            v01=np.array([p[1][0]-p[0][0],p[1][1]-p[0][1],p[1][2]-p[0][2]])
            v02=np.array([p[2][0]-p[0][0],p[2][1]-p[0][1],p[2][2]-p[0][2]])
            k=(np.array(np.cross(v01,v02)/np.linalg.norm(np.cross(v01,v02))))
            i=(v01/np.linalg.norm(v01))
            j=(np.cross(k,i))

            mat=[[i[0],j[0],k[0],origin[0]],
                 [i[1],j[1],k[1],origin[1]],
                 [i[2],j[2],k[2],origin[2]],
                 [0.0, 0.0, 0.0, 1.0]]

            plot.plot_frame(mat)
            plot.plot_p(p)
            plot.plot_plane(calib.fit_piano(poses))
            plt.show()
            return mat

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
    def from_poses_to_array(self,pose):
        p_position=[]
        p_orientation=[]
        for i in range(len(pose)):
            p_position.append([(pose[i]).position.x,(pose[i]).position.y,(pose[i]).position.z])
            p_orientation.append([(pose[i]).orientation.x,(pose[i]).orientation.y,(pose[i]).orientation.z,(pose[i]).orientation.w])
        return[p_position,p_orientation]

    #metodo che blocca l'esecuzione fino che non viene premuto un tasto
    def wait_key(self):
        input('Premi invio per continuare...')
    
    

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
            ax.scatter(p[i][0], p[i][1], p[i][2], color='black')
            index="p"+str(i)
            ax.text(p[i][0], p[i][1], p[i][2]+0.075, index)    
    
    #metodo che permette di visualizzare un pinao definito come [a,b,c,d]
    def plot_plane(self,piano):
        x=np.linspace(x_lim[0],x_lim[1],10)
        y=np.linspace(y_lim[0],y_lim[1],10)
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

        xr = Arrow3D([0, 1*dim_ax], [0, 0], [0, 0], **arrow_prop_dict, color='r')
        ax.add_artist(xr)
        yr = Arrow3D([0, 0], [0, 1*dim_ax], [0, 0], **arrow_prop_dict, color='b')
        ax.add_artist(yr)
        zr = Arrow3D([0, 0], [0, 0], [0, 1*dim_ax], **arrow_prop_dict, color='g')
        ax.add_artist(zr)

        x_ws = Arrow3D([M[0][3], M[0][3]+(M[0][0]*dim_ax)], [M[1][3], M[1][3]+(M[1][0]*dim_ax)], [M[2][3], M[2][3]+(M[2][0]*dim_ax)], **arrow_prop_dict, color='r')
        ax.add_artist(x_ws)
        y_ws = Arrow3D([M[0][3], M[0][3]+(M[0][1]*dim_ax)], [M[1][3], M[1][3]+(M[1][1]*dim_ax)], [M[2][3], M[2][3]+(M[2][1]*dim_ax)], **arrow_prop_dict, color='b')
        ax.add_artist(y_ws)
        z_ws = Arrow3D([M[0][3], M[0][3]+(M[0][2]*dim_ax)], [M[1][3], M[1][3]+(M[1][2]*dim_ax)], [M[2][3], M[2][3]+(M[2][2]*dim_ax)], **arrow_prop_dict, color='g')
        ax.add_artist(z_ws)

        t0 = Arrow3D([0, M[0][3]*(0.9)], [0, M[1][3]*(0.9)], [0, M[2][3]*(0.9)], **arrow_prop_dict_t, color='y')
        ax.add_artist(t0)
        t1 = Arrow3D([M[0][3]*(0.9),M[0][3]], [M[1][3]*(0.9),M[1][3]], [M[2][3]*(0.9),M[2][3]], **arrow_prop_dict, color='y')
        ax.add_artist(t1)

        ax.text(-0.1*dim_ax, 0.0, -0.1*dim_ax, r'$o_R$',color='y')
        ax.text(1.1*dim_ax, 0, 0, r'$x_R$',color='r')
        ax.text(0, 1.1*dim_ax+0.04, 0, r'$y_R$',color='b')
        ax.text(0, 0, 1.1*dim_ax, r'$z_R$',color='g')

        ax.text(M[0][3]+M[0][0]*1.1*dim_ax, M[1][3]+M[1][0]*1.1*dim_ax, M[2][3]+M[2][0]*1.1*dim_ax, r'$x(WS)$',color='r')
        ax.text(M[0][3]+M[0][1]*1.1*dim_ax, M[1][3]+M[1][1]*1.1*dim_ax, M[2][3]+M[2][1]*1.1*dim_ax, r'$y(WS)$',color='b')
        ax.text(M[0][3]+M[0][2]*1.1*dim_ax, M[1][3]+M[1][2]*1.1*dim_ax, M[2][3]+M[2][2]*1.1*dim_ax, r'$z(WS)$',color='g')

        ax.view_init(azim=20, elev=10)
        ax.set_xlim(x_lim)
        ax.set_ylim(y_lim)
        ax.set_zlim(z_lim)


##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():

    calib=Calib()
    util=Util()
    [poses,n_punti]=calib.get_points()

    pose_ws=[]
    pose_ws_grasp=[]
    if n_punti==0:
        #nessuna calibrazione
        M=calib.get_matrix([])
        #[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]] #la metrice di rototraslazione corrisponde a un'identità
        for i in range(len(poses)):
            pose_ws.append(Pose(Point(poses[i].position.x+offs_x,poses[i].position.y+offs_y,poses[i].position.z+offs_z),poses[i].orientation))
            pose_ws_grasp.append(Pose(Point(poses[i].position.x+offs_x+grasp_x,poses[i].position.y+offs_y+grasp_y,poses[i].position.z+offs_z+grasp_z),poses[i].orientation))
    else:
        #calibrazione con più punti
        M=calib.get_matrix(poses)
        position_ws=list()
        position_ws.append([0,0,0])
        position_ws.append([l-2*a,0,0])
        position_ws.append([l-2*a,h-2*b,0])
        position_ws.append([0,h-2*b,0])
        position_ws.append([l/2-a,h/2-b,0])

        for i in range(len(position_ws)):
            pose_ws.append(Pose(Point(position_ws[i][0]+offs_x,position_ws[i][1]+offs_y,position_ws[i][2]+offs_z),relative_orientation))
            pose_ws_grasp.append(Pose(Point(position_ws[i][0]+offs_x+grasp_x,position_ws[i][1]+offs_y+grasp_y,position_ws[i][2]+offs_z+grasp_z),relative_orientation))

    #definizione del percorso
    movimento_veloce=Cartesian_mover(v_spostamenti,a_spostamenti)
    movimento_lento=Cartesian_mover(v_avvicinamenti,a_avvicinamenti)
    input('premere il tasto INVIO per iniziare la prima triettoria')
    for i in range(30):
        movimento_veloce.move_to_position(pose_ws_grasp[0],M)
        movimento_lento.move_to_position(pose_ws[0],M)
        #util.wait_key()
        movimento_veloce.move_to_position(pose_ws_grasp[0],M)
        movimento_veloce.move_to_position(pose_ws_grasp[1],M)
        movimento_lento.move_to_position(pose_ws[1],M)
        #util.wait_key()
        movimento_veloce.move_to_position(pose_ws_grasp[1],M)
        movimento_veloce.move_to_position(pose_ws_grasp[2],M)
        movimento_lento.move_to_position(pose_ws[2],M)
        #util.wait_key()
        movimento_veloce.move_to_position(pose_ws_grasp[2],M)
        movimento_veloce.move_to_position(pose_ws_grasp[3],M)
        movimento_lento.move_to_position(pose_ws[3],M)
        #util.wait_key()
        movimento_veloce.move_to_position(pose_ws_grasp[3],M)
        movimento_veloce.move_to_position(pose_ws_grasp[4],M)
        movimento_lento.move_to_position(pose_ws[4],M)
        #util.wait_key()
        movimento_veloce.move_to_position(pose_ws_grasp[4],M)

    
    
if __name__ == '__main__':
    main()

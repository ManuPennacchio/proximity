#! /usr/bin/env python3
from sre_constants import CATEGORY_LINEBREAK
from tkinter import *
import os
import string
from time import sleep
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
from PIL import ImageTk, Image



##############################################################################################################################################
#DEFINIZIONE DELLE COSTANTI
##############################################################################################################################################
STEP_CONTROLLER=0
#parametri del workspace (in metri)
L=0.468
H=0.595
ALPHA=math.atan(H/L)
A=0.1*math.sqrt(L**2+H**2)*math.cos(ALPHA)
B=0.1*math.sqrt(L**2+H**2)*math.sin(ALPHA)
OFFS_X=-0.01    #negativo
OFFS_Y=-0.01    #negativo
OFFS_Z=0.01     #positivo
GRASP_X=-0.0    #negativo
GRASP_Y=-0.0    #negativo
GRASP_Z=0.05    #positivo

#parametri ddelle prove
V_POSE=0.1
A_POSE=0.01
V_GRASP=1.2
A_GRASP=1
V_TRY_POSE=0.1
A_TRY_POSE=0.1
V_TRY_GRASP=0.1
A_TRY_GRASP=0.1
V_CONTROLLER=0.2
A_CONTROLLER=0.1
N_PROVE=30

#limiti del WS (utilizzato per il plot)
X_LIM=(-0.5,1.5)
Y_LIM=(-0.5,1.5)
Z_LIM=(-0.5,1.5)
DIM_AX=0.5



##############################################################################################################################################
#Classe Interface
##############################################################################################################################################
poses=list()
relative_orientation=Quaternion(0,0,0,0)
'''descrizione classe Interface:
    la classe interface crea un'interfaccia intuitiva e semplificata nell'utilizzo del programma
'''
n_punti=0
calibration=None
class Interface():
    def __init__(self):
        limb="right"
        self.mover=Cartesian_mover(V_CONTROLLER,A_CONTROLLER)
        self._arm = limb
        self._limb = intera_interface.Limb(self._arm)
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self._navigator = intera_interface.Navigator()
        util=Util()
        pass

    #finestra di dialogo necessaria a definire il tipo di calibrazione
    def calibr(self):
        def _calib():
            global calibration
            if (tkvarq.get())==options[0]:
                calibration=False
            else:
                calibration=True
            w.destroy()
            
        w = Tk()
        a=int
        w.title("Calibrazione")
        w.geometry('775x450')
        w.columnconfigure(0, weight=1)
        w.columnconfigure(1, weight=3)
        w.configure(background="#254117")
        w.resizable=(False,False)

        label=Label(text="Inserisci il tipo di calibraizione",background="#254117",fg="white",font=("",16))
        label.grid(row=0,column=0,padx=5,pady=20,columnspan=1)
        
        options = ["nessuna calibrazione","calibrazione a n punti"]
        tkvarq = StringVar(w)
        tkvarq.set(options[0])
        question_menu = OptionMenu(w, tkvarq, *options)
        question_menu.grid(row=1,column=0,padx=10,pady=40,sticky="N")

        submit_button = Button(w, text='ok', command=lambda:_calib, pady=0,fg="white",bg="#254117",width=40,height=2)
        submit_button.grid(row=2,column=0,padx=20,pady=30)

        img = Image.open("/home/penna/ros_ws/src/Sawyer/intera_sdk/intera_examples/scripts/cad.jpg")
        img = img.resize((350, 275), Image.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        panel = Label(w, image = img)
        panel.grid(row=1,column=1,rowspan=2,padx=10,pady=10)

        info_txt="""NB: nel caso in cui si proceda senza eseguire la calibrazione\nnon sarà possibile regolare automaticamente l'orientamento dell'end-effector"""
        info=Label(text=info_txt,background="#254117",fg="white")
        info.grid(row=3,column=0,padx=5,pady=20,columnspan=4)

        w.mainloop()

    #metodo che consente i definire i pricipali parametri del workspace e della calibrazione
    def param_calib(self):
        def _param_calib():
            global H,L,n
            H=h_entry.get()
            L=l_entry.get()
            n=n_punti.get()
            w.destroy()
        w = Tk()
        w.title("parametri calibrazione")
        w.geometry('650x700')
        w.columnconfigure(0, weight=1)
        w.columnconfigure(1, weight=1)
        w.columnconfigure(1, weight=4)
        w.configure(background="#254117")
        w.resizable=(False,False)

        label=Label(text="Parametri di ws e di calibrazione:",background="#254117",fg="white")
        label.grid(row=0,column=1,padx=5,pady=20,columnspan=3)

        info="""NB: inserendo i parametri correttamente\nil path verrà calcolato automaticamente\n\n
            I primi 3 punti da registrare sono necessariamente:\nl'origine ━━➤ un punto sull'asse x ━━➤ un punto sull'asse y\n\n
            I punti aggiuntivi verranno utilizzati per approssimare il piano con maggiore accuratezza\n\n
            Per acquisire una Pose premere il pulsante OK/wheel\n
            Per terminare l'acquisizione premere il pulsante Rethink"""
        label=Label(text=info,background="#254117",fg="white")
        label.grid(row=9,column=1,padx=5,pady=10,columnspan=3)


        label_h=Label(text="Altezza ws [m]:",background="#254117",fg="white")
        label_h.grid(row=2,column=1,padx=5,pady=20,sticky="W")
        h_entry = Entry(w)
        h_entry.insert(0,str(H))
        h_entry.grid(row=2,column=2,padx=5,pady=10)

        label_l=Label(text="Base ws [m]:",background="#254117",fg="white")
        label_l.grid(row=3,column=1,padx=5,pady=10,sticky="W")
        l_entry = Entry(w)
        l_entry.insert(0,str(L))
        l_entry.grid(row=3,column=2,padx=5,pady=10)

        label_n=Label(text="numero punti:",background="#254117",fg="white")
        label_n.grid(row=4,column=1,padx=5,pady=10,sticky="W")
        options=[3,4,5,6,7,8,9,10]
        n_punti = StringVar(w)
        n_punti.set(options[0])
        question_menu = OptionMenu(w, n_punti, *options)
        question_menu.grid(row=4,column=2,padx=5,pady=10,sticky="W")

        submit_button = Button(w, text='ok', command=lambda:_param_calib,width=40,fg="white",bg="#254117")
        submit_button.grid(row=7,column=1,padx=5,pady=30,columnspan=3)

        img = Image.open("/home/penna/ros_ws/src/Sawyer/intera_sdk/intera_examples/scripts/ws.jpg")
        img = img.resize((300, 250), Image.ANTIALIAS)
        img = ImageTk.PhotoImage(img)
        panel = Label(w, image = img)
        panel.grid(row=2,column=3,rowspan=4,padx=10,pady=10)

        w.mainloop()

    #inrfaccia che consente di registrare i dati Pose
    def reg_position(self):
        global poses
        p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
        def _reg_position():
            p_offs=Point(p.position.x+float(offs_x.get()),p.position.y+float(offs_y.get()),p.position.z+float(offs_z.get()))
            poses.append(Pose(p_offs,self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())).orientation)
            w.destroy()
        
        w = Tk()
        w.title("Inserimento offset")
        w.geometry('400x225')
        w.configure(background="#254117")
        w.resizable=(False,False)
        #x
        l=Label(w,text="x= "+str(p.position.x)+" +",background="#254117",fg="white")
        l.grid(row=1,column=0,padx=5,pady=10,sticky="E")
        offs_x = Entry(w)
        offs_x.grid(row=1,column=1,padx=5,pady=10,sticky="W")
        #y
        l=Label(w,text="y= "+str(p.position.y)+" +",background="#254117",fg="white")
        l.grid(row=2,column=0,padx=5,pady=10,sticky="E")
        offs_y = Entry(w)
        offs_y.grid(row=2,column=1,padx=5,pady=10,sticky="W")
        #z
        l=Label(w,text="z= "+str(p.position.z)+" +",background="#254117",fg="white")
        l.grid(row=3,column=0,padx=5,pady=10,sticky="E")
        offs_z = Entry(w)
        offs_z.grid(row=3,column=1,padx=5,pady=10,sticky="W")

        ok_button = Button(w, text='acquisisci posizione', command=lambda:_reg_position(),width=30,fg="white",bg="#254117")
        ok_button.grid(row=4,column=0,padx=0,pady=20,sticky="N",columnspan=2)

        w.mainloop()

    #interaffcia che consente di interagire con il robot
    def controller(self):
        def rpy_set(self):
            global relative_orientation
            rpy=[roll.get(),pitch.get(),yaw.get()]
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.orientation=self.get_quaternion_from_euler(rpy)
            relative_orientation=p.orientation
            self.mover.move_to_position(p)
        def x_plus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.x=p.position.x+STEP_CONTROLLER
            self.mover.move_to_position(p)
        def y_plus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.y=p.position.y+STEP_CONTROLLER
            self.mover.move_to_position(p)
        def z_plus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.z=p.position.z+STEP_CONTROLLER
            self.mover.move_to_position(p)
        def x_minus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.x=p.position.x+STEP_CONTROLLER
            self.mover.move_to_position(p)
        def y_minus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.y=p.position.y+STEP_CONTROLLER
            self.mover.move_to_position(p)
        def z_minus(self):
            p=self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles())
            p.position.z=p.position.z+STEP_CONTROLLER
            self.mover.move_to_position(p)

        w = Tk()
        w.title("Controller")
        w.geometry('600x400')
        w.configure(background="#254117")
        w.resizable=(False,False)
        #position
        l=Label(text="Position [in metri]",background="#254117",fg="white")
        l.grid(row=0,column=0,padx=5,pady=30,columnspan=3)

        l=Label(text="x",background="#254117",fg="white")
        l.grid(row=1,column=0,padx=5,pady=10)
        x_p = Button(w, text='+', command=lambda:x_plus(self),width=10,fg="white",bg="#254117")
        x_p.grid(row=1,column=1,padx=5,pady=10,sticky="W")
        x_m = Button(w, text='-', command=lambda:x_minus(self),width=10,fg="white",bg="#254117")
        x_m.grid(row=1,column=2,padx=5,pady=10,sticky="W")
        l=Label(text=str(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).position.x),background="#254117",fg="white")
        l.grid(row=1,column=3,padx=5,pady=10)

        l=Label(text="y",background="#254117",fg="white")
        l.grid(row=2,column=0,padx=5,pady=10)
        y_p = Button(w, text='+', command=lambda:y_plus(self),width=10,fg="white",bg="#254117")
        y_p.grid(row=2,column=1,padx=5,pady=10,sticky="W")
        y_m = Button(w, text='-', command=lambda:y_minus(self),width=10,fg="white",bg="#254117")
        y_m.grid(row=2,column=2,padx=5,pady=10,sticky="W")
        l=Label(text=str(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).position.y),background="#254117",fg="white")
        l.grid(row=2,column=3,padx=5,pady=10)

        l=Label(text="z",background="#254117",fg="white")
        l.grid(row=3,column=0,padx=5,pady=10)
        z_p = Button(w, text='+', command=lambda:z_plus(self),width=10,fg="white",bg="#254117")
        z_p.grid(row=3,column=1,padx=5,pady=10,sticky="W")
        z_m = Button(w, text='-', command=lambda:z_minus(self),width=10,fg="white",bg="#254117")
        z_m.grid(row=3,column=2,padx=5,pady=10,sticky="W")
        l=Label(text=str(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).position.z),background="#254117",fg="white")
        l.grid(row=3,column=3,padx=5,pady=10)

        #orientation
        l=Label(text="Orientation  [in gradi]",background="#254117",fg="white")
        l.grid(row=0,column=4,padx=5,pady=30,columnspan=2)

        l=Label(text="r:",background="#254117",fg="white")
        l.grid(row=1,column=4,padx=5,pady=10)
        roll = Entry(w)
        roll.insert(0,(self.util.euler_from_quaternion(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).orientation)[0]))
        roll.grid(row=1,column=5,padx=5,pady=10,sticky="W")

        l=Label(text="p:",background="#254117",fg="white")
        l.grid(row=2,column=4,padx=5,pady=10)
        pitch = Entry(w)
        pitch.insert(0,(self.util.euler_from_quaternion(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).orientation)[1]))
        pitch.grid(row=2,column=5,padx=5,pady=10,sticky="W")

        l=Label(text="y:",background="#254117",fg="white")
        l.grid(row=3,column=4,padx=5,pady=10)
        yaw = Entry(w)
        yaw.insert(0,(self.util.euler_from_quaternion(self._limb.joint_angles_to_cartesian_pose(self._limb.joint_angles()).orientation)[2]))
        yaw.grid(row=3,column=5,padx=5,pady=10,sticky="W")

        ok_button = Button(w, text='set orientazione  [r,p,y]', command=lambda:rpy_set(self),width=20,fg="white",bg="#254117")
        ok_button.grid(row=6,column=4,padx=70,pady=25,sticky="E",columnspan=2)

        p_button = Button(w, text='acquisisci posizione  [x,y,z]', command=lambda:self.reg_position(),width=25,fg="white",bg="#254117")
        p_button.grid(row=6,column=0,padx=0,pady=25,sticky="E",columnspan=4)

        stop_button = Button(w, text='stop', command=lambda:w.destroy(),width=60,fg="white",bg="#254117")
        stop_button.grid(row=8,column=0,padx=0,pady=40,sticky="N",columnspan=7)

        w.mainloop()
        
    #metodo che consente di definire i parametri del percorso di prova
    def param_traiettoria_try(self):
        global esegui_prova
        esegui_prova=True

        def _param_traiettoria1():
            global V_TRY_POSE,A_TRY_POSE,V_TRY_GRASP,A_TRY_GRASP,esegui_prova
            V_TRY_GRASP=v_try_grasp.get()
            A_TRY_GRASP=a_try_grasp.get()
            V_TRY_POSE=v_try.get()
            A_TRY_POSE=a_try.get()
            esegui_prova=True
            w.destroy()

        def _param_traiettoria2():
            global esegui_prova
            esegui_prova=False
            w.destroy()

        w = Tk()
        w.title("info")
        w.geometry('400x300')
        w.configure(background="#254117")
        w.columnconfigure(0, weight=1)
        w.columnconfigure(1, weight=1)
        w.resizable=(False,False)

        label_l=Label(text="parametri della traiettoria di prova",background="#254117",fg="white",font=("",16))
        label_l.grid(row=0,column=0,padx=5,pady=10,columnspan=2)

        label_v_try_grasp=Label(text="Velocità grasp",background="#254117",fg="white")
        label_v_try_grasp.grid(row=2,column=0,padx=5,pady=10,sticky="E")
        v_try_grasp = Entry(w)
        v_try_grasp.insert(0,str(V_TRY_GRASP))
        v_try_grasp.grid(row=2,column=1,padx=5,pady=10,sticky="W")

        label_a_try_grasp=Label(text="Accelerazione grasp",background="#254117",fg="white")
        label_a_try_grasp.grid(row=3,column=0,padx=5,pady=10,sticky="E")
        a_try_grasp = Entry(w)
        a_try_grasp.insert(0,str(A_TRY_GRASP))
        a_try_grasp.grid(row=3,column=1,padx=5,pady=10,sticky="W")

        label_v_try=Label(text="Velocità Pose",background="#254117",fg="white")
        label_v_try.grid(row=4,column=0,padx=5,pady=10,sticky="E")
        v_try = Entry(w)
        v_try.insert(0,str(V_TRY_POSE))
        v_try.grid(row=4,column=1,padx=5,pady=10,sticky="W")

        label_a_try=Label(text="Accelerazione Pose",background="#254117",fg="white")
        label_a_try.grid(row=5,column=0,padx=5,pady=10,sticky="E")
        a_try = Entry(w)
        a_try.insert(0,str(A_TRY_POSE))
        a_try.grid(row=5,column=1,padx=5,pady=10,sticky="W")

        submit_button1 = Button(w, text='esegui prova', command=lambda:_param_traiettoria1,width=15,fg="white",bg="#254117")
        submit_button1.grid(row=7,column=0,padx=5,pady=30)

        submit_button2 = Button(w, text='non eseguire prova', command=lambda:_param_traiettoria2,width=15,fg="white",bg="#254117")
        submit_button2.grid(row=7,column=1,padx=5,pady=30)

        w.mainloop()

    #metodo che consente di definire i parametri del percorso di misurazione
    def param_traiettoria(self):

        def _param_traiettoria():
            global V_POSE,A_POSE,V_GRASP,A_GRASP
            V_GRASP=v_grasp.get()
            A_GRASP=a_grasp.get()
            V_POSE=v.get()
            A_POSE=a.get()
            w.destroy()

        w = Tk()
        w.title("info")
        w.geometry('400x300')
        w.configure(background="#254117")
        w.columnconfigure(0, weight=1)
        w.columnconfigure(1, weight=1)
        w.resizable=(False,False)

        label_l=Label(text="parametri della traiettoria di misurazione",background="#254117",fg="white",font=("",16))
        label_l.grid(row=0,column=0,padx=5,pady=10,columnspan=2)

        label_v_grasp=Label(text="Velocità grasp",background="#254117",fg="white")
        label_v_grasp.grid(row=2,column=0,padx=5,pady=10,sticky="E")
        v_grasp = Entry(w)
        v_grasp.insert(0,str(V_GRASP))
        v_grasp.grid(row=2,column=1,padx=5,pady=10,sticky="W")

        label_a_grasp=Label(text="Accelerazione grasp",background="#254117",fg="white")
        label_a_grasp.grid(row=3,column=0,padx=5,pady=10,sticky="E")
        a_grasp = Entry(w)
        a_grasp.insert(0,str(A_GRASP))
        a_grasp.grid(row=3,column=1,padx=5,pady=10,sticky="W")

        label_v=Label(text="Velocità Pose",background="#254117",fg="white")
        label_v.grid(row=4,column=0,padx=5,pady=10,sticky="E")
        v = Entry(w)
        v.insert(0,str(V_POSE))
        v.grid(row=4,column=1,padx=5,pady=10,sticky="W")

        label_a=Label(text="Accelerazione Pose",background="#254117",fg="white")
        label_a.grid(row=5,column=0,padx=5,pady=10,sticky="E")
        a = Entry(w)
        a.insert(0,str(A_POSE))
        a.grid(row=5,column=1,padx=5,pady=10,sticky="W")

        submit_button1 = Button(w, text='esegui prova', command=lambda:_param_traiettoria,width=30,fg="white",bg="#254117")
        submit_button1.grid(row=7,column=0,padx=5,pady=30,columnspan=2)

        w.mainloop()



##############################################################################################################################################
#Classe Util
##############################################################################################################################################
'''descrizione classe Util: 
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

    #metodo che permette di convertire gli angoli di eulero in quaternioni
    def euler_from_quaternion(q):
        x=q.x
        y=q.y
        z=q.z
        w=q.w

        t0 =+2.0*(w*x+y*z)
        t1 =+1.0-2.0*(x*x+y*y)
        roll_x=math.atan2(t0,t1)
        t2=+2.0*(w*y-z*x)
        t2=+1.0 if t2>+1.0 else t2
        t2=-1.0 if t2<-1.0 else t2
        pitch_y = math.asin(t2)
        t3=+2.0*(w*z+x*y)
        t4=+1.0-2.0*(y*y+z*z)
        yaw_z=math.atan2(t3,t4)

        rpy=[roll_x*180/math.pi,pitch_y*180/math.pi,yaw_z*180/math.pi]
        return rpy

    #metodo che permette di convertire i quaternioni in angoli di aulero
    def get_quaternion_from_euler(rpy):
        roll=rpy[0]/180*math.pi
        pitch=rpy[1]/180*math.pi
        yaw=rpy[2]/180*math.pi

        qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)-np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
        qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
        qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)-np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
        qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
        
        return Quaternion(qx,qy,qz,qw)



##############################################################################################################################################
#CLASSE Mover
##############################################################################################################################################

class Cartesian_mover():
    """ create a cartesian mover thread and move cobot to position=[x,y,z] coordinats in meters. When cobot is in position it calls the given callback function  """
    def __init__(self,speed,accel):
        self.linear_speed=speed
        self.linear_accel=accel
        self.util=Util()
        self.rotational_speed = 1.57
        self.rotational_accel = 1.57
        self.timeout=None
        self.tip_name = 'right_hand'
        
    def run(self):
        self.Move_to_position()
        self.callback(self)

    def Move_to_position(self,p,m):
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

            pose=Pose()
            pose.position.x = p.position.x*m[0][0]+p.position.y*m[0][1]+p.position.z*m[0][2]+m[0][3]
            pose.position.y = p.position.x*m[1][0]+p.position.y*m[1][1]+p.position.z*m[1][2]+m[1][3]
            pose.position.z = p.position.x*m[2][0]+p.position.y*m[2][1]+p.position.z*m[2][2]+m[2][3]

            poseStamped = PoseStamped()
            poseStamped.pose.position = pose.position
            poseStamped.pose.orientation = p.orientation
            
            joint_angles = limb.joint_ordered_angles()
            waypoint.set_cartesian_pose(poseStamped, self.tip_name, joint_angles)

            traj.append_waypoint(waypoint.to_msg())

            result = traj.send_trajectory(timeout=self.timeout)

            if result is None:
                rospy.logerr('Trajectory FAILED to send')
                return
            
            if result.result:
                rospy.loginfo('')
            else:
                rospy.logerr('Triettoria non completata %s',result.errorId)
                input('Premi invio per continuare la traiettoria saltanto il prossimo target...')
        except rospy.ROSInterruptException:
            rospy.logerr('')
        
    def path(self,p,p_grasp,m,v1,a1,v2,a2,n):
        movimento_lento=Cartesian_mover(v1,a1)
        movimento_veloce=Cartesian_mover(v2,a2)
        for i in range(n):
            print('\nciclo numero: '+string(i)+' avviato...\n')
            movimento_veloce.Move_to_position(p_grasp[0],m)
            movimento_lento.Move_to_position(p[0],m)
            self.util.wait_key()
            movimento_veloce.Move_to_position(p_grasp[0],m)
            movimento_veloce.Move_to_position(p_grasp[1],m)
            movimento_lento.Move_to_position(p[1],m)
            self.util.wait_key()
            movimento_veloce.Move_to_position(p_grasp[1],m)
            movimento_veloce.Move_to_position(p_grasp[2],m)
            movimento_lento.Move_to_position(p[2],m)
            self.util.wait_key()
            movimento_veloce.Move_to_position(p_grasp[2],m)
            movimento_veloce.Move_to_position(p_grasp[3],m)
            movimento_lento.Move_to_position(p[3],m)
            self.util.wait_key()
            movimento_veloce.Move_to_position(p_grasp[3],m)
            movimento_veloce.Move_to_position(p_grasp[4],m)
            movimento_lento.Move_to_position(p[4],m)
            self.util.wait_key()
            movimento_veloce.Move_to_position(p_grasp[4],m)
            print('\nciclo numero: '+string(i)+' terminato correttamente\n')
        print('\n\npath terminato\n')
        

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

            mat=[[float(i[0]),float(j[0]),float(k[0]),float(origin[0])],
                 [float(i[1]),float(j[1]),float(k[1]),float(origin[1])],
                 [float(i[2]),float(j[2]),float(k[2]),float(origin[2])],
                 [float(0)   ,float(0)   ,float(0)   ,float(1)]       ]

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
        x=np.linspace(X_LIM[0],X_LIM[1],10)
        y=np.linspace(Y_LIM[0],Y_LIM[1],10)
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
    '''
    ┌           ┐
    |i1 j1 k1 Ox|
    |i2 j2 k2 Oy|
    |i3 j3 k3 Oz|
    |0  0  0  1 |
    └           ┘
    '''
    def plot_frame(self,m):

        ax=self.ax
        arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0)
        arrow_prop_dict_t = dict(mutation_scale=20, arrowstyle='-', shrinkA=0, shrinkB=0)

        xr = Arrow3D([0, 1*DIM_AX], [0, 0], [0, 0], **arrow_prop_dict, color='r')
        ax.add_artist(xr)
        yr = Arrow3D([0, 0], [0, 1*DIM_AX], [0, 0], **arrow_prop_dict, color='b')
        ax.add_artist(yr)
        zr = Arrow3D([0, 0], [0, 0], [0, 1*DIM_AX], **arrow_prop_dict, color='g')
        ax.add_artist(zr)

        x_ws = Arrow3D([m[0][3], m[0][3]+(m[0][0]*DIM_AX)], [m[1][3], m[1][3]+(m[1][0]*DIM_AX)], [m[2][3], m[2][3]+(m[2][0]*DIM_AX)], **arrow_prop_dict, color='r')
        ax.add_artist(x_ws)
        y_ws = Arrow3D([m[0][3], m[0][3]+(m[0][1]*DIM_AX)], [m[1][3], m[1][3]+(m[1][1]*DIM_AX)], [m[2][3], m[2][3]+(m[2][1]*DIM_AX)], **arrow_prop_dict, color='b')
        ax.add_artist(y_ws)
        z_ws = Arrow3D([m[0][3], m[0][3]+(m[0][2]*DIM_AX)], [m[1][3], m[1][3]+(m[1][2]*DIM_AX)], [m[2][3], m[2][3]+(m[2][2]*DIM_AX)], **arrow_prop_dict, color='g')
        ax.add_artist(z_ws)

        t0 = Arrow3D([0, m[0][3]*(0.9)], [0, m[1][3]*(0.9)], [0, m[2][3]*(0.9)], **arrow_prop_dict_t, color='y')
        ax.add_artist(t0)
        t1 = Arrow3D([m[0][3]*(0.9),m[0][3]], [m[1][3]*(0.9),m[1][3]], [m[2][3]*(0.9),m[2][3]], **arrow_prop_dict, color='y')
        ax.add_artist(t1)

        ax.text(-0.1*DIM_AX, 0.0, -0.1*DIM_AX, r'$o_R$',color='y')
        ax.text(1.1*DIM_AX, 0, 0, r'$x_R$',color='r')
        ax.text(0, 1.1*DIM_AX+0.04, 0, r'$y_R$',color='b')
        ax.text(0, 0, 1.1*DIM_AX, r'$z_R$',color='g')

        ax.text(m[0][3]+m[0][0]*1.1*DIM_AX, m[1][3]+m[1][0]*1.1*DIM_AX, m[2][3]+m[2][0]*1.1*DIM_AX, r'$x(WS)$',color='r')
        ax.text(m[0][3]+m[0][1]*1.1*DIM_AX, m[1][3]+m[1][1]*1.1*DIM_AX, m[2][3]+m[2][1]*1.1*DIM_AX, r'$y(WS)$',color='b')
        ax.text(m[0][3]+m[0][2]*1.1*DIM_AX, m[1][3]+m[1][2]*1.1*DIM_AX, m[2][3]+m[2][2]*1.1*DIM_AX, r'$z(WS)$',color='g')

        ax.view_init(azim=20, elev=10)
        ax.set_xlim(X_LIM)
        ax.set_ylim(Y_LIM)
        ax.set_zlim(Z_LIM)


##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():
    interface=Interface()
    calib=Calib()
    util=Util()
    interface.controller()
    interface.calibr()
    pose_ws=list()
    pose_ws_grasp=list()
    if calibration==True:
        interface.param_calib()
        m=calib.get_matrix(poses)
        position_ws=list()
        position_ws.append([0,0,0])
        position_ws.append([L-2*A,0,0])
        position_ws.append([L-2*A,H-2*B,0])
        position_ws.append([0,H-2*B,0])
        position_ws.append([L/2-A,H/2-B,0])

        for i in range(len(position_ws)):
            pose_ws.append(Pose(Point(position_ws[i][0]+OFFS_X,position_ws[i][1]+OFFS_Y,position_ws[i][2]+OFFS_Z),poses[0].orientation))
            pose_ws_grasp.append(Pose(Point(position_ws[i][0]+OFFS_X+GRASP_X,position_ws[i][1]+OFFS_Y+GRASP_Y,position_ws[i][2]+OFFS_Z+GRASP_Z),poses[0].orientation))
    else:
        m=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i in range(len(poses)):
            pose_ws.append(Pose(Point(poses[i].position.x+OFFS_X,poses[i].position.y+OFFS_Y,poses[i].position.z+OFFS_Z),poses[i].orientation))
            pose_ws_grasp.append(Pose(Point(poses[i].position.x+OFFS_X+GRASP_X,poses[i].position.y+OFFS_Y+GRASP_Y,poses[i].position.z+OFFS_Z+GRASP_Z),poses[i].orientation))
        
    #esecuzione delle triettorie
    mover=Cartesian_mover
    interface.param_traiettoria_try()
    mover.path(pose_ws,pose_ws_grasp,m,V_TRY_POSE,A_TRY_POSE,V_TRY_GRASP,A_TRY_GRASP,1) #traiettoria di prova a velocità ridotta
    print('path di prova terminato\n')
    interface.param_traiettoria()
    mover.path(pose_ws,pose_ws_grasp,m,V_POSE,A_POSE,V_GRASP,A_GRASP,N_PROVE) #traiettoria della prova
    print('path di misurazione terminato\n')
    input('premere il tasto INVIO per terminare il programma...')
    
    
if __name__ == '__main__':
    main()

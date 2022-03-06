import openpyxl
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

n_marker=0

color_m='red'
color_0p='lightsteelblue'
color_3p='cornflowerblue'
color_5p='navy'

volume_points=20
volume_point_marker=70

def data_extraction(plane):
    
    p=[[],[],[],[],[],[]]

    plane_ex = openpyxl.load_workbook('file\\'+str(plane)+'.xlsx')
    plane_0p=plane_ex["0p"]
    plane_3p=plane_ex["3p"]
    plane_5p=plane_ex["5p"]

    for i in [n for n in range(33) if n>=3]:

        p[0].append([[plane_0p["b"+str(i)].value,plane_0p["c"+str(i)].value,plane_0p["d"+str(i)].value],
                    [plane_3p["b"+str(i)].value,plane_3p["c"+str(i)].value,plane_3p["d"+str(i)].value],
                    [plane_5p["b"+str(i)].value,plane_5p["c"+str(i)].value,plane_5p["d"+str(i)].value]])
        
        p[1].append([[plane_0p["e"+str(i)].value,plane_0p["f"+str(i)].value,plane_0p["g"+str(i)].value],
                    [plane_3p["e"+str(i)].value,plane_3p["f"+str(i)].value,plane_3p["g"+str(i)].value],
                    [plane_5p["e"+str(i)].value,plane_5p["f"+str(i)].value,plane_5p["g"+str(i)].value]])

        p[2].append([[plane_0p["h"+str(i)].value,plane_0p["i"+str(i)].value,plane_0p["j"+str(i)].value],
                    [plane_3p["h"+str(i)].value,plane_3p["i"+str(i)].value,plane_3p["j"+str(i)].value],
                    [plane_5p["h"+str(i)].value,plane_5p["i"+str(i)].value,plane_5p["j"+str(i)].value]])

        p[3].append([[plane_0p["k"+str(i)].value,plane_0p["l"+str(i)].value,plane_0p["m"+str(i)].value],
                    [plane_3p["k"+str(i)].value,plane_3p["l"+str(i)].value,plane_3p["m"+str(i)].value],
                    [plane_5p["k"+str(i)].value,plane_5p["l"+str(i)].value,plane_5p["m"+str(i)].value]])

        p[4].append([[plane_0p["n"+str(i)].value,plane_0p["o"+str(i)].value,plane_0p["p"+str(i)].value],
                    [plane_3p["n"+str(i)].value,plane_3p["o"+str(i)].value,plane_3p["p"+str(i)].value],
                    [plane_5p["n"+str(i)].value,plane_5p["o"+str(i)].value,plane_5p["p"+str(i)].value]])


    plane_ex.close()

    return p

def calc_axes(p,mode):
    axes_min=0
    axes_max=0
    min_rel=[0,0,0]
    max_rel=[0,0,0]
    axes=[[],[],[]]
    m=[0,0,0]
    for i in range(len(p)):
        for k in range(3):
            if min_rel[k]>min(p[i][0][k],p[i][1][k],p[i][2][k]): min_rel[k]=min(p[i][0][k],p[i][1][k],p[i][2][k])
            if max_rel[k]<max(p[i][0][k],p[i][1][k],p[i][2][k]): max_rel[k]=max(p[i][0][k],p[i][1][k],p[i][2][k])
        
    if min(min_rel)<axes_min: axes_min=min(min_rel)
    if max(max_rel)>axes_max: axes_max=max(max_rel)
    axes_dim=(abs(axes_min)+abs(axes_max))/2

    for i in range(len(m)):

        if mode=='rel':
            m[i]=(min_rel[i]+max_rel[i])/2
        else:
            m[i]=0
        axes[i]=[m[i]-axes_dim,m[i]+axes_dim]
    return [axes[0],axes[1],axes[2]]

def gif_3d(p,plane,mode):
    n_marker=1

    for i in range(len(p)):
        f3d=plt.figure(figsize=(8,8))
        ax_3d = f3d.add_subplot(1, 1, 1, projection='3d', adjustable='box')
        
        [axes_x,axes_y,axes_z]=calc_axes(p[i],mode)

        ax_3d.axes.set_xlim3d(left=axes_x[0], right=axes_x[1]) 
        ax_3d.axes.set_ylim3d(bottom=axes_y[0], top=axes_y[1]) 
        ax_3d.axes.set_zlim3d(bottom=axes_z[0], top=axes_z[1])
        
        ax_3d.scatter(0, 0, 0, color=color_m, s=volume_point_marker, label='Posizione del marker')
        ax_3d.scatter(p[i][0][0][0],p[i][0][0][1],p[i][0][0][2], color=color_0p, s=volume_points, label='Nessuna calibrazione')
        ax_3d.scatter(p[i][0][1][0],p[i][0][1][1],p[i][0][1][2], color=color_3p, s=volume_points, label='Calibrazione 3 punti')
        ax_3d.scatter(p[i][0][2][0],p[i][0][2][1],p[i][0][2][2], color=color_5p, s=volume_points, label='Calibrazione 5 punti')
        
        for k in range(len(p[i])-1):
            ax_3d.scatter(p[i][k+1][0][0],p[i][k+1][0][1],p[i][k+1][0][2], color=color_0p, s=volume_points)
            ax_3d.scatter(p[i][k+1][1][0],p[i][k+1][1][1],p[i][k+1][1][2], color=color_3p, s=volume_points)
            ax_3d.scatter(p[i][k+1][2][0],p[i][k+1][2][1],p[i][k+1][2][2], color=color_5p, s=volume_points)
 
        ax_3d.set_xlabel('x [mm]',fontsize=16)
        ax_3d.set_ylabel('y [mm]',fontsize=16)
        ax_3d.set_zlabel('z [mm]',fontsize=16)
            
        plt.show()

        def rotate(angle):
            ax_3d.view_init(azim=angle)

        angle = 3
        ani = animation.FuncAnimation(f3d, rotate, frames=np.arange(0, 360, angle), interval=20)
        print('making P'+str(n_marker)+' of plane '+str(plane)+' mode:'+str(mode)+'...')
        ani.save('plot\\'+str(plane)+'\\gif_3d\\'+str(n_marker)+'_'+str(mode)+'.gif', writer=animation.PillowWriter(fps=15))
        n_marker=n_marker+1
        print('done')
        plt.axis('on')
        ax_3d.legend()
        
        

##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():
    for plane in ['xy','xz','yz']:
        p=data_extraction(plane)
        gif_3d(p,plane,'rel')
        gif_3d(p,plane,'abs')

    
    
if __name__ == '__main__':
    main()
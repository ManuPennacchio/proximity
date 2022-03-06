import openpyxl
import matplotlib.pyplot as plt

color_m='red'
color_0p='skyblue'
color_3p='royalblue'
color_5p='navy'


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


def calc_axes(p):

    min_rel=[0,0,0]
    max_rel=[0,0,0]
    axes=[[],[],[]]
    m=[0,0,0]
    for i in range(len(p)):
        for k in range(3):
            if min_rel[k]>min(p[i][0][k],p[i][1][k],p[i][2][k]): min_rel[k]=min(p[i][0][k],p[i][1][k],p[i][2][k])
            if max_rel[k]<max(p[i][0][k],p[i][1][k],p[i][2][k]): max_rel[k]=max(p[i][0][k],p[i][1][k],p[i][2][k])

    axes_dim=(abs(min(min_rel))+abs(max(max_rel)))/2
    for i in range(len(m)):
        m[i]=(min_rel[i]+max_rel[i])/2
        axes[i]=[(m[i]-axes_dim)*1.05,(m[i]+axes_dim)*1.05]
    return [axes[0],axes[1],axes[2]]


def plot_2d(p,plane):
    colors=[color_0p,color_3p,color_5p]
    vertical_label=['x [mm]','y [mm]','z [mm]']

    for i in range(len(p)-1):
        axes=calc_axes(p[i])
        f, ((x_0p,y_0p,z_0p),(x_3p,y_3p,z_3p),(x_5p,y_5p,z_5p))= plt.subplots(3, 3,figsize=(10,4))
        plots=[[x_0p,y_0p,z_0p],[x_3p,y_3p,z_3p],[x_5p,y_5p,z_5p]]
        f.subplots_adjust(hspace=0.8,wspace=0.4)
        
        for h in [0,1,2]:
            for q in[0,1,2]:
                for j in range(len(p[i])):
                    plots[h][q].bar(j+1,p[i][j][h][q],color=colors[q]) 
                    plots[h][q].set_xlabel('N prova',fontsize='12')
                    plots[h][q].set_ylabel(vertical_label[q],fontsize='12')
                    plt.setp(plots[h][q], xlim=(0.5,30.5),ylim=(axes[q][0],axes[q][1]))
                    plots[h][q].axhline(y=0, color='grey', linestyle='-')
        plt.savefig( 'grafici\\'+str(plane)+'\\plot_2d\\'+plane+'_p'+str(i+1)+'.jpg')
        print('done '+str(plane)+'_p'+str(i+1))
        plt.show()

##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():
    
    for plane in ['xy','xz','yz']:
        p=data_extraction(plane)
        plot_2d(p,plane)
    
    
if __name__ == '__main__':
    main()



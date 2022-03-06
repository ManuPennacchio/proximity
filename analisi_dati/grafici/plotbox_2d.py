from cProfile import label
from tkinter import font
import openpyxl
from tkinter import *
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from PIL import ImageTk, Image
import numpy as np
import matplotlib.animation as animation

#definizione delle costanti e delle variabili globali
k=0
COLOR_M='red'
COLOR_0P='skyblue'
COLOR_3P='royalblue'
COLOR_5P='navy'


def data_extraction(plane):
    '''
    funzione che si occupa di estrarre i dati dal foglio excel
    relativo agli errori di posizionamento e salvato nell'apposita directory
    definita nel programma
    '''

    #inizializzazione delle variabili
    p1=[[],[],[]]   #errori di posizionamento del marker P1
    p2=[[],[],[]]   #errori di posizionamento del marker P2
    p3=[[],[],[]]   #errori di posizionamento del marker P3
    p4=[[],[],[]]   #errori di posizionamento del marker P4
    p5=[[],[],[]]   #errori di posizionamento del marker P5

    #apertura del file .xlsx contenente gli errori di posizionamento
    plane_ex = openpyxl.load_workbook('file\\'+str(plane)+'.xlsx') #NB:definizione directory
    
    #apertura dei fogli relativi ai tre tipi di calibrazione
    plane_0p=plane_ex["0p"]
    plane_3p=plane_ex["3p"]
    plane_5p=plane_ex["5p"]

    #estrazione degli errori di posizionamento per ogni tipo di calibrazione e per ogni marker
    for i in [n for n in range(33) if n>=3]:
        p1[0].append([plane_0p["b"+str(i)].value,plane_0p["c"+str(i)].value,plane_0p["d"+str(i)].value])
        p2[0].append([plane_0p["e"+str(i)].value,plane_0p["f"+str(i)].value,plane_0p["g"+str(i)].value])
        p3[0].append([plane_0p["h"+str(i)].value,plane_0p["i"+str(i)].value,plane_0p["j"+str(i)].value])
        p4[0].append([plane_0p["k"+str(i)].value,plane_0p["l"+str(i)].value,plane_0p["m"+str(i)].value])
        p5[0].append([plane_0p["n"+str(i)].value,plane_0p["o"+str(i)].value,plane_0p["p"+str(i)].value])

        p1[1].append([plane_3p["b"+str(i)].value,plane_3p["c"+str(i)].value,plane_3p["d"+str(i)].value])
        p2[1].append([plane_3p["e"+str(i)].value,plane_3p["f"+str(i)].value,plane_3p["g"+str(i)].value])
        p3[1].append([plane_3p["h"+str(i)].value,plane_3p["i"+str(i)].value,plane_3p["j"+str(i)].value])
        p4[1].append([plane_3p["k"+str(i)].value,plane_3p["l"+str(i)].value,plane_3p["m"+str(i)].value])
        p5[1].append([plane_3p["n"+str(i)].value,plane_3p["o"+str(i)].value,plane_3p["p"+str(i)].value])

        p1[2].append([plane_5p["b"+str(i)].value,plane_5p["c"+str(i)].value,plane_5p["d"+str(i)].value])
        p2[2].append([plane_5p["e"+str(i)].value,plane_5p["f"+str(i)].value,plane_5p["g"+str(i)].value])
        p3[2].append([plane_5p["h"+str(i)].value,plane_5p["i"+str(i)].value,plane_5p["j"+str(i)].value])
        p4[2].append([plane_5p["k"+str(i)].value,plane_5p["l"+str(i)].value,plane_5p["m"+str(i)].value])
        p5[2].append([plane_5p["n"+str(i)].value,plane_5p["o"+str(i)].value,plane_5p["p"+str(i)].value])

    #chiusura foglio excel
    plane_ex.close()

    #salvataggio delle coordinate di tutti i punti nel piano
    p_0p=p1[0]+p2[0]+p3[0]+p4[0]+p5[0]  #coordinate con 0 punti di calibrazione
    p_3p=p1[1]+p2[1]+p3[1]+p4[1]+p5[1]  #coordinate con 3 punti di calibrazione
    p_5p=p1[2]+p2[2]+p3[2]+p4[2]+p5[2]  #coordinate con 5 punti di calibrazione

    return([p_0p,p_3p,p_5p])
    
    

#metodo che permette di visualizzare una lista di punti
def plot(p_0p,p_3p,p_5p,plane):
    '''
    funzione che si occupa di creare e salvare i plotbox
    dei dati estratti in precedenza
    '''
    #riordinamento dei dati
    x_0p=list()
    y_0p=list()
    z_0p=list()
    x_3p=list()
    y_3p=list()
    z_3p=list()
    x_5p=list()
    y_5p=list()
    z_5p=list()

    for i in range(len(p_0p)):
        x_0p.append(p_0p[i][0])
        x_3p.append(p_3p[i][0])
        x_5p.append(p_5p[i][0])
        y_0p.append(p_0p[i][1])
        y_3p.append(p_3p[i][1])
        y_5p.append(p_5p[i][1])
        z_0p.append(p_0p[i][2])
        z_3p.append(p_3p[i][2])
        z_5p.append(p_5p[i][2])

    x=x_0p+x_3p+x_5p    #lista che comprende tutti gli errori di posizionamento sull'asse x del relativo piano
    y=y_0p+y_3p+y_5p    #lista che comprende tutti gli errori di posizionamento sull'asse y del relativo piano
    z=z_0p+z_3p+z_5p    #lista che comprende tutti gli errori di posizionamento sull'asse z del relativo piano

    #definizioe degli estremi dell'asse y
    axes_min=min(x+y+z)*1.05
    axes_max=max(x+y+z)*1.05
    #creazione delle figure e dei subplot
    fig_piano, (ax_x, ax_y,ax_z) = plt.subplots(nrows=1, ncols=3, figsize=(10, 4))
    fig_piano.subplots_adjust(wspace=0.6)
    labels=['','','']
    
    bplot_x=ax_x.boxplot([x_0p,x_3p,x_5p], notch=True,vert=True,patch_artist=True,boxprops=dict(facecolor=COLOR_0P),medianprops=dict(color='k'),labels=labels)
    ax_x.axhline(y=0, color='grey', linestyle='-')
    plt.setp(ax_x, ylim=(axes_min,axes_max))
    bplot_y=ax_y.boxplot([y_0p,y_3p,y_5p],notch=True,vert=True,patch_artist=True,boxprops=dict(facecolor=COLOR_3P),medianprops=dict(color='k'),labels=labels)
    ax_y.axhline(y=0, color='grey', linestyle='-')
    plt.setp(ax_y, ylim=(axes_min,axes_max))
    bplot_z=ax_z.boxplot([z_0p,z_3p,z_5p],notch=True,vert=True,patch_artist=True,boxprops=dict(facecolor=COLOR_5P),medianprops=dict(color='k'),labels=labels)
    ax_z.axhline(y=0, color='grey', linestyle='-')
    plt.setp(ax_z, ylim=(axes_min,axes_max))
    #colori
    for bplot in (bplot_x,bplot_y,bplot_z):
        for patch, color in zip(bplot['boxes'], [COLOR_0P,COLOR_3P,COLOR_5P]):
            patch.set_facecolor(color)
    #nominazione assi
    for ax in [ax_x, ax_y,ax_z]:
        ax.yaxis.grid(True)
        if ax==ax_x:    ax.set_xlabel('asse x',fontsize='18')
        if ax==ax_y:    ax.set_xlabel('asse y',fontsize='18')
        if ax==ax_z:    ax.set_xlabel('asse z',fontsize='18')
        ax.set_ylabel('dispersione [mm]',fontsize='14')

    #fig_piano.legend([bplot_x["boxes"][0], bplot_z["boxes"][0], bplot_z["boxes"][0]], ['Nessuna calibrazione', 'Calibrazione a 3 punti','Calibrazione a 5 punti'], loc='upper right')
    
    #salvataggio del plot come .jpg
    global k
    plt.savefig( 'plot\\'+str(plane)+'\\plotbox_'+plane+'.jpg')  #NB:definizione directory di salvataggio
    print('done plotbox_'+str(plane))
    k=k+1
        
##############################################################################################################################################
#MAIN
##############################################################################################################################################
def main():
    for plane in ['xy','xz','yz']:
        [p_0p,p_3p,p_5p]=data_extraction(plane)
        plot(p_0p,p_3p,p_5p,plane)
    print('finish')
    
if __name__ == '__main__':
    main()

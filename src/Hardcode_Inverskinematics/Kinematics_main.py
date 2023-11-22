import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_q(vec_des):
    #Matrix form:
    '''
    0° ; 30°; 60° ;90°...

    0°[
    30°
    60°
    90°
    ]



    '''
    '''
    # def make_vector(q1_deg,q2_deg):
    #     # Define variables
    #     e1 = np.pi / 4
    #     e2 = np.pi / 4
        
    #     #180°-pi
    #     #x- y.px
    #     q1=q1_deg*np.pi/180
    #     q2=q2_deg*np.pi/180
    #     # Define transformation matrices
    #     C_01 = np.array([
    #         [np.cos(q1), 0, np.sin(q1)],
    #         [0, 1, 0],
    #         [-np.sin(q1), 0, np.cos(q1)]
    #     ])

    
    #     C_12 = np.array([
    #         [np.cos(-e1), -np.sin(-e1), 0],
    #         [np.sin(-e1), np.cos(-e1), 0],
    #         [0, 0, 1]
    #     ])

    #     C_23 = np.array([
    #         [np.cos(q2), 0, np.sin(q2)],
    #         [0, 1, 0],
    #         [-np.sin(q2), 0, np.cos(q2)]
    #     ])

    #     C_34 = np.array([
    #         [np.cos(-e2), -np.sin(-e2), 0],
    #         [np.sin(-e2), np.cos(-e2), 0],
    #         [0, 0, 1]
    #     ])

    #     C_0E = np.dot(np.dot(np.dot(C_01, C_12), C_23), C_34)

    #     return C_0E[:,[1]]

    # def get_matrix():
    #     print('Making Matrix')
    #     for a1 in range(0, int(360 / step_size)):
    #         q1 = a1 * step_size
    #         for a2 in range(0, int(360 / step_size)):
    #             q2=a2*step_size
    #             y_vector=make_vector(q1,q2)          
    #             #Stack the vectors
    #             if q2==0: 
    #                 temp_vec=y_vector 
    #             else:
    #                 temp_vec=np.vstack((temp_vec,y_vector))

    #         if q1==0:
    #             val_matrix=temp_vec
    #         else:  
    #             val_matrix=np.column_stack((val_matrix,temp_vec))   
    #     return val_matrix
    '''
    def compare_vectors(vec_vgl,i,j,vec_best):
        #print('Comparing vectors:')
        #Normalize vector: 
        vec_vgl_norm=vec_vgl/np.linalg.norm(vec_vgl)
        
        vec_dif=np.subtract(vec_des,vec_vgl_norm)
        #print(vec_dif)
        norm=np.linalg.norm(vec_dif)#taking the Forbenius norm.
        #print(norm)
        #print(vec_best[5])

        if norm<vec_best[5]:
            vec_best=[vec_vgl_norm[0],vec_vgl_norm[1],vec_vgl_norm[2],i,j,norm]
            #print('New best vector',vec_best)
        return vec_best

    def plot_vectors(v1, v2):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Ursprung des Vektors
        origin = np.zeros(3)

        # Vektoren zeichnen
        ax.quiver(*origin, *v1, color='r', label='Reference')
        ax.quiver(*origin, *v2, color='b', label='Nozzle')

        # Achsenbeschriftung
        ax.set_xlabel('X-Achse')
        ax.set_ylabel('Y-Achse')
        ax.set_zlabel('Z-Achse')

        # Achsengrenzen anpassen
        ax.set_xlim([0, max(max(v1[0], v2[0]), 1)])
        ax.set_ylim([0, max(max(v1[1], v2[1]), 1)])
        ax.set_zlim([0, max(max(v1[2], v2[2]), 1)])

        # Legende hinzufügen
        ax.legend()

        plt.show()


    #[0,1/math.sqrt(2),1/math.sqrt(2)] #0.5->q1:  204.5 q2:  114.5 #1 ->q1:  205 q2:  114.0
    #[0,1,0]
    #[1,1,0]/np.linalg.norm([1,1,0])
    # #[0,1/math.sqrt(2),1/math.sqrt(2)]

    #Start
    #-----------------------------------------------------------------------------------------------------------------------

    resolution  =1                                      #set your desired resolution: - this must match the matrix you load, should be updated automatically
    vec_best=[0,0,0,0,0,1000]                           #initailizing the best vector:  #-> 0-2 xyz 3=q1 4=q2 5=np.linalg.norm(vec_dif)
    counter=0                                           #inizialize counter
    vec_vgl=[0,0,0]                                     #inizialize comparisonvector , is a dynamic one will get changed every time 

    #set your desired vector:
    #vec_des=[0,1,0]
    print(vec_des)

    matrix = np.load(f'Matrix_{resolution}_rad.npy')    #load tghe matrix


    for j in range(int(360/resolution)):                #iterate over colums
        for i in range(int(360/resolution*3)):          #the matrix has dimension (N*3)xN
            if counter!=2: 
                vec_vgl[counter]=matrix[i,j]
                counter+=1
            else:
                vec_vgl[counter]=matrix[i,j]
                counter=0
                vec_best=compare_vectors(vec_vgl, i,j, vec_best)
                vec_vgl=[0,0,0]

    print(vec_best)

    #retransforn i, j to degrees
    q2_final=(((vec_best[3]+1)/3)-1)*(resolution)
    q1_final=(vec_best[4])*resolution 
    
    print('q1: ', q1_final, 'q2: ', q2_final)

    #Plottig:
    vector1 = [vec_des[0], vec_des[2], vec_des[1]]
    vector2 = [vec_best[0],vec_best[2],vec_best[1]]
    #plot_vectors(vector1, vector2)
    return q1_final, q2_final

vec_des=[0,1,0]

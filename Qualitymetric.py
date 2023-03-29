from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import math
from numpy import linalg as LA
origine = [0,0,0,1]
vecteurmondex = [1, 0, 0,0]
vecteurmondey = [0, 1, 0,0]
vecteurmondez = [0, 0, 1,0]
p0 = [1, 0, 0]
p1 = [0, 1, 0]
p2 = [0, 0, 1]

# Matrice de rotation 3D autour de l'axe x
def matrice_Rotation_3D_X(teta):
     teta = math.radians(teta)
     return np.array([[1, 0, 0, 0], [0, math.cos(teta), -math.sin(teta), 0], [0, math.sin(teta), math.cos(teta), 0], [0, 0, 0, 1]])
# Matrice de rotation autour axe y
def matrice_rotation_3D_Y(teta):
    teta = math.radians(teta)
    return np.array([[math.cos(teta), 0, math.sin(teta), 0], [0, 1, 0, 0], [-math.sin(teta), 0, math.cos(teta), 0], [0, 0, 0, 1]])
# Matrice de rotation autour axe z
def matrice_rotation_3D_Z(teta):
     teta = math.radians(teta)
     return np.array([[math.cos(teta), -math.sin(teta), 0, 0], [math.sin(teta), math.cos(teta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# Matrice de Translation
def matrice_translation_3D(Tx, Ty, Tz):
    return np.array([[1, 0, 0, Tx], [0, 1, 0, Ty], [0, 0, 1, Tz], [0, 0, 0, 1]])
def point_on_line(p, v,origine):
    # On cherche à résoudre
    # p = q + t*v
    # Coordonnées du point
    x, y, z = p
    # Coordonnées du vecteur
    u, v, w = v
    # Coordonnées de l'origine du vecteur par rapport à la base canonique
    uO,vO,wO = origine
    # Vérification des valeurs nulles
    if u != 0:
        t = (x - uO) / u
    elif v != 0:
        t = (y - vO) / v
    elif w != 0:
        t = (z - wO) / w
    else:
        # Vecteur nul
        return False
    
    # Si le point p appartient à la droite engendrée par le vecteur
    # Si la valeur de t vérifie les 3 équations 
    res = [t*u+uO, t*v+vO, t*w+wO]
    return res == p
Ry = matrice_rotation_3D_Y(20)
Rx = matrice_Rotation_3D_X(-20)
T = matrice_translation_3D(5,5,5)
# Camera 1
# TR = np.dot(T,Ry)
# print("TR:",TR)
# C1 = np.dot(TR,origine)
C1 = np.dot(T,origine)
print(C1)

# Camera 2
PC2 = np.array([6,5,5,1])

monderotationx = np.dot(matrice_rotation_3D_Z(90),vecteurmondex)
monderotationy = np.dot(matrice_rotation_3D_Z(90),vecteurmondey)
monderotationz = np.dot(matrice_rotation_3D_Z(90),vecteurmondez)
# Test normalisation du vecteur 
monderotationx = monderotationx/np.linalg.norm(monderotationx)
monderotationy =monderotationy/np.linalg.norm(monderotationy)
monderotationz = monderotationz/np.linalg.norm(monderotationz)
print("Vecteur rotation ",monderotationx,"y",monderotationy,"z",monderotationz)



fig = plt.figure(figsize=(6, 4))
ax = fig.add_subplot(111,projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
# set limit
ax.set(xlim=(-10, 10), ylim=(-10, 10), zlim=(0, 20))
# For camera 1 
X1,Y1,Z1 = zip(C1[0:3], C1[0:3], C1[0:3])
U1,V1,W1 = zip(p0,p1,p2)
ax.quiver(X1, Y1, Z1, U1, V1, W1, color="blue",
          arrow_length_ratio=0.01 )
ax.plot([X1[0],U1[0]+5],[Y1[1],V1[1]+5],[Z1[2],W1[2]+5],"r-")
# For camera 2 
X, Y, Z = zip(PC2[0:3], PC2[0:3], PC2[0:3])
print("X",X)
U, V, W = zip(monderotationx[0:3],monderotationy[0:3], monderotationz[0:3])
#print("Monde rotation : ",monderotation[0][0:3])
ax.quiver(X, Y, Z, U, V, W, color="blue",
          arrow_length_ratio=0.01 )
ax.plot([X[0],U[0]+5],[Y[1],V[1]+5],[Z[2],W[2]+5],"g-")
plt.show()
####
# Calcul de l'angle entre deux vecteurs caméra 1 et 2 
# Coordoonées du vecteur 1
# 365 cm longueur du vecteur représentant la focale de la caméra

V1 = np.dot(365,p0)+np.dot(365,p1)+np.dot(365,p2)
print(V1)
V2 = np.dot(365,monderotationx[0:3])+np.dot(365,monderotationy[0:3])+np.dot(365,monderotationz[0:3])
print(V2)
# Calcul de l'angle entre ces deux vecteurs
produitscalaire = np.dot(V1,V2)
print(produitscalaire)
normeV1 = LA.norm(V1,2)
normeV2 = LA.norm(V2,2)
cos = produitscalaire/(normeV1*normeV2)
angle = math.degrees(math.acos(cos))
print("angle in degree",angle)
##
# Calcul de l'intersection entre 1 point et 1 vecteur
P1 = [365,365,365]

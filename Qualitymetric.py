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
    print("coefficient V*t",t)
    if t <= 1:
        res = [t*u+uO, t*v+vO, t*w+wO]
        return res == p
    else:
        return False
def angle_between_vectors(V1,V2):
# V1 et V2 coordoonées du vecteur dans leur bases respectives
# Calcul de l'angle entre ces deux vecteurs
    produitscalaire = np.dot(V1,V2)
    print(produitscalaire)
    normeV1 = LA.norm(V1,2)
    normeV2 = LA.norm(V2,2)
    cos = produitscalaire/(normeV1*normeV2)
    angle = math.degrees(math.acos(cos))
    if angle>=40 and angle <= 140:
        print("angle in degree",angle)
        return True
    else:
        print("L'angle ne respecte pas les conditions sur l'angle de convergence")
        return False
    ##
def intersection_between_vectors(V1,V2,origine1,origine2):
    # Coordoonées du vecteur V1
    u1,v1,z1 = V1
    # Coordonnées de l'origine du vecteur 1
    u1_O,v1_O,z1_O = origine1
    # Coordonnées du vecteur V2
    u2,v2,z2 = V2
   # Coordonnées de l'origine du vecteur 2 
    u2_O,v2_O,z2_O = origine2
    # Chaque vecteur peut engendrer une droite représenté sous la forme d'un système paramétrique
    # Ex x = t*u1 + u1_O
    # Avec t un coeff entre 0 et 1
    # Ainsi on détermine si un point est commun avec les deux droites 
    if u1!=u2:
        t = abs((u1_O-u2_O)/(u1-u2))
    elif v1!=v2:
        t = abs((v1_O-v2_O)/(v1-v2))
    elif z1!=z2:
        t = abs((z1_O-z2_O)/(z1-z2))
    else:
        return False
    if t<=1:
        res1 = [t*u1+u1_O,t*v1+v1_O,t*z1+z1_O]
        res2 = [t*u2+u2_O,t*v2+v2_O,t*z2+z2_O]
        if res1 == res2:
            print("Intersection entre les vecteurs",res1)
            return res1
        else:
            print("Pas d'intersection")
            return False

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
# Point test 1
P1 = [371,371,371]

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
ax.scatter(P1[0],P1[1],P1[2],'o')
plt.show()
####
# Calcul de l'angle entre deux vecteurs caméra 1 et 2 
# Coordoonées du vecteur 1
# 365 cm longueur du vecteur représentant la focale de la caméra

V1 = np.dot(365,p0)+np.dot(365,p1)+np.dot(365,p2)
print(V1)
V2 = np.dot(365,monderotationx[0:3])+np.dot(365,monderotationy[0:3])+np.dot(365,monderotationz[0:3])
print(V2)
## Quality metric
# Calcul de l'intersection entre 1 point et 1 vecteur
print("Croisement ? ",point_on_line(P1,V1,[5,5,5]))
# Calcul de l'angle entre deux vecteur
angle_between_vectors(V1,V2)
# Détermination de l'intersection entre deux vecteur
intersection_between_vectors(V1,V2,[5,5,5],[6,5,5])
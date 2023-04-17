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
def matrice_rotation_3D_X(teta):
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
def angle_between_vectors(v1,v2):
# V1 et V2 coordoonées du vecteur dans leur bases respectives
# Calcul de l'angle entre ces deux vecteurs
    produitscalaire = np.dot(v1,v2)
    print(produitscalaire)
    normev1 = LA.norm(v1,2)
    normev2 = LA.norm(v2,2)
    cos = produitscalaire/(normev1*normev2)
    angle = math.degrees(math.acos(cos))
    if angle>=40 and angle <= 140:
        print("angle between vectors in degree",angle)
        return angle
    else:
        print("L'angle ne respecte pas les conditions sur l'angle de convergence")
        return False
    ##
def intersection_between_vectors(v1,v2,origine1,origine2):
    # Coordoonées du vecteur V1
    u1,v1,w1 = v1
    # Coordonnées de l'origine du vecteur 1
    u1_O,v1_O,w1_O = origine1
    # Coordonnées du vecteur V2
    u2,v2,w2 = v2
   # Coordonnées de l'origine du vecteur 2 
    u2_O,v2_O,w2_O = origine2
    # Chaque vecteur peut engendrer une droite représenté sous la forme d'un système paramétrique
    # Ex x = t*u1 + u1_O
    # Avec t un coeff entre 0 et 1
    # Ainsi on détermine si un point est commun avec les deux droites 
    if u1!=u2:
        t = abs((u1_O-u2_O)/(u1-u2))
    elif v1!=v2:
        t = abs((v1_O-v2_O)/(v1-v2))
    elif w1!=w2:
        t = abs((w1_O-w2_O)/(w1-w2))
    else:
        return False
    if t<=1:
        res1 = [t*u1+u1_O,t*v1+v1_O,t*w1+w1_O]
        res2 = [t*u2+u2_O,t*v2+v2_O,t*w2+w2_O]
        if res1 == res2:
            print("Intersection entre les vecteurs",res1)
            return res1
        else:
            print("Pas d'intersection")
            return False
# Fonction permettant de réaliser une transformation géométrique par rapport au référentiel canonique
# Retourne les coordonnées de l'origine de la nouvelle base ainsi que les coordonnées des vecteurs unitaires
# rotation en degrés et translation en array 3D
# axis en str pour déterminer selon quels axes
def define_rotation_translation(angle,translation,axis):
    if axis == "X":
            monderotationx = np.dot(matrice_rotation_3D_X(angle),vecteurmondex)
            monderotationy = np.dot(matrice_rotation_3D_X(angle),vecteurmondey)
            monderotationz = np.dot(matrice_rotation_3D_X(angle),vecteurmondez)
    elif axis == "Y":
            monderotationx = np.dot(matrice_rotation_3D_Y(angle),vecteurmondex)
            monderotationy = np.dot(matrice_rotation_3D_Y(angle),vecteurmondey)
            monderotationz = np.dot(matrice_rotation_3D_Y(angle),vecteurmondez)
    elif axis == "Z":
            monderotationx = np.dot(matrice_rotation_3D_Z(angle),vecteurmondex)
            monderotationy = np.dot(matrice_rotation_3D_Z(angle),vecteurmondey)
            monderotationz = np.dot(matrice_rotation_3D_Z(angle),vecteurmondez)
    else:
        print("Veuillez saisir X, Y ou Z dans axis.")
        # Normalisation des vecteurs
    monderotationx = monderotationx/np.linalg.norm(monderotationx)
    monderotationy =monderotationy/np.linalg.norm(monderotationy)
    monderotationz = monderotationz/np.linalg.norm(monderotationz)
    origine = np.dot(matrice_translation_3D(translation[0],translation[1],translation[2]),[0,0,0,1])
    return np.array([monderotationx,monderotationy,monderotationz,origine])
# Création d'une classe représentant les caméras
class Camera: 
    def __init__(self,monderotationx,monderotationy,monderotationz,origine,name):
        self.origine = origine[0:3]
        self.vector = np.dot(365,monderotationx[0:3])+np.dot(365,monderotationy[0:3])+np.dot(365,monderotationz[0:3])
        self.name = name
# Ry = matrice_rotation_3D_Y(20)
# Rx = matrice_rotation_3D_X(-20)
# T = matrice_translation_3D(5,5,5)
# # Camera 1
# # TR = np.dot(T,Ry)
# # print("TR:",TR)
# # C1 = np.dot(TR,origine)
# C1 = np.dot(T,origine)
# #print(C1)

# # Camera 2
# PC2 = np.array([6,5,5,1])

# Monderotationx = np.dot(matrice_rotation_3D_Z(90),vecteurmondex)
# Monderotationy = np.dot(matrice_rotation_3D_Z(90),vecteurmondey)
# Monderotationz = np.dot(matrice_rotation_3D_Z(90),vecteurmondez)
# # Test normalisation du vecteur 
# Monderotationx = Monderotationx/np.linalg.norm(Monderotationx)
# Monderotationy = Monderotationy/np.linalg.norm(Monderotationy)
# Monderotationz = Monderotationz/np.linalg.norm(Monderotationz)
# #print("Vecteur rotation ",Monderotationx,"y",Monderotationy,"z",Monderotationz)
# # Point test 1
# P1 = [6.5,6.5,6.5]


# ####
# # Calcul de l'angle entre deux vecteurs caméra 1 et 2 
# # Coordoonées du vecteur 1
# # 365 cm longueur du vecteur représentant la focale de la caméra

# V1 = np.dot(365,p0)+np.dot(365,p1)+np.dot(365,p2)
# #print(V1)
# V2 = np.dot(365,Monderotationx[0:3])+np.dot(365,Monderotationy[0:3])+np.dot(365,Monderotationz[0:3])
# #print("V2",V2)
# ## Quality metric
# # Calcul de l'intersection entre 1 point et 1 vecteur
# # print("Croisement ? ",point_on_line(P1,V1,[5,5,5]))
# # # Calcul de l'angle entre deux vecteur
# # print("Entre V1 et V2",angle_between_vectors(V1,V2))
# # Détermination de l'intersection entre deux vecteur
# #intersection_between_vectors(V1,V2,[5,5,5],[6,5,5])
# translation_one = [5,5,5]
# translation_two = [8,5,5]
# referential_one = define_rotation_translation(0,translation_one,"Z")
# referential_two = define_rotation_translation(90,translation_two,"Z")
# print("Ref1",referential_one)
# print("Ref2",referential_two)
# camera_one = Camera(referential_one[0],referential_one[1],referential_one[2],referential_one[3])
# camera_two = Camera(referential_two[0],referential_two[1],referential_two[2],referential_two[3])
# print("Cameraone",camera_one.vector,camera_one.origine)
# print("Cameratwo",camera_two.vector,camera_two.origine)
# angle_between_vectors(camera_one.vector,camera_two.vector)
# intersection_between_vectors(camera_one.vector,camera_two.vector,camera_one.origine,camera_two.origine)
# Point_intersect = intersection_between_vectors(camera_one.vector,camera_two.vector,camera_one.origine,camera_two.origine)
# #### Affichage des caméras
# fig = plt.figure(figsize=(6, 4))
# ax = fig.add_subplot(111,projection='3d')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# # set limit
# ax.set(xlim=(-10, 10), ylim=(-10, 10), zlim=(0, 20))
# # For camera 1 
# X1,Y1,Z1 = [camera_one.origine, camera_one.origine, camera_one.origine]
# U1,V1,W1 = [referential_one[0][0:3],referential_one[1][0:3],referential_one[2][0:3]]
# print("X1",X1,"Y1",Y1,"U1",U1,"V1",V1)
# print("U1 quiver : ",referential_one[0][0:3])
# ax.quiver(X1[0], Y1[1], Z1[2], U1[0], U1[1], U1[2], color="red",
#           normalize = True, length = 1, label = 'x')
# ax.quiver(X1[0], Y1[1], Z1[2], V1[0], V1[1], V1[2], color="red",
#           normalize = True, length = 1, label = 'y')
# ax.quiver(X1[0], Y1[1], Z1[2], W1[0], W1[1], W1[2], color="red",
#           normalize = True, length = 1, label = 'z')
# ax.plot([X1[0],camera_one.vector[0]],[Y1[1],camera_one.vector[1]],[Z1[2],camera_one.vector[2]],"r-")
# # For camera 2 
# X, Y, Z = [camera_two.origine,camera_two.origine,camera_two.origine]
# print("X",X,"Y",Y,"Z",Z)
# U, V, W = [referential_two[0][0:3],referential_two[1][0:3], referential_two[2][0:3]]
# print("U",U,"V",V,"W",W)
# #print("Monde rotation : ",monderotation[0][0:3])
# ax.quiver(X[0], Y[1], Z[2], U[0], U[1], U[2], color="blue",
#           normalize = True, length = 1, label = 'x')
# ax.quiver(X[0], Y[1], Z[2], V[0], V[1], V[2], color="blue",
#           normalize = True, length = 1, label = 'y')
# ax.quiver(X[0], Y[1], Z[2], W[0], W[1], W[2], color="blue",
#           normalize = True, length = 1, label = 'z')
# X_final = U[0]+V[0]+W[0]+365
# Y_final =  U[1]+V[1]+W[1]+365
# Z_final =  U[2]+V[2]+W[2]+365
# print("X_final",X_final)
# ax.plot([X[0],camera_two.vector[0]],[Y[1],camera_two.vector[1]],[Z[2],camera_two.vector[2]],"g-")
# ax.scatter(Point_intersect[0],Point_intersect[1],Point_intersect[2],'o')
# plt.show()
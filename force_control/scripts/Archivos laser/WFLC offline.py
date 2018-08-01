import numpy as np
import math
from matplotlib import pyplot as plt


#wflc     (y,     mu_0, mu_1, mu_b,  omega_0_init, M)
def wflc (senal, mu_0, mu_1, mu_b, frec_ini, M):

    # Periodo de actualizacin
    T =1/10

    # Inicializar variables
    sum_omega_0 = 0;
    X=[]
    W=[]
    omega_0=[]
    Vec=[]
    vec_error=[]
    sum_rec_vec=[]
    sum_vec=[]
    sum_vec_pend=[]

    sum_vec.append(sum_omega_0)


    for z in range (0,2*M):
        X.append([])
        W.append([])
        W[z].append(0)

    omega_0.append(frec_ini*2*np.pi*T)


    for k in range (0,len(senal)):

        for j in range (0,M):
            X[j].append(math.sin((j+1)*sum_omega_0))
            X[j+M].append(math.cos((j+1)*sum_omega_0))


        MUL_XW=0
        for z in range (0,len(X)):
            MUL_XW= MUL_XW + X[z][k]* W[z][k]


        error = senal[k] - MUL_XW - mu_b
        vec_error.append(error)


        sum_rec = 0

        for j in range (0,M):
            #print("entro 3")
            sum_rec = sum_rec + (j+1)*(W[j][k]*X[M+j][k] - W[M+j][k]*X[j][k])
            sum_rec_vec.append(sum_rec)


        omega_0_pred = omega_0[k] + 2*mu_0*error*sum_rec

        W_pred = []
        for z in range(0,len(W)):
            W_pred.append(W[z][k] + 2*mu_1*X[z][k]*error)

        if k < len(senal)-1:

            omega_0.append(omega_0_pred)
            for z in range(0,len(W)):
                W[z].append(W_pred[z])

            sum_omega_0    = sum_omega_0 + omega_0_pred;
            sum_vec.append(sum_omega_0)
            sum_vec_pend.append(abs(((sum_vec[k+1]-sum_vec[k]))*57.29578))

        else:
            break

    harmonics = []
    for z in range(0,len(W)):
        Vec=[]
        for j in range(0,len(W[z])):
            Vec.append(W[z][j] * X[z][j])
        harmonics.append(Vec)


    tremor=[]
    for z in range(0,len(harmonics[0])):
        suma = 0
        for j in range(0,len(harmonics)):
            suma =  suma + harmonics[j][z]
        tremor.append(suma)

    omega_0_Hz=[]
    for z in range (0,len(omega_0)):
        omega_0_Hz.append((omega_0[z]/(2*np.pi)/T))


    return tremor,X,W,omega_0_Hz,sum_vec,sum_vec_pend


senal=[]
x=[]
d=[]

for m in range (0,80000):
    tiempo= m/360
    x.append(tiempo)
    if m <= 30000:
        senal.append(0.5*math.sin(0.25*m/180*np.pi))
    elif m > 30000 and m <= 40000:
        senal.append(0.3*math.sin(0.5*m/180*np.pi))
    elif m > 40000 and m <= 50000:
        senal.append(0.7*math.sin(0.5*m/180*np.pi))
    elif m > 50000 and m <= 60000:
        senal.append(0.7*math.sin(1*m/180*np.pi))
    elif m > 60000 and m <= 70000:
        senal.append(0.4*math.sin(0.75*m/180*np.pi))
    else:
        senal.append(0.8*math.sin(0.75*m/180*np.pi))

plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Senal seno")
plt.legend()
plt.show()

tremor,X,W,omega_0_Hz,sum_vec,sum_vec_pend = wflc(senal, 0.00005,   0.004,    0,        0.028, 1)
#                                           wflc  (    y, mu_0,   mu_1, mu_b, omega_0_init,  M)
#tremor,X,W,omega_0_Hz,sum_vec,sum_vec_pend = wflc(senal,  0.7, 0.0027,    0,          1.5,  1)

for z in range (0,len(omega_0_Hz)):
    omega_0_Hz[z]= omega_0_Hz[z] * 44.4444444

#print(tremor)
print("-----")
#print(X[0])
print("-----")
#print(W)
#print(X[1])
print("-----")
#print(omega_0_Hz)

plt.plot(x, tremor, label="Senal tremor")
plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Senal estimada")
plt.legend()
plt.show()

plt.plot( X[0], label="X0")
plt.plot( X[1], label="X1")
#plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Componentes senosoidales")
plt.legend()
plt.show()


plt.plot( W[0], label="W0")
plt.plot( W[1], label="W1")
#plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Amplitudes")
plt.legend()
plt.show()

plt.plot(x, omega_0_Hz, label="Frecuencia")
plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Frecuancia")
plt.legend()
plt.show()

plt.plot(x, sum_vec, label="Sum Vec")
plt.plot(x, senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Frecuancia")
plt.legend()
plt.show()

plt.plot(sum_vec_pend, label="Sum Vec pend")
plt.plot(senal, label="Senal seno")
plt.xlabel("Eje X")
plt.ylabel("Eje Y")
plt.grid(True)
plt.title("Frecuancia")
plt.legend()
plt.show()

#print(sum_vec_pend)
print(sum_vec_pend[5000])
print(sum_vec_pend[15000])
print(sum_vec_pend[25000])

print(omega_0_Hz[5000])
print(omega_0_Hz[15000])
print(omega_0_Hz[25000])

## ENtradas
## senal: Valor actual de la entrada
## mu_0: peso para actualizar las freceuncia
## mu_1: Peso para actualizar las amplitud
## mu_b: pero para corregir la deriva
## M: Orden del filtro (cantidad de armonicos para la reconstruccion de la señal)
## sum_omega_0: Variable para calcular los armonicos
## W: lista de los pesos (amplitudes) de los armonicos iniciales
## omega_0: frecuencia inicial

def wflc (senal, mu_0, mu_1, mu_b, M, sum_omega_0, W, omega_0):

    T =1/10
    X=[]

    for j in range (0,M):
        X.append(math.sin((j+1)*sum_omega_0))
        X.append(math.cos((j+1)*sum_omega_0))

    MUL_XW=0
    for z in range (0,len(W)):
        MUL_XW = MUL_XW + X[z]* W[z]


    error = senal - MUL_XW - mu_b

    sum_rec = 0

    for j in range (0,M):
        sum_rec = sum_rec + (j+1)*( W[j]*X[M+j] - W[M+j]*X[j])


    omega_0_pred = omega_0 + 2*mu_0*error*sum_rec

    W_pred = []
    for z in range(0,len(W)):
        W_pred.append(W[z] + 2*mu_1*X[z]*error)

    omega_0s = omega_0_pred

    Ws=[]
    for z in range(0,len(W_pred)):
        Ws.append(W_pred[z])

    sum_omega_0_ant = sum_omega_0
    sum_omega_0s    = sum_omega_0 + omega_0_pred;

    harmonics = []
    for z in range(0,len(W)):
        harmonics.append(W[z] * X[z])


    suma = 0
    for z in range(0,len(harmonics)):
        suma =  suma + harmonics[z]

    tremor = suma
    omega_0_Hz = (omega_0s/(2*np.pi)/T)

    return tremor,X,Ws,omega_0_Hz,omega_0s,sum_omega_0s

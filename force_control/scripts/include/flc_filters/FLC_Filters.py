import numpy as np

def wflc(signal,params):
    params["X"] = np.zeros(2*params["M"])
    for r in range (0,params["M"]):
        params["X"][r] = np.sin((r+1)*params["sum_w0"])
        params["X"][r+params["M"]] = np.cos((r+1)*params["sum_w0"])

    y = np.dot(params["W"],params["X"])
    error = signal - y - params["Wb"]
    delta = 0
    i = 0
    for r in range (params["M"]):
        t1 = params["W"][r]*params["X"][params["M"]+r]
        t2 = params["W"][params["M"]+r]*params["X"][r]
        delta += (r+1)*(t1-t2)
    #print('error',error,'delta',delta)
    params["w0"] += 2*params["mu0"]*error*delta
    params["w0"] = abs(params["w0"])
    params["sum_w0"] += params["w0"]
    params["W"] += 2*params["mu"]*error*params["X"]
    #params["Wb"] += 2*params["mub"]*error
    #print(params["w0"]*params["fs"])
    tremor = np.dot(params["W"],params["X"]) + params["Wb"]
    return tremor,params

def flc(signal,params):
    params["X"] = np.zeros(2*params["M"])
    for r in range (params["M"]):
        params["X"][r] = np.sin((r+1)*params["w0"]*params["k"]/params["fs"])
        params["X"][r+params["M"]] = np.cos((r+1)*params["w0"]*params["k"]/params["fs"])

    y = np.dot(params["W"],params["X"])
    error = signal - y

    params["W"] += 2*params["mu"]*error*params["X"]
    tremor = np.dot(params["W"],params["X"])
    params["k"] += 1
    return tremor,params

def bmflc():
    pass

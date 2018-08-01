#-*- coding: utf-8 -*-

import usb.core
import usb.util
import time

class USB6009():
    def __init__(self):
        vendedor = 0x3923;
        producto = 0x717b;
        # Buscar dispocitivo
        self.dev = usb.core.find(idVendor=vendedor, idProduct=producto)
        # Esta disponible ?
        if self.dev is None:
            raise ValueError('El dispositivo no se encontro')
            exit()
        try:
            print("Configuracion")
            self.dev.ctrl_transfer(0x00,0x09,wValue = 1,wIndex = 0,data_or_wLength=0,timeout = 0)   #configuracion
            print("Configuracion Completada")
        except usb.core.USBError:
            print("Error de tiempo")

        # get an endpoint instance
        self.cfg = self.dev.get_active_configuration()
        self.intf = self.cfg[(0,0)]

        #   129 IN 0x81 configuracion de targeta
        self.ep0 = self.intf[0]
        assert self.ep0 is not None

        #   1 OUT 0x01 configuracion de targeta
        self.ep1 = self.intf[1]
        assert self.ep1 is not None

        #   130 IN 0x82 lectura de targeta
        self.ep2 = self.intf[2]
        assert self.ep2 is not None

        #   2 OUT 0x02 lectura de targeta
        self.ep3 = self.intf[3]
        assert self.ep3 is not None

        self.envio_d0 = [
            ('\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x02\x00\x00\x00\x04\x00\x00',1),
            ('\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x02\x00\x00\x00\x03\x00\x00',1), # diff
        #    ('\x00\x01\x00\x1c\x00\x18\x01\x10\x02\x02\x00\x00\x00\x00\x4e\x20\xff\xff\xb1\xe0\xfd\xfd\x00\x04\x00\x00\x00\x00',1), # diff
            ('\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x00\x00\x00\x00\x00\x00\x00',1), # all
            ('\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x00\x00\x00\x00\x00\x00\x01',1),# all
            ('\x00\x01\x00\x0c\x00\x08\x01\x13\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x15\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x18\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0f\x02\x03\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x03\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x10\x00\x0c\x00\x14\x02\x03\x00\x00\x00\x00\x00\x01',2), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0b\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x03\x00\x00',1)]

        self.envio_d1 = [
            ('\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x02\x00\x00\x00\x04\x00\x01',1),
            ('\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x02\x00\x00\x00\x03\x00\x00',1), # diff
        #    ('\x00\x01\x00\x1c\x00\x18\x01\x10\x02\x02\x00\x00\x00\x00\x4e\x20\xff\xff\xb1\xe0\xfd\xfd\x00\x04\x00\x00\x00\x00',1), # diff
            ('\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x00\x00\x00\x00\x00\x00\x00',1), # all
            ('\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x00\x00\x00\x00\x00\x00\x01',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x13\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x15\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x18\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0f\x02\x03\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x03\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x10\x00\x0c\x00\x14\x02\x03\x00\x00\x00\x00\x00\x01',2), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0b\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x00\x00\x00',1), # all
            ('\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x03\x00\x00',1)]

        self.read_inicial = [
            '\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x00\x00\x00\x00\x00\x00\x00',
            '\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x00\x00\x00\x00\x00\x00\x01',
            '\x00\x01\x00\x0c\x00\x08\x01\x13\x02\x00\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x15\x02\x00\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x18\x02\x00\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x0f\x02\x03\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x03\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x09\x02\x00\x00\x00']

        self.read_final = [
            '\x00\x01\x00\x0c\x00\x08\x01\x0b\x02\x00\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x00\x00\x00',
            '\x00\x01\x00\x0c\x00\x08\x01\x0c\x02\x03\x00\x00',]

        self.read_leer = '\x00\x01\x00\x10\x00\x0c\x00\x14\x02\x03\x00\x00\x00\x00\x00\x01'

    # Lee un dato del canal 0 en diferencial
    def leerD0(self):
        lectura = self.leer(self.envio_d0)
        con = len(lectura) - 1
        valor = 0
        while 0 <= con:
            valor = int(lectura[con]) * self.potencia(len(lectura) - con - 1) + valor
            con = con - 1
        valor = (valor - 32767) / 32767 * 20.9
        return valor
        
    # Lee un dato del canal 1 en diferencial
    def leerD1(self):
        lectura = self.leer(self.envio_d1)
        con = len(lectura) - 1
        valor = 0
        while 0 <= con:
            valor = int(lectura[con]) * self.potencia(len(lectura) - con - 1) + valor
            con = con - 1
        valor = (valor - 32767) / 32767 * 20.9
        return valor

    def potencia(self, exp):
        con = 0
        pro = 1
        while con < exp:
            pro = pro * 256
            con = con + 1
        return pro

    def cerrar(self):
        if self.dev:
            usb.util.dispose_resources(self.dev)
            usb.util.release_interface(self.dev, self.intf)
        self.dev = None

    #   lee el puerto y retorna un arreglo de hexa
    def leer(self, envio):
        con = 0
        lectura = -1;
        while len(envio) > con:
            self.ep1.write(envio[con][0], timeout=0)
            if envio[con][1] == 1:
                h = self.ep0.read(40, timeout=0)
            if envio[con][1] == 2:
                lectura = self.ep2.read(40, timeout=0)
            con = con + 1
        return lectura

    def reset(self):
        self.ep1.write('\x00\x01\x00\x0c\x00\x08\x01\x0d\x02\x00\x00\x00', timeout=0)
        h = self.ep0.read(40, timeout=0)

    def readchannel(self,chanels,reads=1,sec=0):
        retorno = []
        resultado = []
        print(0)
        # info[0] la cadena de configuración 1 de envio de canales
        # info[1] numero de canales validos
        info = self.crearcadena1(chanels)
        self.ep1.write(info[0], timeout=0)
        self.ep0.read(40, timeout=0)
        print(1)
        #cadena de configuración 2
        info2 = self.crearcad2(info[1])
        self.ep1.write(info2, timeout=0)
        self.ep0.read(40, timeout=0)
        print(2)
        # cadena de configuración 3
        info3 = self.crearcad3(info[1],chanels)
        self.ep1.write(info3, timeout=0)
        self.ep0.read(40, timeout=0)
        print(3)
        '''
        con = 0
        while len(inicial) > con:
            print(31)
            self.ep1.write(self.read_inicial[con], timeout=0)
            print(32)
            h = self.ep0.read(40, timeout=0)
            print(33)
            con = con + 1
        '''
        print(4)
        for x in range(0, reads):
            self.ep1.write(self.read_leer, timeout=0)
            retorno.append(self.ep2.read(40, timeout=0))
            if sec > 0:
                time.sleep(sec)
        print(5)
        '''
        con = 0
        while len(final) > con:
            self.ep1.write(self.read_final[con], timeout=0)
            self.ep0.read(40, timeout=0)
            con = con + 1
        '''
        print(6)
        for x in range(0, reads):
            cadena = retorno[x]
            atem = []
            for y in range(0, info[1]):
                valor = cadena[2 * y] * 256 + cadena[2 * y + 1]
                valor = valor * 6.37755 / 10000 - 31.16326
                atem.append(valor)
            resultado.append(atem)
        print(7)
        #usb.util.release_interface(self.dev, self.intf)
        #self.init()
        #self.reset()
        return resultado

# *******************************  Crear cadenas ********************************************

    # canales es un array con los numeros de los canales leer o un entero que es el canal a leer
    def crearcadena1(self,canales):
        cad = '';
        numcanal = 0
#        isinstance(canales, tuple)
        if isinstance(canales, int):
            cad = '\x00\x01\x00\x10\x00\x0c\x01\x0f\x02\x02\x00\x00\x00\x04\x00'
            cad = cad + self.elegir1(canales)
            numcanal = 1
        elif isinstance(canales, tuple):
            cad = '\x00\x01'
            var = self.crearparca1(canales)
            numcanal = var[1]
            cad = cad + self.logcad1(var[1])
            cad = cad + '\x01\x0f\x02\x02\x00\x00'
            cad = cad + self.numcad1(var[1])
            cad = cad + var[0]
            if var[1]==2 or var[1]==4 or var[1]==6 or var[1]==8:
                cad = cad + '\x00\x00'
        return (cad,numcanal)

    def elegir1(self,canal):

        if canal == 0:
            cad = '\x00'
        elif canal == 1:
            cad = '\x01'
        elif canal == 2:
            cad = '\x02'
        elif canal == 3:
            cad = '\x03'
        elif canal == 4:
            cad = '\x04'
        elif canal == 5:
            cad = '\x05'
        elif canal == 6:
            cad = '\x06'
        elif canal == 7:
            cad = '\x07'
        return cad

    def crearparca1(self,canales):
        con=0;
        cad =''
        if 0 in canales:
            cad = cad + '\x00\x00'
            con = con + 1
        if 1 in canales:
            cad = cad + '\x00\x01'
            con = con + 1
        if 2 in canales:
            cad = cad + '\x00\x02'
            con = con + 1
        if 3 in canales:
            cad = cad + '\x00\x03'
            con = con + 1
        if 4 in canales:
            cad = cad + '\x00\x04'
            con = con + 1
        if 5 in canales:
            cad = cad + '\x00\x05'
            con = con + 1
        if 6 in canales:
            cad = cad + '\x00\x06'
            con = con + 1
        if 7 in canales:
            cad = cad + '\x00\x07'
            con = con + 1
        return (cad,con)

    def logcad1(self, lon):
        cad = ''
        if lon == 8:
            cad = '\x00\x20\x00\x1c'
        elif lon > 5:
            cad = '\x00\x1c\x00\x18'
        elif lon > 3:
            cad = '\x00\x18\x00\x14'
        elif lon > 1:
            cad = '\x00\x14\x00\x10'
        return cad

    def numcad1(self,val):
        num = ''
        if val == 2:
            num = '\x00\x06'
        elif val == 3:
            num = '\x00\x08'
        elif val == 4:
            num = '\x00\x0a'
        elif val == 5:
            num = '\x00\x0c'
        elif val == 6:
            num = '\x00\x0e'
        elif val == 7:
            num = '\x00\x10'
        elif val == 8:
            num = '\x00\x12'
        return num

    def crearcad2(self,num):
        cad = ''
        if num == 1:
            cad = '\x00\x01\x00\x10\x00\x0c\x01\x0e\x02\x02\x00\x00\x00\x03\x02\x00'
        else:
            cad = '\x00\x01'
            if num > 6:
                cad = cad + '\x00\x18\x00\x14'
            elif num > 2:
                cad = cad + '\x00\x14\x00\x10'
            elif num==2:
                cad = cad + '\x00\x10\x00\x0c'

            cad = cad + '\x01\x0e\x02\x02\x00\x00'
            cad = cad + self.loncad2(num)
            cad = cad + '\x02\02'
            cad = cad + self.fincad2(num)
        return cad

    def loncad2(self,num):
        cad =''
        if num == 2:
            cad = '\x00\x04'
        elif num == 3:
            cad = '\x00\x05'
        elif num == 4:
            cad = '\x00\x06'
        elif num == 5:
            cad = '\x00\x07'
        elif num == 6:
            cad = '\x00\x08'
        elif num == 7:
            cad = '\x00\x09'
        elif num == 8:
            cad = '\x00\x0a'
        return cad

    def fincad2(self, num):
        cad = ''
        if num == 3:
            cad = '\x02\x00\x00\x00'
        elif num == 4:
            cad = '\x02\x02\x00\x00'
        elif num == 5:
            cad = '\x02\x02\x02\x00'
        elif num == 6:
            cad = '\x02\x02\x02\x02'
        elif num == 7:
            cad = '\x02\x02\x02\x02\x02\x00\x00\x00'
        elif num == 8:
            cad = '\x02\x02\x02\x02\x02\x02\x00\x00'
        return cad

    def crearcad3(self, num,canales):
        cad = '\x00\x01'
        if num == 1:
            cad = cad + '\x00\x1c\x00\x18'
        elif num == 2:
            cad = cad + '\x00\x1c\x00\x18'
        elif num == 3:
            cad = cad + '\x00\x20\x00\x1c'
        elif num == 4:
            cad = cad + '\x00\x20\x00\x1c'
        elif num == 5:
            cad = cad + '\x00\x24\x00\x20'
        elif num == 6:
            cad = cad + '\x00\x24\x00\x20'
        elif num == 7:
            cad = cad + '\x00\x28\x00\x24'
        elif num == 8:
            cad = cad + '\x00\x28\x00\x24'

        cad = cad + '\x01\x10\x02\x02\x00\x00\x00\x00\x27\x10\xff\xff\xd8\xf0\xfd\xfd'

        if num == 1:
            cad = cad + '\x00\x04'
        elif num == 2:
            cad = cad + '\x00\x06'
        elif num == 3:
            cad = cad + '\x00\x08'
        elif num == 4:
            cad = cad + '\x00\x0a'
        elif num == 5:
            cad = cad + '\x00\x0c'
        elif num == 6:
            cad = cad + '\x00\x0e'
        elif num == 7:
            cad = cad + '\x00\x10'
        elif num == 8:
            cad = cad + '\x00\x12'

        if isinstance(canales, int):
            cad = cad + '\x00' + self.elegir1(canales)
        elif isinstance(canales, tuple):
            tem = self.crearparca1(canales)
            cad = cad + tem[0]

        if num == 1 or num == 3 or num == 5 or num == 7:
            cad = cad + '\x00\x00'
        return cad

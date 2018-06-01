# coding: utf-8

# Importar modulos esenciales para el manejo del sensor y el motor
import RPi.GPIO as GPIO
import time
from pymultiwii import MultiWii

# Constante del macro que define el cambio en un parametro de motores
SET_MOTOR = 214

# Activacion del puerto serial al cual esta conectado el ESC del motor
serialPort = "/dev/ttyUSB0"
board = MultiWii(serialPort)

#Inicializacion del motor
power = 1080
Throttle = [power, 0, 0, 0]
board.sendCMD(8, SET_MOTOR, Throttle)

# Pin GPIO donde esta conectado el activador (entrada) del
# sensor HC-SR04.
TRIG = 11

# Pin GPIO donde esta conectado el eco (salida) del sensor
# HC-SR04.
ECHO = 13

# Distancia segmentada del cilindro - 27 cm hasta el Sensor
tamCilindro = 27

# Indicar que se usa la numeracion de pines en la Raspberry
GPIO.setmode(GPIO.BOARD)

# Establecer que TRIG es un canal de salida.
GPIO.setup(TRIG, GPIO.OUT)

# Establecer que ECHO es un canal de entrada.
GPIO.setup(ECHO, GPIO.IN)

# Modulo para obtener lectura del sensor ultrasonido
def leerDistancia():
	distancia = 0
	l = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	# Al aumentar el promedio de las lecturas incrementa la precision de la medicion
	for i in range(1, 20):
		# Retardo entre medidas 
		GPIO.output(TRIG, GPIO.LOW)
		# Al realizarse un promedio de 20 lecturas se tiene Tm = 2 s
		time.sleep(0.1)
		# Prender el pin activador por 10 microsegundos
		# y despues volverlo a apagar.
		GPIO.output(TRIG, GPIO.HIGH)
		time.sleep(0.00001)
		GPIO.output(TRIG, GPIO.LOW)
		# En este momento el sensor envia 8 pulsos
		# ultrasonicos de 40kHz y coloca su salida ECHO
		# en HIGH. Se debe detectar este evento e iniciar
		# la medicion del tiempo.
		while True:
			pulso_inicio = time.time()
			if GPIO.input(ECHO) == GPIO.HIGH:
				break
		# La salida ECHO se mantendra en HIGH hasta recibir
		# el eco reflejado por el obstaculo. En ese momento
		# el sensor pondra¡ ECHO en LOW y se debe terminar
		# la medicion del tiempo.
		while True:
			pulso_fin = time.time()
			if GPIO.input(ECHO) == GPIO.LOW:
				break
		# La medicion del tiempo es en segundos.
		duracion = pulso_fin - pulso_inicio
		# Calcular la distancia usando la velocidad del
		# sonido y considerando que la duracion incluye
		# la ida y vuelta.
		l[i] = (34300 * duracion) / 2
		distancia += l[i]
	
	distanciaProm = distancia/20
	distanciaNormalizada = tamCilindro - distanciaProm
	return distanciaNormalizada
	
# Definicion de las constantes del control PID
Kp = 4
Ki = 0
Kd = 0
# Este periodo de muestreo se tiene en cuenta a partir de la lectura del sensor
Tm = 2
# Inicializacion de las senales de control
up = 0
ui = 0
ud = 0
errorD = 0
# Ciclo de ejecucion del programa
while True:
	try:
		# Imprimir datos relevantes
		# Lectura inicial o estable
		D = leerDistancia()
		print("Posicion actual: %.2f cm" % D + "\t Potencia: %d\n" % power)
		cambiarDistancia = raw_input("Desea Modificar la posicion de la esfera? (Y/N): ")
		if cambiarDistancia == 'Y':
			posicion = float(input("Digite la posicion en cm: "))
		else:
			posicion = D
		# Bucle de control
		# Se desea implementar un control PID
		while D != posicion:
			D = leerDistancia()
			Ref = posicion
			# Senal de error
			e_k = Ref - D
			# Componente proporcional
			up = e_k*Kp
			# Componente integral
			ui += e_k*Ki*Tm
			# Componente derivativa
			errorD -= e_k
			ud = (e_k-errorD)*Kd*Tm
			# Senal de control
			u_k = up + ui + ud
			#power = u_k

			if D < Ref:
				power += up
				Throttle = [power, 0, 0, 0]
				board.sendCMD(8, SET_MOTOR, Throttle)

			elif D > Ref:
				power -= u_k
				Throttle = [power, 0, 0, 0]
				board.sendCMD(8, SET_MOTOR, Throttle)
			Error = (e_k/Ref)*100
			print("Referencia: %.2f cm" % posicion + "\tPosicion actual: %.2f cm" % D + "\tPotencia del motor: %d" % power + "\tError: %.2f" % Error + "%")	

	except KeyboardInterrupt:
		# Solo se especifica un valor de potencia al ser un motor
		Throttle = [1000, 0, 0, 0]
		board.sendCMD(8, SET_MOTOR, Throttle)
		print("\nApagando Motor...")
		print("Programa Terminado!")
		GPIO.cleanup()
		break

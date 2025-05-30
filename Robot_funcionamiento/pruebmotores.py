import machine
import time
from motores import Motores  # Asegúrate de que el archivo se llame motores.py

def main():
    robot = Motores()
    VELOCIDAD = 700  # Puedes ajustar la velocidad entre 0 y 1023

    try:
        print("Moviendo hacia adelante...")
        robot.mover_adelante(VELOCIDAD)
        time.sleep(5)

        print("Girando a la izquierda...")
        robot.girar_izquierda(VELOCIDAD)
        time.sleep(5)

        print("Girando a la derecha...")
        robot.girar_derecha(VELOCIDAD)
        time.sleep(5)

        print("Deteniendo motores.")
        robot.detener_motores()

    except KeyboardInterrupt:
        print("\nDeteniendo motores por interrupción...")
        robot.detener_motores()
    except Exception as e:
        print(f"Error crítico: {e}")
        robot.detener_motores()
        machine.reset()

# Asegúrate de llamar a la función principal
main()
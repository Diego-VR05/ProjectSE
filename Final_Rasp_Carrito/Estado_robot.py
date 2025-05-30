import paho.mqtt.client as mqtt
import json
from rich import print
from rich.console import Console
from rich.table import Table
from datetime import datetime

console = Console()

def on_connect(client, userdata, flags, rc):
    client.subscribe("robot/estado")
    print("[MONITOR] Suscrito a robot/estado")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Titulo general
        console.rule(f"[bold blue] - Estado del Robot - {timestamp}[/bold blue]")

        # Campos principales destacados
        status = data.get("status", "N/A")
        angle = data.get("angle", "N/A")
        distance = data.get("distance", "N/A")

        console.print(f"[cyan]- Status:[/cyan] [bold]{status}[/bold]")
        console.print(f"[yellow]- Angulo:[/yellow] [bold]{angle}[/bold]")
        console.print(f"[magenta]- Distancia:[/magenta] [bold]{distance} cm[/bold]\n")

        # Tabla con los campos restantes (excluyendo los ya mostrados)
        extras = {k: v for k, v in data.items() if k not in ["status", "angle", "distance"]}
        if extras:
            table = Table(title="[white]?? Otros Datos[/white]", show_header=True, header_style="bold green")
            table.add_column("Campo", style="cyan", no_wrap=True)
            table.add_column("Valor", style="magenta")

            for key, value in extras.items():
                table.add_row(str(key), str(value))

            console.print(table)

        # Registro en archivo (todo el diccionario original)
        with open("registro_estado.txt", "a") as f:
            f.write(f"{timestamp} - {json.dumps(data)}\n")

    except Exception as e:
        console.print(f"[red][ERROR][/red] Mensaje malformado: {e}")


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("172.20.10.2", 1883)
print("[MONITOR] Esperando estados de robots...")
client.loop_forever()

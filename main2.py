import tkinter as tk
import tkinter as tk
import serial
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import queue

# Utworzenie połączenia z portem szeregowym
ser = serial.Serial('COM5', 115200, timeout=1)

# Globalne zmienne do przechowywania ostatnich ustawień PID
last_kp = 0.1
last_ki = 0.2
last_kd = 0.1

# Kolejka do przechowywania danych z wątku czytającego
data_queue = queue.Queue()

def read_serial():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            values = line.split(',')
            if len(values) == 3:
                try:
                    val1, val2, val3 = [float(v) for v in values]
                    data_queue.put((val1, val2, val3))
                except ValueError:
                    pass

def update_plot():
    while not data_queue.empty():
        val1, val2, val3 = data_queue.get()
        data1.append(val1)
        data2.append(val2)
        data3.append(val3)
    ax.clear()
    ax.plot(data1, label='Data 1')
    ax.plot(data2, label='Data 2')
    ax.plot(data3, label='Data 3')
    ax.legend(loc='upper left')
    canvas.draw()
    root.after(100, update_plot)
# Tkinter Functions
def send_rpm():
    try:
        rpm_value = int(rpm_entry.get())
        if 2000 <= rpm_value <= 15000:
            command = f"SET:{rpm_value}:{last_kp}:{last_ki}:{last_kd}\n"
            ser.write(command.encode())
            status_label.config(text=f"Wysłano: RPM={rpm_value}")
        else:
            status_label.config(text="Wartość RPM poza zakresem 2000-15000!")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawną wartość liczbową dla RPM")

def send_pid():
    global last_kp, last_ki, last_kd
    try:
        last_kp = float(kp_entry.get())
        last_ki = float(ki_entry.get())
        last_kd = float(kd_entry.get())
        # Tutaj możesz zaimplementować logikę wysyłania wartości PID
        # jeżeli jest taka potrzeba, na przykład:
        # command = f"SET_PID:{last_kp}:{last_ki}:{last_kd}\n"
        # ser.write(command.encode())
        status_label.config(text=f"Wysłano: Kp={last_kp}, Ki={last_ki}, Kd={last_kd}")
    except ValueError:
        status_label.config(text="Proszę wprowadzić poprawne wartości liczbowe dla PID")

# Inicjalizacja interfejsu użytkownika Tkinter
root = tk.Tk()
root.title("RPM i Regulator PID")

# Inicjalizacja list do przechowywania danych
data1 = []
data2 = []
data3 = []

# Ramka dla RPM
frame_rpm = tk.Frame(root)
frame_rpm.pack(padx=10, pady=10)

tk.Label(frame_rpm, text="RPM (2000-15000):").pack(side=tk.LEFT)
rpm_entry = tk.Entry(frame_rpm)
rpm_entry.pack(side=tk.LEFT)

tk.Button(frame_rpm, text="Potwierdź RPM", command=send_rpm).pack(side=tk.LEFT)

# Ramka dla PID
frame_pid = tk.Frame(root)
frame_pid.pack(padx=10, pady=10)

tk.Label(frame_pid, text="Kp:").pack(side=tk.LEFT)
kp_entry = tk.Entry(frame_pid)
kp_entry.insert(0, "0.1")  # Ustawienie domyślnej wartości Kp
kp_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Ki:").pack(side=tk.LEFT)
ki_entry = tk.Entry(frame_pid)
ki_entry.insert(0, "0.2")  # Ustawienie domyślnej wartości Ki
ki_entry.pack(side=tk.LEFT)

tk.Label(frame_pid, text="Kd:").pack(side=tk.LEFT)
kd_entry = tk.Entry(frame_pid)
kd_entry.insert(0, "0.1")  # Ustawienie domyślnej wartości Kd
kd_entry.pack(side=tk.LEFT)

# Etykieta wyświetlająca status
status_label = tk.Label(root, text="Status: Nie wysłano danych")
status_label.pack(pady=10)

tk.Button(frame_pid, text="Wyślij Parametry PID", command=send_pid).pack()

# Inicjalizacja wykresu
fig, ax = plt.subplots()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack()

# Uruchomienie wątku odczytu danych
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# Uruchomienie aktualizacji wykresu
update_plot()

root.mainloop()